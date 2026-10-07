#include "speed_sensor.h"
#include "debug_tee/debug_tee.h"

// 載入設定和共享變數
volatile speed_sensor settings{};

// 宣告全域指標，用來儲存這個計數器的控制代碼
mcpwm_cap_timer_handle_t cap_timer = NULL;
mcpwm_cap_channel_handle_t cap_chan = NULL;

// RTOS系統時間(用於延遲)
TickType_t xLastWakeTime = xTaskGetTickCount();

// 算完脈衝數後用於呼叫PID任務的旗標(實體位於motor_PID.cpp中)
extern TaskHandle_t xPIDTaskHandle;

// ★記憶體保護★：本檔案對 settings 的存取一律透過 settings.h 提供的介面——
// 單一欄位讀寫用 speed_get_xxx()/speed_set_xxx()；環形緩衝(ISR寫入/任務端快照)
// 用專屬的 speed_capture_*() 系列函式；需要「一次原子讀寫好幾個相關欄位」的狀態
// 轉換(例如 trip_fault、階段切換)則直接用 SettingsLockGuard 包住整段。
// 這些全部共用 settings.h 內宣告的 g_speed_mux，取代舊版自行宣告的 my_mutex。

uint32_t cap_buffer_Snapshot[(1 << buf_idx_bit)]; // 脈衝時間戳快照

// 編譯時先計算出常數，就不用每次MCU還要再算一遍
// MCPWM Capture 預設時鐘 80MHz → RPM = (80e6 * 60) / 一整圈 ticks
constexpr float RPM_CONSTANT = (80000000ULL * 60ULL);
constexpr uint8_t IDX_MASK = (1 << buf_idx_bit) - 1; // 環形遮罩

// 兩次觸發間隔若小於此 tick 數(換算轉速遠超 RPM_RUNAWAY_MAX)，視為彈跳/雜訊直接丟棄
constexpr uint32_t MIN_EDGE_TICKS =
    (uint32_t)(RPM_CONSTANT / (RPM_RUNAWAY_MAX * (float)EDGES_PER_REV * MIN_EDGE_TICKS_SAFETY_RPM_MULT));

// 任務端狀態(不需給 ISR 使用)
static uint32_t last_edge_count = 0;           // 上次已處理到的絕對邊緣數
static float last_valid_rpm = 0.0f;            // 最近一次可信轉速(正常模式保持用)
static uint32_t last_edge_ms = 0;              // 最近一次邊緣到達的 millis
static uint16_t settle_samples_local = 0;      // 開環固定 PWM 後，已觀察到的同位置整圈樣本數(本地副本)
static uint32_t no_pulse_pwm_ms = 0;           // 有 PWM 卻無脈衝的累計時間(失控保護，全程持續累計)
static uint32_t last_protect_ms = 0;
static uint32_t probe_arm_edge = 0;            // probe 請求當下的邊緣計數(只收之後的新資料)
static bool probe_armed = false;
static float speed_filter_ema = 0.0f;          // 同位置整圈轉速(rpm_same)的 EMA 濾波值(給 keep_rpm/PID 使用)
static bool speed_filter_inited = false;       // 濾波器是否已灌入初值
static uint8_t speed_outlier_reject_streak = 0; // 連續拒絕離群樣本次數(避免真實大階躍被永遠擋住)

// 時間差變數
uint32_t time_diff;

// 將本地就緒樣本數同步到共享結構(供除錯/上位機觀察)
static void publish_settle_progress()
{
  speed_set_settle_samples(settle_samples_local);
}

// 先宣告回乎函數的原型
static bool mcpwm_cap_cb(mcpwm_cap_channel_handle_t cap_chan,
                         const mcpwm_capture_event_data_t *edata,
                         void *user_data);

// 重置「開環固定 PWM 後的就緒觀察」進度(尚未固定 PWM，或固定後又真停轉時呼叫)
static void reset_settle_progress()
{
  settle_samples_local = 0;
  speed_filter_inited = false; // 重新起動時，EMA 濾波器也要跟著重新起算
  speed_outlier_reject_streak = 0;
  publish_settle_progress();
}

// 置位失控保護(與 motor_PID 共用；輸出切斷由 PID 任務執行)
// 一次改動多個彼此相關的欄位(fault/phase/旗標...)，整段用同一把鎖包住，
// 確保其他任務(motor_PID/main)不會讀到「只更新一半」的中間狀態。
static void trip_fault(uint8_t code)
{
  {
    SettingsLockGuard lock(g_speed_mux);
    settings.fault = true;
    settings.fault_code = code;
    settings.const_speed_ready = false;
    settings.ol_hold = false;
    settings.ol_probe_request = false;
    settings.ol_probe_ready = false;
    settings.init_phase = SPEED_PHASE_FAULT;
    settings.speed_valid = false;
    settings.speed_stable = false; // 異常時對外穩調旗標必須為 false
    // 立刻歸零，避免觸發瞬間的異常值(如彈跳造成的離譜轉速)殘留污染後續輸出
    settings.now_speed = 0.0f;
  }
  // 以下為本檔案內部的本地狀態(非共享欄位)，不需要鎖保護
  last_valid_rpm = 0.0f;
  speed_filter_inited = false;
  speed_outlier_reject_streak = 0;
}

// 開環固定 PWM(LEARNING)後，每來一個新的「同位置整圈」樣本就累計一次觀察進度，
// 達到 READY_SETTLE_MIN_SAMPLES 後才允許離開 LEARNING 進入 READY / 定速
static void count_settle_sample()
{
  if (settle_samples_local < 0xFFFF)
  {
    settle_samples_local++;
  }
  publish_settle_progress();
}

// 同位置整圈量測
static float rpm_from_same_slot(const uint32_t *snap, uint8_t now_idx)
{
  const uint8_t last_idx = (uint8_t)((now_idx - EDGES_PER_REV) & IDX_MASK);
  time_diff = snap[now_idx] - snap[last_idx];
  if (time_diff == 0)
  {
    return 0.0f;
  }
  return RPM_CONSTANT / (float)time_diff;
}

// 初始化測速補丁：
// 低轉速時一般輪詢容易讀到「保持舊值/空窗」，因此在 ol_probe_request 期間
// 只接受「請求當下之後」出現的第一筆新同位置資料作為當前可信轉速
static void try_latch_init_probe(float rpm_same, bool got_new_same_slot, uint32_t edge_count_snap)
{
  if (!speed_get_ol_probe_request())
  {
    probe_armed = false;
    return;
  }
  if (speed_get_ol_probe_ready())
  {
    return;
  }

  // 請求剛升起：鎖存當下邊緣數，之後只收更新的資料
  if (!probe_armed)
  {
    probe_arm_edge = edge_count_snap;
    probe_armed = true;
    return; // 本週期不採用(可能仍是 settle 前舊邊緣)
  }

  if (!got_new_same_slot || rpm_same <= 0.0f)
  {
    return;
  }
  if (edge_count_snap <= probe_arm_edge)
  {
    return;
  }

  // 這 3 個欄位代表「這次鎖存」同一個結果，整段上鎖確保三者同時對外可見
  {
    SettingsLockGuard lock(g_speed_mux);
    settings.ol_probe_rpm = rpm_same;
    settings.ol_probe_ready = true;
    settings.now_speed = rpm_same;
    settings.speed_valid = true;
  }
  last_valid_rpm = rpm_same;
  // probe 已通過可讀門檻：立刻把 EMA 對齊這筆同位置整圈，否則 LEARNING 期間
  // 離群過濾會拿爬升中的低 EMA 把真實 ~1150+ RPM 當雜訊丢掉，keep_rpm 就被鎖在一半。
  speed_filter_ema = rpm_same;
  speed_filter_inited = true;
  speed_outlier_reject_streak = 0;
  probe_armed = false;
}

// 失控保護檢查(測速與輸出協調)
static void runaway_protect(uint32_t now_ms, bool has_new_edge)
{
  if (speed_get_fault())
  {
    return;
  }

  // 飛車：轉速超限。now_speed 與 speed_valid 視為一組，整段上鎖讀取，
  // 避免拿到「新轉速配舊有效旗標」這種不曾真實存在過的組合去誤判。
  bool valid_now;
  float speed_now;
  {
    SettingsLockGuard lock(g_speed_mux);
    valid_now = settings.speed_valid;
    speed_now = settings.now_speed;
  }
  if (valid_now && speed_now > RPM_RUNAWAY_MAX)
  {
    trip_fault(FAULT_RUNAWAY_RPM);
    return;
  }

  // ★馬達通電看門狗(單一機制，全程覆蓋)：只要有 PWM 輸出(從剛啟動的第一階
  // 開環升速，到就緒觀察/自動調參/定速運行，全程都算)，就持續檢查「多久沒有
  // 新脈衝」；只要連續 NO_PULSE_TIMEOUT_MS 沒有新脈衝就立即鎖定，PWM=0 時
  // 累計時間歸零。全程只有這一個閾值、沒有分段/分階，不會有空窗。
  const uint32_t dt = (last_protect_ms == 0) ? speed_get_read_space() : (now_ms - last_protect_ms);
  last_protect_ms = now_ms;

  const uint16_t pwm_cmd_now = speed_get_ol_pwm_cmd();
  const bool protect_stall = (pwm_cmd_now > 0);

  if (protect_stall && !has_new_edge)
  {
    no_pulse_pwm_ms += dt;
    if (no_pulse_pwm_ms >= (uint32_t)NO_PULSE_TIMEOUT_MS)
    {
      Serial.printf("[SPD] watchdog: no pulse for %ums while pwm=%u\n",
                    (unsigned)NO_PULSE_TIMEOUT_MS, (unsigned)pwm_cmd_now);
      trip_fault(FAULT_NO_PULSE_TIMEOUT);
      return;
    }
  }
  else
  {
    no_pulse_pwm_ms = 0;
  }
}

// 依就緒觀察進度更新階段旗標(階梯/probe 由 motor_PID 主導)
// 整個函式代表「一次階段判定」，好幾個欄位彼此牽動，故整段用同一把鎖包住，
// 確保其他任務讀到的 phase/const_speed_ready/speed_stable 永遠是同一輪判定的結果。
static void update_phase_after_sample(bool got_new_same_slot)
{
  SettingsLockGuard lock(g_speed_mux);

  if (settings.fault)
  {
    settings.init_phase = SPEED_PHASE_FAULT;
    settings.const_speed_ready = false;
    settings.speed_stable = false;
    return;
  }

  // 固定 PWM 後進入學習
  if (settings.ol_hold)
  {
    if (settle_samples_local >= READY_SETTLE_MIN_SAMPLES)
    {
      // 測速就緒；若 motor 正在/已調參，保留其 PID_TUNE / PID_RUN 階段不被覆蓋。
      // ★PID_PAUSED(量測序列請求暫停，見 motor_PID.cpp)也要一併保留：
      // 否則本函式每輪都會把 motor_PID 剛設好的 PID_PAUSED 搶回 READY、
      // 把 const_speed_ready 搶回 true，跟上面「真停轉」分支剛置的 false
      // 互踩、造成暫停狀態每 15ms 閃爍一次(不影響安全——motor_PID 的
      // pause 判斷完全不看這兩個欄位——但會讓 OLED/上位機顯示閃爍、誤導使用者)。
      if (settings.init_phase != SPEED_PHASE_PID_TUNE &&
          settings.init_phase != SPEED_PHASE_PID_RUN &&
          settings.init_phase != SPEED_PHASE_PID_PAUSED)
      {
        settings.init_phase = SPEED_PHASE_READY;
      }
      if (settings.init_phase != SPEED_PHASE_PID_PAUSED)
      {
        settings.const_speed_ready = true;
        settings.rpm_stable = true;
      }
    }
    else
    {
      settings.init_phase = SPEED_PHASE_LEARNING;
      settings.const_speed_ready = false;
      settings.speed_stable = false;
    }
    return;
  }

  // 尚未 hold：由 motor 的 settle / probe 旗標反映階段
  if (settings.ol_probe_request && !settings.ol_probe_ready)
  {
    settings.init_phase = SPEED_PHASE_PROBE;
  }
  else if (!settings.ol_hold)
  {
    settings.init_phase = SPEED_PHASE_STEP_UP;
  }

  settings.const_speed_ready = false;
  settings.speed_stable = false;
  (void)got_new_same_slot;
}

void speed_sensor_init(void *pvParameters)
{
  // 硬體就緒；就緒觀察進度歸零，等開環停在可讀轉速後再開始累計
  reset_settle_progress();
  {
    SettingsLockGuard lock(g_speed_mux);
    settings.min_measurable_rpm = RPM_INIT_READABLE;
    settings.init_phase = SPEED_PHASE_STEP_UP;
    settings.const_speed_ready = false;
    settings.rpm_stable = false;
    settings.speed_stable = false;
    settings.fault = false;
    settings.fault_code = FAULT_NONE;
    settings.ol_hold = false;
    settings.ol_probe_request = false;
    settings.ol_probe_ready = false;
    settings.ol_probe_rpm = 0.0f;
  }
  last_edge_ms = millis();
  last_protect_ms = last_edge_ms;
  no_pulse_pwm_ms = 0;

  // 配置時鐘
  mcpwm_capture_timer_config_t timer_config = {};
  timer_config.clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT; // 預設 80MHz
  timer_config.group_id = 0;
  mcpwm_new_capture_timer(&timer_config, &cap_timer); // 啟用設定

  // 配置讀取相關設定
  mcpwm_capture_channel_config_t chan_config = {};
  chan_config.gpio_num = SPEED_SENSOR_PIN;
  chan_config.prescale = 1;                                      // 不分頻
  chan_config.flags.pos_edge = 1;                                // 啟用上升沿
  chan_config.flags.neg_edge = 1;                                // 啟用下降沿 (雙沿觸發)
  mcpwm_new_capture_channel(cap_timer, &chan_config, &cap_chan); // 啟用設定

  // 註冊回呼函式 (ISR)
  mcpwm_capture_event_callbacks_t cbs = {};
  cbs.on_cap = mcpwm_cap_cb; // 指定你的 ISR 函式名稱
  mcpwm_capture_channel_register_event_callbacks(cap_chan, &cbs, NULL);

  // 啟用並開始測量
  mcpwm_capture_channel_enable(cap_chan);
  mcpwm_capture_timer_enable(cap_timer);
  mcpwm_capture_timer_start(cap_timer);

  // 開始計數
  while (1)
  {
    // 延時(時間取決於設定值，位於settings.h中)
    xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(speed_get_read_space()));

    // 已故障：維持安全值，不再嘗試計算轉速(轉子可能仍在自然減速，避免殘留雜訊污染輸出)，
    // 待重新開機才會重新初始化
    if (speed_get_fault())
    {
      {
        SettingsLockGuard lock(g_speed_mux);
        settings.now_speed = 0.0f;
        settings.speed_valid = false;
      }
      last_valid_rpm = 0.0f;
      speed_filter_inited = false;
      speed_outlier_reject_streak = 0;
      if (xPIDTaskHandle != NULL)
      {
        xTaskNotifyGive(xPIDTaskHandle);
      }
      continue;
    }

    // 先做快照(ISR 寫入/任務端讀取這組耦合欄位的唯一合法路徑，見 settings.h)
    uint32_t edge_count_snap = 0;
    uint8_t count_number_snap = 0;
    speed_capture_snapshot(cap_buffer_Snapshot, (uint8_t)(1 << buf_idx_bit),
                           count_number_snap, edge_count_snap);

    const uint8_t now_idx = (uint8_t)((count_number_snap - 1) & IDX_MASK);
    const uint32_t now_ms = millis();
    const bool has_new_edge = (edge_count_snap != last_edge_count);

    float rpm_same = 0.0f;
    bool got_new_same_slot = false;

    if (has_new_edge)
    {
      if (edge_count_snap >= (uint32_t)(EDGES_PER_REV + 1))
      {
        rpm_same = rpm_from_same_slot(cap_buffer_Snapshot, now_idx);
        if (rpm_same > 0.0f)
        {
          got_new_same_slot = true;

          // 初始化 probe 補丁：只在請求後鎖存第一筆「新」同位置轉速
          try_latch_init_probe(rpm_same, got_new_same_slot, edge_count_snap);

          // 輸出轉速策略：一律採用「同位置整圈」量測(rpm_same)，只加一層輕量 EMA 平滑。
          // 「同位置整圈」量測是用同一個物理光柵位置量測整整一圈後的時間，天生就把每格的
          // 加工誤差完全抵銷，精度與原始需求(用同位置保證精度)完全一致；相較於曾經嘗試過
          // 『單格(1/4 圈)』瞬時補償量測，時間基準短了 4 倍，會把 ISR/彈跳/時鐘量化的相對
          // 抖動、以及定速時馬達真實的瞬時角速度漲落放大暴露出來，配合本機台 PWM→RPM
          // 增益極高(約每 1 count 對應 10+ RPM)，這種被放大的雜訊經比例項直接回饋，正是
          // 先前 PID_RUN 階段轉速劇烈震盪、久久無法收斂的根本原因之一，故不採用。
          float rpm_for_filter = rpm_same;
          const uint8_t phase_now = speed_get_init_phase();
          // 離群拒絕只在閉環定速做：開環爬升／就緒觀察／sTune 階躍時轉速本來就會一次跳數百 RPM，
          // 若這時把真值當雜訊丢掉，EMA 會卡在半途，keep_rpm 也會跟著鎖錯。
          if (speed_filter_inited && phase_now == SPEED_PHASE_PID_RUN)
          {
            const float delta = fabsf(rpm_same - speed_filter_ema);
            const float rel_limit = fmaxf(speed_filter_ema, 1.0f) * (float)SPEED_OUTLIER_MAX_RATIO;
            const float limit = fmaxf((float)SPEED_OUTLIER_MAX_ABS_RPM, rel_limit);
            if (delta > limit)
            {
              if (speed_outlier_reject_streak < SPEED_OUTLIER_REJECT_MAX)
              {
                speed_outlier_reject_streak++;
                Serial.printf("[EVT] speed outlier reject raw=%.1f ema=%.1f d=%.1f streak=%u\n",
                              (double)rpm_same, (double)speed_filter_ema, (double)delta,
                              (unsigned)speed_outlier_reject_streak);
                rpm_for_filter = speed_filter_ema; // 本筆視為雜訊，維持上一筆 EMA
              }
              else
              {
                // 連續多筆都偏離：可能是真實階躍，但仍限制單步對 EMA 的影響，避免雜訊直接
                // 污染 now_speed→PID 造成假性轉速抽搐與 speed_stable 永遠無法成立
                speed_outlier_reject_streak = 0;
                const float max_step = limit * 0.35f;
                if (delta > max_step)
                {
                  rpm_for_filter = speed_filter_ema +
                                   copysignf(max_step, rpm_same - speed_filter_ema);
                }
              }
            }
            else
            {
              speed_outlier_reject_streak = 0;
            }
          }

          if (!speed_filter_inited)
          {
            speed_filter_ema = rpm_for_filter;
            speed_filter_inited = true;
          }
          else
          {
            speed_filter_ema += SPEED_FILTER_ALPHA * (rpm_for_filter - speed_filter_ema);
          }
          const float rpm_out = speed_filter_ema;

          // ★診斷探針★：定速運行中若單一週期轉速跳變異常大，印一行事件方便回頭定位根因，
          // 幫助分辨是感測端(邊緣漏跳/雜訊/debounce誤丟)還是馬達物理上真的轉速突變。
          // 以本機台 process gain(~10RPM/count)搭配 15ms 週期估計，正常動態下單週期變化
          // 很難超過此值，故此閾值僅用於標記「異常大」事件，不影響任何控制邏輯。
          // 必須用「更新前」的 last_valid_rpm 做比較基準，且要在 rpm_out 寫回前判斷。
          if (phase_now == SPEED_PHASE_PID_RUN && last_valid_rpm > 1.0f)
          {
            const float jump = fabsf(rpm_out - last_valid_rpm);
            if (jump > SPEED_JUMP_WARN_RPM)
            {
              Serial.printf("[EVT] speed jump %.1f -> %.1f (d=%.1f) edges=%lu gap_ms=%lu rawSame=%.1f\n",
                            (double)last_valid_rpm, (double)rpm_out, (double)jump,
                            (unsigned long)edge_count_snap,
                            (unsigned long)(now_ms - last_edge_ms),
                            (double)rpm_same);
            }
          }

          // probe 等待期間：未鎖存前不要用「保持舊值」冒充當前轉速
          // (讀 2 個相關旗標 + 條件式寫 2 個欄位視為一組決策，整段上鎖確保自洽)
          bool did_publish_speed = false;
          {
            SettingsLockGuard lock(g_speed_mux);
            if (!(settings.ol_probe_request && !settings.ol_probe_ready))
            {
              settings.now_speed = rpm_out;
              settings.speed_valid = true;
              did_publish_speed = true;
            }
          }
          if (did_publish_speed)
          {
            last_valid_rpm = rpm_out;
          }

          speed_capture_mark_used(now_idx);

          // 固定開環 PWM 後，累計「就緒觀察」樣本數(見 count_settle_sample())
          if (speed_get_ol_hold() && speed_get_init_phase() == SPEED_PHASE_LEARNING)
          {
            count_settle_sample();
          }
        }
      }

      last_edge_count = edge_count_snap;
      last_edge_ms = now_ms;
    }
    else
    {
      // 無新邊緣
      const bool probe_waiting = speed_get_ol_probe_request() && !speed_get_ol_probe_ready();
      if (probe_waiting)
      {
        // 初始化補丁：probe 視窗內不回報過期保持值，避免誤判「已經夠快」
        speed_set_speed_valid(false);
        // now_speed 維持上次顯示即可，決策以 ol_probe_ready 為準
      }
      else if ((now_ms - last_edge_ms) <= SPEED_STALL_TIMEOUT_MS)
      {
        // ★診斷探針★：定速運行中若一段時間沒有新邊緣(但還沒到 stall 判定)，印一次事件，
        // 方便回頭比對是否與轉速跳變事件同時發生(感測端訊號中斷的直接證據)
        if (speed_get_init_phase() == SPEED_PHASE_PID_RUN)
        {
          const uint32_t gap = now_ms - last_edge_ms;
          if (gap == SPEED_EDGE_GAP_WARN_MS) // 剛跨過門檻時只印一次，避免每輪刷屏
          {
            Serial.printf("[EVT] edge gap warn %lums (last_rpm=%.1f)\n",
                          (unsigned long)gap, (double)last_valid_rpm);
          }
        }
        SettingsLockGuard lock(g_speed_mux);
        settings.now_speed = last_valid_rpm;
        settings.speed_valid = (last_valid_rpm > 0.0f);
      }
      else
      {
        {
          SettingsLockGuard lock(g_speed_mux);
          settings.now_speed = 0.0f;
          settings.speed_valid = false;
        }
        last_valid_rpm = 0.0f;

        // 就緒觀察未完成就真停轉：清掉進度，下次開環重來
        const bool ready_now = speed_get_const_speed_ready();
        if (!ready_now && settle_samples_local < READY_SETTLE_MIN_SAMPLES)
        {
          reset_settle_progress();
          SettingsLockGuard lock(g_speed_mux);
          settings.ol_hold = false;
          settings.rpm_stable = false;
        }
        if (ready_now)
        {
          speed_set_const_speed_ready(false);
        }
        speed_set_speed_stable(false); // 停轉/不可信 → 未穩調
      }
    }

    update_phase_after_sample(got_new_same_slot);
    runaway_protect(now_ms, has_new_edge);

    // 每個測速週期喚醒 PID
    if (xPIDTaskHandle != NULL)
    {
      xTaskNotifyGive(xPIDTaskHandle);
    }
  }
}

// 中斷服務函式 (Callback)
static bool IRAM_ATTR mcpwm_cap_cb(mcpwm_cap_channel_handle_t cap_chan,
                                   const mcpwm_capture_event_data_t *edata,
                                   void *user_data)
{
  // 去彈跳：與上次觸發間隔小於 MIN_EDGE_TICKS 視為機械彈跳/雜訊，直接丟棄不計入，
  // 避免這種極短的假邊緣污染「同位置整圈」的時間基準(進而誤觸發飛車保護)
  static uint32_t last_cap_ticks = 0;
  static bool has_last_cap = false;
  const uint32_t now_ticks = edata->cap_value;
  if (has_last_cap && ((now_ticks - last_cap_ticks) < MIN_EDGE_TICKS))
  {
    return false;
  }
  last_cap_ticks = now_ticks;
  has_last_cap = true;

  // 寫入環形緩衝(buf_idx/cap_buffer/Timestamp_state/edge_count 整組原子更新，
  // 且與任務端的 speed_capture_snapshot() 互斥，見 settings.h 說明)
  speed_capture_push_edge(edata->cap_value);
  return false;
}

void speed_sensor_start()
{
  // 任務設定/分配
  xTaskCreatePinnedToCore(speed_sensor_init,
                          "speed_sensor_start",
                          4096,
                          NULL,
                          RTOS_SPEED_SENSOR_LEVEL,
                          NULL,
                          0);
}
