#include "motor_PID.h"
#include <sTune.h>
#include "debug_tee/debug_tee.h"

// 算完脈衝數後用於呼叫PID任務的旗標實體
TaskHandle_t xPIDTaskHandle = NULL;
// 載入設定
volatile motor_PID PID_settings{};
// 快照用變數
static float pid_output;          // 輸出到驅動主動力馬達PWM的定時器值
static float pid_input_bridge;    // 輸入到PID模組的(RPM)
static float pid_setpoint_bridge; // 要維持的值(RPM)
// PWM最大值限制
// ★注意：這裡是 C++ 全域變數的靜態初始化，發生在 setup()/所有 RTOS 任務啟動之前，
// 當下只有目前這個執行緒在跑、不會有人同時寫入 PID_settings，故直接讀欄位即可，
// 刻意不透過 pid_get_pwm_res()：跨編譯單元(settings.cpp)的鎖初始化順序沒有保證
// 一定早於這裡執行，為了不要引入不必要的「靜態初始化順序」風險，這一行維持原樣。
float max_pwm_value = (float)((1 << PID_settings.pwm_res) - 1);

// 階梯開環狀態
static uint16_t step_pwm = 0;       // 目前階梯 duty
static uint32_t settle_start_ms = 0; // 本次升階後開始等待的時間
static bool settling = false;       // 是否處於「升階後等待穩定」
static bool step_applied = false;   // 本階 PWM 是否已寫出

// sTune 自動調參
static bool tuner_configured = false;
static float tune_kp = 0.0f;
static float tune_ki = 0.0f;
static float tune_kd = 0.0f;
static uint32_t tune_sample_hits = 0;   // 進入 sample 狀態次數(進度估計)
static uint32_t tune_last_print_ms = 0; // 上次進度列印時間
static float tune_out_start = 0.0f;
static float tune_out_step = 0.0f;
static float tune_input_span = 0.0f;    // 本次調參用的 sTune 輸入全幅(換算真實單位增益時要用)
static uint32_t tune_start_ms = 0;      // 本次調參開始時間(用於總時長保險絲)
static bool tune_ready_seen = false;    // 測速剛就緒的旗標(用於 pre-settle 等待)
static uint32_t tune_ready_since_ms = 0;

// 定速閉環輸出斜率限制：避免每個控制週期都在 0↔滿載間硬切換
static uint16_t last_final_pwm = 0;

// 轉速穩調判定狀態
static uint8_t speed_stable_hits = 0;
static float last_keep_rpm_seen = -1.0f;

// 負載通斷 / 換轉速檔的短暫增益排程(只在 PID_RUN 生效)
static bool pid_transient_active = false;
static uint32_t pid_transient_start_ms = 0;
static bool pid_load_seen = false;
static bool pid_last_load_connected = false;
static float last_keep_for_gain = -1.0f;
static uint8_t pid_assist_hits = 0;
static uint8_t pid_transient_exit_hits = 0;

// ★記憶體保護★：本檔案對 settings/PID_settings 的存取一律透過 settings.h 提供的
// 介面——單一欄位讀寫用 speed_get_x()/set_x()、pid_get_x()/set_x()；需要「一次原子
// 讀寫好幾個相關欄位」的狀態轉換(例如 trip_fault、調參結果落地)則用 SettingsLockGuard
// 包住整段直接存取欄位。務必注意：settings 用 g_speed_mux、PID_settings 用 g_pid_mux，
// 兩個不同的鎖，程式中任何地方都不會巢狀同時持有兩者，避免交叉鎖死的風險。
// 暫態增益只讀 meas_get_load_connected()(g_meas_mux)，讀完即放，不會與 g_pid_mux /
// g_speed_mux 同時持有。

// 強制清除對外穩調旗標(初始化/故障/換目標/未就緒時呼叫)
static void clear_speed_stable()
{
    speed_set_speed_stable(false);
    speed_stable_hits = 0;
}

// 僅在 PID 閉環正常運行時更新 speed_stable
static void update_speed_stable()
{
    const float keep_rpm_now = pid_get_keep_rpm();
    const float now_speed = speed_get_now_speed();

    // 換目標：立刻視為未穩，重新計數
    if (fabsf(keep_rpm_now - last_keep_rpm_seen) > 0.5f)
    {
        last_keep_rpm_seen = keep_rpm_now;
        clear_speed_stable();
    }

    const bool system_ok =
        (!speed_get_fault()) &&
        speed_get_const_speed_ready() &&
        pid_get_autotune_done() &&
        (!pid_get_autotune_active()) &&
        (speed_get_init_phase() == SPEED_PHASE_PID_RUN) &&
        speed_get_speed_valid() &&
        (now_speed >= SPEED_STABLE_MIN_RPM) &&
        (keep_rpm_now > 1.0f);

    if (!system_ok)
    {
        clear_speed_stable();
        return;
    }

    const float err = fabsf(now_speed - keep_rpm_now);
    const float ref = fmaxf(now_speed, keep_rpm_now);
    const float abs_eps = (float)SPEED_STABLE_ABS_EPS +
                          keep_rpm_now * ((float)SPEED_STABLE_ABS_EPS_PER_1000RPM / 1000.0f);
    const bool near_target =
        (err <= abs_eps) ||
        ((ref > 1.0f) && ((err / ref) <= (float)SPEED_STABLE_REL_EPS));

    if (near_target)
    {
        if (speed_stable_hits < 255)
        {
            speed_stable_hits++;
        }
        if (speed_stable_hits >= SPEED_STABLE_NEED_HITS)
        {
            speed_set_speed_stable(true);
        }
    }
    else
    {
        // 還沒穩到新目標 / 偏離目標 → 立刻 false
        clear_speed_stable();
    }
}

static void pid_read_base_gains(float &kp, float &ki, float &kd)
{
    SettingsLockGuard lock(g_pid_mux);
    kp = PID_settings.Kp;
    ki = PID_settings.Ki;
    kd = PID_settings.Kd;
}

static void pid_apply_base_tunings(QuickPID &myPID)
{
    float kp, ki, kd;
    pid_read_base_gains(kp, ki, kd);
    myPID.SetTunings(kp, ki, kd);
}

static void pid_apply_transient_tunings(QuickPID &myPID)
{
    float kp, ki, kd;
    pid_read_base_gains(kp, ki, kd);
    myPID.SetTunings(kp * (float)PID_TRANSIENT_GAIN_SCALE,
                     ki * (float)PID_TRANSIENT_GAIN_SCALE,
                     kd);
}

static void pid_exit_transient(QuickPID &myPID, const char *why)
{
    if (!pid_transient_active)
    {
        return;
    }
    pid_transient_active = false;
    pid_assist_hits = 0;
    pid_transient_exit_hits = 0;
    pid_apply_base_tunings(myPID);
    Serial.printf("[PID] transient gain OFF (%s)\n", why);
}

static void pid_enter_transient(QuickPID &myPID, uint32_t now_ms, const char *why,
                                bool load_on_ff, bool restart_timer)
{
    if (!pid_transient_active)
    {
        // 進暫態當下立刻清穩調：負載剛切時轉速還沒掉，speed_stable 可能仍為 true，
        // 若不先清，同一輪結尾就會誤判「已穩」而立刻退回原增益。
        clear_speed_stable();
        pid_transient_active = true;
        pid_transient_start_ms = now_ms;
        pid_transient_exit_hits = 0;
        pid_apply_transient_tunings(myPID);
        Serial.printf("[PID] transient gain ON (%s) x%.2f slew=%u\n",
                      why, (double)PID_TRANSIENT_GAIN_SCALE,
                      (unsigned)PID_TRANSIENT_SLEW_MAX);
    }
    else if (restart_timer)
    {
        // 負載通斷／換檔是新的真實擾動：重算 8s 上限。誤差輔助不得每輪刷新，否則永遠退不出。
        pid_transient_start_ms = now_ms;
        pid_transient_exit_hits = 0;
    }
    if (load_on_ff)
    {
        const float bumped = fminf(pid_output + (float)PID_LOAD_ON_PWM_BUMP, max_pwm_value);
        pid_output = bumped;
        last_final_pwm = (uint16_t)bumped;
    }
}

static void pid_reset_transient_state(QuickPID *myPID)
{
    if (pid_transient_active && myPID != nullptr)
    {
        pid_exit_transient(*myPID, "reset");
    }
    else
    {
        pid_transient_active = false;
    }
    pid_load_seen = false;
    last_keep_for_gain = -1.0f;
    pid_assist_hits = 0;
    pid_transient_exit_hits = 0;
}

static void pid_update_transient_enter(QuickPID &myPID, uint32_t now_ms)
{
    const float keep_now = pid_get_keep_rpm();
    const bool load_now = meas_get_load_connected();

    if (!pid_load_seen)
    {
        pid_load_seen = true;
        pid_last_load_connected = load_now;
    }
    else if (load_now != pid_last_load_connected)
    {
        pid_last_load_connected = load_now;
        pid_enter_transient(myPID, now_ms, load_now ? "load ON" : "load OFF", load_now, true);
    }

    if (last_keep_for_gain < 0.0f)
    {
        last_keep_for_gain = keep_now;
    }
    else if (fabsf(keep_now - last_keep_for_gain) > 0.5f)
    {
        last_keep_for_gain = keep_now;
        pid_enter_transient(myPID, now_ms, "rpm step", false, true);
    }

    // 誤差輔助：必須轉速可信、且連續超差。單筆光柵離群把 EMA 晃過門檻不得進暫態。
    const bool speed_ok = speed_get_speed_valid();
    const float err = fabsf(speed_get_now_speed() - keep_now);
    if (speed_ok && err > (float)PID_ASSIST_ERR_RPM && keep_now > 1.0f)
    {
        if (pid_assist_hits < 255)
        {
            pid_assist_hits++;
        }
        if (pid_assist_hits >= (uint8_t)PID_ASSIST_NEED_HITS)
        {
            pid_enter_transient(myPID, now_ms, "error assist", false, false);
        }
    }
    else
    {
        pid_assist_hits = 0;
    }
}

static void pid_update_transient_exit(QuickPID &myPID, uint32_t now_ms)
{
    if (!pid_transient_active)
    {
        return;
    }
    const float err = fabsf(speed_get_now_speed() - pid_get_keep_rpm());
    if (speed_get_speed_valid() && err <= (float)PID_TRANSIENT_EXIT_ERR_RPM)
    {
        if (pid_transient_exit_hits < 255)
        {
            pid_transient_exit_hits++;
        }
        if (pid_transient_exit_hits >= (uint8_t)PID_TRANSIENT_EXIT_NEED_HITS)
        {
            pid_exit_transient(myPID, "in band");
        }
        return;
    }
    pid_transient_exit_hits = 0;
    if ((now_ms - pid_transient_start_ms) >= (uint32_t)PID_TRANSIENT_MAX_MS)
    {
        pid_exit_transient(myPID, "timeout");
    }
}

// 觸發失控保護：切斷輸出並鎖定故障碼
static void trip_fault(uint8_t code)
{
    // settings 這組欄位代表「已故障」這單一事實的好幾個面向，整段上鎖一起寫入
    {
        SettingsLockGuard lock(g_speed_mux);
        settings.fault = true;
        settings.fault_code = code;
        settings.const_speed_ready = false;
        settings.ol_hold = false;
        settings.ol_probe_request = false;
        settings.ol_probe_ready = false;
        settings.init_phase = SPEED_PHASE_FAULT;
        settings.ol_pwm_cmd = 0;
    }
    pid_set_autotune_active(false);
    clear_speed_stable();
    pid_reset_transient_state(nullptr);
    step_pwm = 0;
    pid_output = 0.0f;
    last_final_pwm = 0;
    tune_ready_seen = false;
    ledcWrite(MOTOR_PWM_PIN, 0);
}

// 開環階梯起動：
// 升一次 PWM → 等 MOTOR_STEP_SETTLE_MS → 等 sensor 回報第一筆可用轉速
// 若 >= RPM_INIT_READABLE+餘量 → 固定此 PWM(ol_hold) 讓測速模組做就緒觀察
// 否則再升一階；達上限仍不夠 → 故障
static void open_loop_step_control(uint32_t now_ms)
{
    const uint16_t pwm_step =
        (uint16_t)fmaxf(1.0f, max_pwm_value * MOTOR_PWM_STEP_RATIO);
    const uint16_t pwm_max =
        (uint16_t)constrain(max_pwm_value * MOTOR_STARTUP_PWM_MAX_RATIO, 1.0f, max_pwm_value);
    const float need_rpm = RPM_INIT_READABLE + RPM_INIT_MARGIN;

    // 已達可讀門檻：固定當前 PWM，不再升階
    if (speed_get_ol_hold())
    {
        {
            SettingsLockGuard lock(g_speed_mux);
            settings.ol_pwm_cmd = step_pwm;
            settings.ol_probe_request = false;
        }
        pid_output = (float)step_pwm;
        ledcWrite(MOTOR_PWM_PIN, step_pwm);
        return;
    }

    // 尚未套用第一階：先從一個 step 開始
    if (!step_applied)
    {
        step_pwm = pwm_step;
        if (step_pwm > pwm_max)
        {
            step_pwm = pwm_max;
        }
        {
            SettingsLockGuard lock(g_speed_mux);
            settings.ol_pwm_cmd = step_pwm;
            settings.ol_probe_request = false;
            settings.ol_probe_ready = false;
            settings.ol_probe_rpm = 0.0f;
            settings.init_phase = SPEED_PHASE_STEP_UP;
        }
        settle_start_ms = now_ms;
        settling = true;
        step_applied = true;
        pid_output = (float)step_pwm;
        ledcWrite(MOTOR_PWM_PIN, step_pwm);
        return;
    }

    // 輸出目前階梯 duty
    speed_set_ol_pwm_cmd(step_pwm);
    pid_output = (float)step_pwm;
    ledcWrite(MOTOR_PWM_PIN, step_pwm);

    if (settling)
    {
        speed_set_init_phase(SPEED_PHASE_STEP_UP);
        if ((now_ms - settle_start_ms) < (uint32_t)MOTOR_STEP_SETTLE_MS)
        {
            // 仍在等待速度穩定：不請求 probe，避免用到升階瞬間的髒資料
            SettingsLockGuard lock(g_speed_mux);
            settings.ol_probe_request = false;
            settings.ol_probe_ready = false;
            return;
        }

        // settle 結束：請 speed_sensor 擷取「之後第一筆可用」同位置轉速
        speed_set_init_phase(SPEED_PHASE_PROBE);
        speed_set_ol_probe_request(true);

        const bool probe_timeout =
            ((now_ms - settle_start_ms) >=
             ((uint32_t)MOTOR_STEP_SETTLE_MS + (uint32_t)MOTOR_PROBE_TIMEOUT_MS));

        const bool probe_ready_now = speed_get_ol_probe_ready();
        if (!probe_ready_now && !probe_timeout)
        {
            // 尚無可用資料：繼續等
            return;
        }

        // 已拿到 probe，或逾時視為本階轉速不足(0)
        const float probe_rpm = probe_ready_now ? speed_get_ol_probe_rpm() : 0.0f;
        speed_set_ol_probe_request(false);
        settling = false;

        if (probe_rpm >= need_rpm)
        {
            // 轉速已夠：停在此 PWM，交給測速模組做就緒觀察(穩定確認)
            SettingsLockGuard lock(g_speed_mux);
            settings.ol_hold = true;
            settings.rpm_stable = true;
            settings.init_phase = SPEED_PHASE_LEARNING;
            return;
        }

        // 不夠：再升一階
        if (step_pwm >= pwm_max)
        {
            trip_fault(FAULT_PWM_MAX_NO_SPEED);
            return;
        }

        const uint32_t next = (uint32_t)step_pwm + (uint32_t)pwm_step;
        step_pwm = (uint16_t)((next > pwm_max) ? pwm_max : next);
        {
            SettingsLockGuard lock(g_speed_mux);
            settings.ol_pwm_cmd = step_pwm;
            settings.ol_probe_ready = false;
            settings.ol_probe_rpm = 0.0f;
            settings.init_phase = SPEED_PHASE_STEP_UP;
        }
        settle_start_ms = now_ms;
        settling = true;
        pid_output = (float)step_pwm;
        ledcWrite(MOTOR_PWM_PIN, step_pwm);
    }
}

// 是否需要在測速就緒後執行自動調參
static bool need_autotune()
{
#if !PID_AUTOTUNE_ENABLE
    return false;
#else
#if PID_AUTOTUNE_EVERY_BOOT
    return true;
#else
    return !pid_get_tuned();
#endif
#endif
}

// 量測起點必須是「開環 hold 後真正可讀的工作點」，不能拿尚未收斂的 EMA。
// 燒錄若清掉 NVS，舊路徑會把 keep 鎖在爬升中的 ~600RPM，PID 再把馬達往下拉，
// 且 600<1000 永遠無法 speed_stable。probe 已通過 1150 門檻，優先採用。
static void lock_keep_from_working_point()
{
    const float min_rpm = (float)MEASURE_MIN_RPM;
    const float now = speed_get_now_speed();
    const float probe = speed_get_ol_probe_rpm();
    float keep = now;
    if (probe >= min_rpm)
    {
        keep = fmaxf(keep, probe);
    }
    if (keep < min_rpm)
    {
        keep = min_rpm;
    }
    pid_set_keep_rpm(keep);
}

// 配置並啟動 sTune(僅第一次進入調參時呼叫)
static void begin_autotune(sTune &tuner)
{
    // 以目前開環 hold duty 為起點，再往上做階躍以取得 S 曲線
    const uint16_t ol_pwm_now = speed_get_ol_pwm_cmd();
    float out_start = (float)((ol_pwm_now > 0) ? ol_pwm_now : step_pwm);
    if (out_start < 1.0f)
    {
        out_start = max_pwm_value * MOTOR_PWM_STEP_RATIO;
    }

    float out_step = out_start + max_pwm_value * PID_TUNE_STEP_RATIO;
    if (out_step > max_pwm_value)
    {
        out_step = max_pwm_value;
    }
    // 階躍太小則無法辨識曲線
    if (out_step < out_start + 2.0f)
    {
        out_step = fminf(out_start + fmaxf(8.0f, max_pwm_value * 0.05f), max_pwm_value);
    }
    if (out_step <= out_start)
    {
        trip_fault(FAULT_AUTOTUNE_FAIL);
        return;
    }

    // 每次調參都重鎖工作點：NVS 裡的 keep 可能是上次誤鎖的低轉，或舊的 1150 常數。
    lock_keep_from_working_point();
    const float now_speed_now = speed_get_now_speed();

    // 依目前實際轉速動態估計 sTune 輸入全幅，而非直接套用飛車保護的絕對上限，
    // 否則會低估製程增益、算出過度激進的 Kp(工作點離飛車上限愈遠，此差異愈明顯)
    const float input_span = fminf(RPM_RUNAWAY_MAX,
                                    fmaxf(PID_TUNE_INPUT_SPAN_MIN,
                                          fmaxf(now_speed_now, pid_get_keep_rpm()) * 4.0f));
    const float estop_rpm = RPM_RUNAWAY_MAX * PID_TUNE_ESTOP_RATIO;

    pid_input_bridge = now_speed_now;
    pid_output = out_start;
    speed_set_ol_pwm_cmd((uint16_t)out_start);
    ledcWrite(MOTOR_PWM_PIN, (uint32_t)out_start);

    tuner.Configure(input_span,
                    max_pwm_value,
                    out_start,
                    out_step,
                    (uint32_t)PID_TUNE_TEST_SEC,
                    (uint32_t)PID_TUNE_SETTLE_SEC,
                    (uint16_t)PID_TUNE_SAMPLES);
    tuner.SetEmergencyStop(estop_rpm);
#if PID_TUNE_SERIAL_VERBOSE
    // printSUMMARY：結束時印 Ku/Tu/Kp…；過程進度由下方 [TUNE] prog 輸出
    tuner.SetSerialMode(tuner.printSUMMARY);
#else
    tuner.SetSerialMode(tuner.printOFF);
#endif

    pid_set_autotune_active(true);
    pid_set_autotune_done(false);
    speed_set_init_phase(SPEED_PHASE_PID_TUNE);
    clear_speed_stable();
    tuner_configured = true;
    tune_sample_hits = 0;
    tune_last_print_ms = millis();
    tune_start_ms = millis();
    tune_out_start = out_start;
    tune_out_step = out_step;
    tune_input_span = input_span;

    Serial.println(F("[TUNE] =============================="));
    Serial.printf("[TUNE] start out=%.0f -> %.0f (span=%.0f)\n",
                  (double)out_start, (double)out_step, (double)max_pwm_value);
    Serial.printf("[TUNE] settle=%us test<=%us samples=%u inputSpan=%.0f eStop=%.0f\n",
                  (unsigned)PID_TUNE_SETTLE_SEC, (unsigned)PID_TUNE_TEST_SEC,
                  (unsigned)PID_TUNE_SAMPLES, (double)input_span, (double)estop_rpm);
    Serial.printf("[TUNE] rpm_now=%.1f keep_rpm=%.1f rule=NoOvershoot_PI action=direct5T\n",
                  (double)now_speed_now, (double)pid_get_keep_rpm());
    Serial.println(F("[TUNE] =============================="));
}

// 測速就緒後的 sTune 狀態機；回傳 true 表示仍在調參(呼叫端勿進閉環)
static bool run_autotune_step(sTune &tuner, QuickPID &myPID)
{
    if (!tuner_configured)
    {
        begin_autotune(tuner);
        if (speed_get_fault())
        {
            return true;
        }
    }

    // 總時長保險絲：5T 測試理論上會等到訊號真正收斂到平台才結束，
    // 若系統一直沒有真正穩定(持續震盪)，避免無限期停在開環高輸出，逾時強制判失敗斷電
    const uint32_t tune_timeout_ms =
        (uint32_t)(PID_TUNE_SETTLE_SEC + PID_TUNE_TEST_SEC + PID_TUNE_TOTAL_TIMEOUT_MARGIN_SEC) * 1000UL;
    if ((millis() - tune_start_ms) > tune_timeout_ms)
    {
        Serial.printf("[TUNE] FAIL timeout after %lums (system not settling)\n",
                      (unsigned long)(millis() - tune_start_ms));
        trip_fault(FAULT_AUTOTUNE_FAIL);
        return true;
    }

    pid_input_bridge = speed_get_now_speed();
    const uint8_t st = tuner.Run();

    // 無論 sample/test，sTune 都會更新 Output → 立刻寫出 PWM
    const uint32_t duty = (uint32_t)constrain(pid_output, 0.0f, max_pwm_value);
    speed_set_ol_pwm_cmd((uint16_t)duty);
    ledcWrite(MOTOR_PWM_PIN, duty);

    // 用本輪剛取樣、也拿去餵 sTune 的值做飛車檢查，確保兩者判斷基準一致，
    // 同時省一次重複上鎖讀取
    if (pid_input_bridge > RPM_RUNAWAY_MAX)
    {
        trip_fault(FAULT_RUNAWAY_RPM);
        return true;
    }

    switch (st)
    {
    case sTune::sample:
        // 每個測試取樣點：輸入已在上方更新
        speed_set_init_phase(SPEED_PHASE_PID_TUNE);
        tune_sample_hits++;
#if PID_TUNE_SERIAL_VERBOSE
        {
            const uint32_t now = millis();
            if ((now - tune_last_print_ms) >= (uint32_t)PID_TUNE_SERIAL_MS)
            {
                tune_last_print_ms = now;
                // hits 含 settle 期，僅作進度參考
                Serial.printf("[TUNE] prog hits=%lu rpm=%.1f pwm=%u target_step=%.0f->%.0f\n",
                              (unsigned long)tune_sample_hits,
                              (double)pid_input_bridge,
                              (unsigned)duty,
                              (double)tune_out_start,
                              (double)tune_out_step);
            }
        }
#endif
        break;

    case sTune::tunings:
    {
        // 調參完成：取增益 → 套用 QuickPID → 存 NVS
        tuner.GetAutoTunings(&tune_kp, &tune_ki, &tune_kd);
        if (!(isfinite(tune_kp) && isfinite(tune_ki) && isfinite(tune_kd)) ||
            (tune_kp <= 0.0f && tune_ki <= 0.0f && tune_kd <= 0.0f))
        {
            Serial.printf("[TUNE] FAIL invalid gains Kp=%.5f Ki=%.5f Kd=%.5f\n",
                          (double)tune_kp, (double)tune_ki, (double)tune_kd);
            trip_fault(FAULT_AUTOTUNE_FAIL);
            return true;
        }

        // ★單位換算(先前閉環一直劇烈震盪的主因之一)★
        // sTune 內部的製程增益是「無因次」的：
        //     Ku = (ΔPV / inputSpan) / (ΔCO / outputSpan)
        // 因此它由 Ku 推導出來的 Kp 也是無因次的『正規化增益』(每單位正規化誤差 →
        // 每單位正規化輸出)。但 QuickPID 是直接吃真實工程單位(誤差=RPM、輸出=PWM count)，
        // 兩者相差 inputSpan/outputSpan 倍。本機台 inputSpan≈5000 RPM、outputSpan=1023 count，
        // 差距高達約 4.9 倍——直接套用等於把 Kp 放大近 5 倍，已逼近甚至超過本系統的
        // 臨界增益(實測製程增益約 10 RPM/count、Tau≈4.1s、死時間≈0.32s，純比例的臨界增益
        // 僅約 2.1 count/RPM，而未換算的 Kp 是 1.77)，必然震盪。
        // (sTune 官方範例多半 inputSpan 與 outputSpan 數量級相近，比值≈1，所以不會踩到。)
        const float ku_norm = tuner.GetProcessGain(); // 無因次製程增益
        const float tau = tuner.GetTau();
        const float dead = tuner.GetDeadTime();
        const float span_ratio = (max_pwm_value > 0.0f) ? (tune_input_span / max_pwm_value) : 0.0f;
        // 還原成真實單位：每 1 count PWM 對應幾 RPM
        const float process_gain = ku_norm * span_ratio;

        if (!(isfinite(process_gain) && isfinite(tau) && isfinite(dead)) ||
            process_gain <= 1e-6f || tau <= 1e-3f || dead <= 1e-3f)
        {
            Serial.printf("[TUNE] FAIL invalid process gain=%.5f tau=%.5f td=%.5f\n",
                          (double)process_gain, (double)tau, (double)dead);
            trip_fault(FAULT_AUTOTUNE_FAIL);
            return true;
        }

        // NoOvershoot_PI 規則(與 sTune 相同公式，但改用真實單位的製程增益重算)：
        //     Kp = (0.35 / 製程增益) × (Tau / 死時間)
        //     Ti = 1.2 × Tau ， Ki = Kp / Ti  (QuickPID 的 Ki 定義為 Kp/Ti)
        // 注意 sTune 的 GetKi() 回傳的是純粹的重置率 1/(1.2×Tau)，並未乘上 Kp，
        // 與 QuickPID 的 Ki 定義不一致，故此處一併自行推導，避免積分力道被算錯。
        // 另外 Kp×製程增益 = 0.35×(Tau/死時間) 就是閉環迴路增益，只由可控性比值決定；
        // 萬一某次量到異常小的死時間會讓迴路增益暴衝，故對此比值設上限保護。
        float tau_over_dead = tau / dead;
        if (tau_over_dead > (float)PID_TUNE_TAU_DEAD_RATIO_MAX)
        {
            Serial.printf("[TUNE] clamp Tau/td %.1f -> %.1f (迴路增益保護)\n",
                          (double)tau_over_dead, (double)PID_TUNE_TAU_DEAD_RATIO_MAX);
            tau_over_dead = (float)PID_TUNE_TAU_DEAD_RATIO_MAX;
        }

        const float kp_raw = (0.35f / process_gain) * tau_over_dead;
        const float ti_sec = 1.2f * tau;
        const float ki_raw = kp_raw / ti_sec;
        const float kd_raw = 0.0f; // PI 規則不含微分項

        // 再乘上安全係數：ZN 系規則本就偏激進(1/4 衰減比)，加上測試訊號難免有
        // 非線性/雜訊誤差，保守化可降低震盪風險
        const float new_kp = kp_raw * PID_TUNE_GAIN_SCALE;
        const float new_ki = ki_raw * PID_TUNE_GAIN_SCALE;
        const float new_kd = kd_raw * PID_TUNE_GAIN_SCALE;

        // Kp/Ki/Kd/tuned 是「這次調參結果」的同一組數字，整段上鎖一起寫入，
        // 避免其他任務(例如 main 的 [DBG] 列印)讀到「新 Kp 配舊 Ki」這種不存在的組合
        {
            SettingsLockGuard lock(g_pid_mux);
            PID_settings.Kp = new_kp;
            PID_settings.Ki = new_ki;
            PID_settings.Kd = new_kd;
            PID_settings.tuned = true;
        }
        PID_settings.save(); // Flash I/O 已在鎖外進行，見 settings.h 內的實作說明

        myPID.SetTunings(new_kp, new_ki, new_kd);
        myPID.SetMode(myPID.Control::manual); // 先手動，下一輪再切自動接棒
        // 以當前輸出接棒，減少交接突跳；同步設定斜率限制基準避免第一輪就被硬夾
        pid_output = (float)duty;
        last_final_pwm = (uint16_t)duty;

        {
            SettingsLockGuard lock(g_pid_mux);
            PID_settings.autotune_active = false;
            PID_settings.autotune_done = true;
        }
        speed_set_init_phase(SPEED_PHASE_PID_RUN);

        Serial.println(F("[TUNE] ---------- RESULT ----------"));
        Serial.printf("[TUNE] sTune norm Kp=%.6f Ki=%.6f Kd=%.6f (無因次，未換算前)\n",
                      (double)tune_kp, (double)tune_ki, (double)tune_kd);
        Serial.printf("[TUNE] process Ku_norm=%.4f span=%.0fRPM/%.0fcnt -> gain=%.3f RPM/count\n",
                      (double)ku_norm, (double)tune_input_span, (double)max_pwm_value,
                      (double)process_gain);
        Serial.printf("[TUNE] Tu=%.4fs td=%.4fs Tau/td=%.2f Ti=%.3fs loop_gain=%.2f\n",
                      (double)tau, (double)dead, (double)tau_over_dead, (double)ti_sec,
                      (double)(new_kp * process_gain));
        Serial.printf("[TUNE] real-unit Kp=%.6f Ki=%.6f Kd=%.6f (scale=%.2f)\n",
                      (double)kp_raw, (double)ki_raw, (double)kd_raw, (double)PID_TUNE_GAIN_SCALE);
        Serial.printf("[TUNE] done Kp=%.6f Ki=%.6f Kd=%.6f\n",
                      (double)new_kp, (double)new_ki, (double)new_kd);
        Serial.printf("[TUNE] saved to NVS, keep_rpm=%.1f -> enter PID_RUN\n",
                      (double)pid_get_keep_rpm());
        Serial.println(F("[TUNE] ------------------------------"));
        break;
    }

    case sTune::runPid:
    case sTune::timerPid:
        // sTune 內部在 tunings 後會進 timerPid/runPid；我們以 autotune_done 接管
        if (!pid_get_autotune_done() && !pid_get_autotune_active())
        {
            // 異常路徑：未拿到 tunings 卻離開 test
            Serial.printf("[TUNE] FAIL unexpected status=%u\n", (unsigned)st);
            trip_fault(FAULT_AUTOTUNE_FAIL);
        }
        break;

    default:
#if PID_TUNE_SERIAL_VERBOSE
        // settle / test 內部尚未到 sample 時也定期印，方便確認卡在穩定段
        {
            const uint32_t now = millis();
            if ((now - tune_last_print_ms) >= (uint32_t)PID_TUNE_SERIAL_MS)
            {
                tune_last_print_ms = now;
                Serial.printf("[TUNE] wait st=%u rpm=%.1f pwm=%u\n",
                              (unsigned)st,
                              (double)pid_input_bridge,
                              (unsigned)duty);
            }
        }
#endif
        break;
    }

    return !pid_get_autotune_done();
}

void motor_PID_init(void *pvParameters)
{
    // 基礎初始化

    // 載入初始資料(從 NVS 讀出 Kp/Ki/Kd/tuned)
    PID_settings.load();
    // ★不要在這裡把 keep_rpm 強制蓋成任何常數★
    // 下面 begin_autotune()/略過調參分支裡「keep_rpm<=1 時鎖存目前速度」的原始邏輯
    // 必須保留原樣：本機台 PWM→RPM 增益極不均勻，sTune 是對「開環階梯自然停下來的
    // 那個轉速」做特性化，量測序列的起點也必須沿用同一個轉速(由 safe_current_reset()
    // 讀 pid_get_keep_rpm() 取得)，而不是任何事先寫死的常數，否則會在調參完成的瞬間
    // 命令系統跳去一個沒被特性化過的工作點，實測會長時間震盪不收斂。
    pid_set_autotune_active(false);
    pid_set_autotune_done(false);
    tuner_configured = false;

    // 配置PID基本參數(輸入,輸出,目標值)
    static QuickPID myPID(&pid_input_bridge, &pid_output, &pid_setpoint_bridge);
    // sTune：完整 5T 測試(不怕響應有震盪) + NoOvershoot_PI 規則(較不激進)。
    // 關鍵：改用 PI(不含 D)而非 PID —— QuickPID 內部把 Kd 換算成每次取樣的增益時
    // 是「Kd / 取樣秒數」，我們的閉環取樣週期(read_space)僅 15ms，換算下來 D 項增益
    // 會被放大約 60~70 倍(相當於 Kd≈5 時內部乘數高達 300+)，光是量測轉速的正常雜訊
    // /微小波動每輪只要差個幾 RPM，D 項就會直接把輸出打到 0 或打滿，正是先前閉環
    // 一直在低轉速與飛車間反覆震盪、久久無法收斂的根本原因。sTune 對 NoOvershoot_PI
    // 直接回傳 Kd=0，從根源移除這個對取樣率極度敏感的項；此製程本身 Tau/DeadTime 比
    // 值高(sTune 自評「easy to control」)，僅用 PI 即足以穩定收斂。
    // 序列摘要由 sTune 與我們的 [TUNE] 日誌輸出
    static sTune tuner(&pid_input_bridge, &pid_output,
                       sTune::NoOvershoot_PI, sTune::direct5T, sTune::printOFF);

    // 套用已載入的 PID 參數
    myPID.SetTunings(pid_get_Kp(), pid_get_Ki(), pid_get_Kd());
    // 配置PID輸出範圍
    myPID.SetOutputLimits(0, ((1 << pid_get_pwm_res()) - 1));
    // 取樣週期對齊測速任務(毫秒)
    myPID.SetSampleTimeUs((uint32_t)speed_get_read_space() * 1000UL);
    myPID.SetAntiWindupMode(myPID.iAwMode::iAwClamp);
    // 比例項改回「誤差值」計算(pOnError，QuickPID 預設)：
    // pOnMeas 原意是避免『設定值(keep_rpm)變動』瞬間造成 P 項突跳，但本專案 keep_rpm
    // 在定速運行期間是固定不變的常數(只在調參完成當下鎖存一次)，完全用不到這個好處；
    // 代價卻很大——pOnMeas 的 P 項只看『量測值這一輪的變化量』，完全不管『目前離目標
    // 差多遠』，於是當轉速穩定停在遠高於/低於 keep_rpm 的地方時(dInput≈0)，P 項會直接
    // 貢獻 0，只剩很保守的 I 項(NoOvershoot_PI 刻意調得很慢，避免超調)在极慢地拉回，
    // 這就是先前 PID_RUN 階段轉速穩定"卡"在遠高於 keep_rpm 處、上百秒都拉不回來的根因。
    // 改回 pOnError 後，P 項直接正比於『目前誤差』，一有偏離就立刻提供強力的拉回力道，
    // 收斂速度與穩定度都會好上非常多；D 項本來就是 0(NoOvershoot_PI)，dOnMeas/dOnError
    // 對結果沒有影響，維持預設 dOnMeas 即可。
    myPID.SetProportionalMode(myPID.pMode::pOnError);
    myPID.SetDerivativeMode(myPID.dMode::dOnMeas);
    // 初始化輸出PWM之定時器
    ledcAttach(MOTOR_PWM_PIN, pid_get_pwm_freq(), pid_get_pwm_res());

    // 上電先關閉
    ledcWrite(MOTOR_PWM_PIN, 0);
    pid_output = 0.0f;
    last_final_pwm = 0;
    {
        SettingsLockGuard lock(g_speed_mux);
        settings.ol_pwm_cmd = 0;
        settings.ol_hold = false;
        settings.ol_probe_request = false;
        settings.ol_probe_ready = false;
    }

    Serial.printf("[PID] loaded Kp=%.5f Ki=%.5f Kd=%.5f tuned=%d keep=%.1f "
                  "autotune_en=%d every_boot=%d\n",
                  (double)pid_get_Kp(), (double)pid_get_Ki(), (double)pid_get_Kd(),
                  (int)pid_get_tuned(), (double)pid_get_keep_rpm(),
                  (int)PID_AUTOTUNE_ENABLE, (int)PID_AUTOTUNE_EVERY_BOOT);

    while (1)
    {
        // 等待任務通知(卡住while迴圈讓cpu時間被釋放)
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // 失控保護：任何模組置位 fault 後，強制切斷並維持
        if (speed_get_fault())
        {
            myPID.SetMode(myPID.Control::manual);
            pid_set_autotune_active(false);
            tune_ready_seen = false;
            clear_speed_stable();
            pid_reset_transient_state(&myPID);
            pid_output = 0.0f;
            speed_set_ol_pwm_cmd(0);
            last_final_pwm = 0;
            ledcWrite(MOTOR_PWM_PIN, 0);
            continue;
        }

        // ★量測序列請求暫停(偵測到發電機斷線)：非故障，PWM 歸零並停在 PID_PAUSED，
        // 不需重開機。刻意不清 const_speed_ready/autotune_done/tuned：
        //   - const_speed_ready 會由 speed_sensor 既有的停轉偵測(SPEED_STALL_TIMEOUT_MS)
        //     自然置 false，馬達真正停止後也會自然帶回開環重新爬升(ol_hold/settle_samples
        //     都沒被清除，speed_sensor 會直接重新套用上次已知可行的 hold PWM，不必重新
        //     從最低階爬升)；speed_sensor.cpp 的 update_phase_after_sample() 已同步修改，
        //     會保留 PID_PAUSED 階段、暫停中不會把 const_speed_ready 搶回 true。
        //   - autotune_done/tuned 維持 true，恢復時直接跳過自動調參，用已存參數閉環拉回
        //     keep_rpm(暫停期間完全沒被動過，就是本次開機自動調參後自然停下來的那個
        //     轉速，不是任何寫死的常數；理由見上面 motor_PID_init() 開頭的說明)。
        if (meas_get_pause_request())
        {
            myPID.SetMode(myPID.Control::manual);
            clear_speed_stable();
            pid_reset_transient_state(&myPID);
            pid_output = 0.0f;
            last_final_pwm = 0;
            speed_set_ol_pwm_cmd(0);
            speed_set_init_phase(SPEED_PHASE_PID_PAUSED);
            ledcWrite(MOTOR_PWM_PIN, 0);
            continue;
        }
        // 暫停已放行：必須立刻離開 PID_PAUSED。否則 speed_sensor 為了避免暫停中
        // 把 const_speed_ready 搶回 true，會一直把 ready 鎖在 false，開環就卡在
        // 上次 hold PWM(~200、約 1600RPM)，keep 還在五千轉，speed_stable 永遠等不到。
        if (speed_get_init_phase() == SPEED_PHASE_PID_PAUSED)
        {
            speed_set_init_phase(SPEED_PHASE_STEP_UP);
            Serial.println("[PID] pause released -> leave PID_PAUSED, resume climb");
        }

        // 測速尚未完成：階梯開環起動 + 就緒觀察
        if (!speed_get_const_speed_ready())
        {
            myPID.SetMode(myPID.Control::manual);
            tuner_configured = false;
            pid_set_autotune_active(false);
            tune_ready_seen = false;
            clear_speed_stable();
            pid_reset_transient_state(&myPID);
            open_loop_step_control(millis());
            continue;
        }

        // 測速就緒後：若需要則先自動調參
        if (!pid_get_autotune_done())
        {
            clear_speed_stable();
            if (need_autotune())
            {
                myPID.SetMode(myPID.Control::manual);

                // 剛進入就緒狀態時，EMA 濾波值可能還沒完全收斂到穩態，
                // 先等待濾波值真正收斂，避免用尚未穩定的瞬時值鎖定 keep_rpm / 啟動測試
                if (!tune_ready_seen)
                {
                    tune_ready_seen = true;
                    tune_ready_since_ms = millis();
                }
                if ((millis() - tune_ready_since_ms) < (uint32_t)PID_TUNE_PRE_SETTLE_MS)
                {
                    continue; // 等待轉速濾波穩定，維持目前 hold PWM 不動作
                }
                // EMA 若還低於可讀門檻，再多等一會；逾時仍進調參，keep 會改用 probe。
                if (speed_get_now_speed() < (float)MEASURE_MIN_RPM &&
                    (millis() - tune_ready_since_ms) < ((uint32_t)PID_TUNE_PRE_SETTLE_MS + 3000u))
                {
                    continue;
                }

                if (run_autotune_step(tuner, myPID))
                {
                    continue; // 仍在調參
                }
                // 調參完成 → 落下繼續閉環
            }
            else
            {
                // 已有 NVS 參數，略過調參
                pid_set_autotune_done(true);
                pid_set_autotune_active(false);
                lock_keep_from_working_point();
                Serial.printf("[PID] skip autotune (NVS tuned=1), Kp=%.5f Ki=%.5f Kd=%.5f keep=%.1f\n",
                              (double)pid_get_Kp(), (double)pid_get_Ki(),
                              (double)pid_get_Kd(), (double)pid_get_keep_rpm());
            }
        }

        // ----- PID 定速 -----
        speed_set_init_phase(SPEED_PHASE_PID_RUN);

        // 重測時 keep_rpm 會被清成 0，且 autotune_done 已是 true 不會再走 skip 分支。
        if (pid_get_keep_rpm() < (float)MEASURE_MIN_RPM)
        {
            lock_keep_from_working_point();
        }

        const uint32_t now_ms = millis();
        pid_update_transient_enter(myPID, now_ms);

        // 先做快照(要在 SetMode 之前，QuickPID 切自動時會用當下的輸入/輸出做無擾接棒)
        pid_input_bridge = speed_get_now_speed();
        pid_setpoint_bridge = pid_get_keep_rpm();

        // 切入自動模式(以當前 duty 接棒，減少突跳)；同步將斜率限制基準對齊當前輸出，
        // 避免剛切自動的第一輪就被斜率限制夾住造成不必要的延遲
        if (myPID.GetMode() != (uint8_t)myPID.Control::automatic)
        {
            myPID.SetMode(myPID.Control::automatic);
            last_final_pwm = (uint16_t)constrain(pid_output, 0.0f, max_pwm_value);
        }

        // 開始計算
        myPID.Compute();

        // ★全程保留 float 精度，直到真的要送進 ledcWrite() 那一刻才轉整數★
        // pid_output 每輪的變化量可能遠小於 1 個 PWM count(例如穩態時每輪只累積 0.5)，
        // 若提早轉成整數再拿去跟 outputSum 做反算回寫，會把這個「小數部分的積分累積」
        // 直接截斷丟掉；一旦誤差不夠大、不足以讓小數部分單輪就跨過整數邊界，就會變成一個
        // 數值上的不動點：每輪都被拉回同一個起點，積分被自己的回寫鎖死、永遠無法累積，
        // 導致輸出卡死在某個 PWM、轉速穩穩停在遠離目標的地方且完全不再變化(而不是正常的
        // 積分饱和/超調)。因此斜率限制的比較、判斷都要用 float 做，只有最終要寫硬體的
        // ledcWrite() 才轉整數。
        float final_pwm_f = constrain(pid_output, 0.0f, max_pwm_value);

        // 輸出斜率限制：避免每個控制週期都在 0↔滿載間硬切換，
        // 那樣既是機械衝擊，也會透過馬達動態誘發量測轉速的假性尖峰
        const float slew_max = pid_transient_active ? (float)PID_TRANSIENT_SLEW_MAX
                                                    : (float)PID_OUTPUT_SLEW_MAX;
        const float delta_f = final_pwm_f - (float)last_final_pwm;
        bool slew_clamped = false;
        if (delta_f > slew_max)
        {
            final_pwm_f = (float)last_final_pwm + slew_max;
            slew_clamped = true;
        }
        else if (delta_f < -slew_max)
        {
            final_pwm_f = (float)last_final_pwm - slew_max;
            slew_clamped = true;
        }
        final_pwm_f = constrain(final_pwm_f, 0.0f, max_pwm_value);
        const uint32_t final_pwm = (uint32_t)final_pwm_f; // 僅供寫硬體/事件旗標，不回頭污染 PID 狀態
        last_final_pwm = (uint16_t)final_pwm_f;

        // 反算式抗積分飽和(back-calculation)：只有在斜率限制「真的把輸出夾住」時才需要，
        // 目的是避免積分繼續往一個實際上打不出來的方向累積(積分飽和)。若這輪根本沒被夾住，
        // 完全不要動 outputSum，讓 QuickPID 保留它自己完整 float 精度的內部狀態；
        // 這是相對於前一版「每輪都回寫」的關鍵修正——每輪回寫在數學上雖然聲稱是恆等操作，
        // 但因為要轉成整數 PWM 才能回寫，等於每輪都把積分的小數部分截斷歸零，
        // 稳态时误差不够大、单轮跨不过 1 个 PWM count 时就会被这个操作反复拉回同一个起点，
        // 造成积分永远无法累积、输出卡死不动(此系统的真实故障现象)。
        //
        // ★扣掉比例項★ QuickPID 在 pOnError 模式下 outputSum 只存『積分量』，
        // Output = outputSum + pTerm，回寫時務必扣掉當輪的 pTerm，否則等於把比例項
        // 也灌進積分器，稍有誤差就會像棘輪一樣被越推越滿(pOnMeas 模式下 pTerm 恆為 0 才沒事)。
        if (slew_clamped)
        {
            myPID.SetOutputSum(final_pwm_f - myPID.GetPterm());
        }

        // 閉環飛車保護：轉速超限立即切斷(用本輪已取樣、也餵給 PID 的 pid_input_bridge，
        // 確保判斷基準與這輪控制計算完全一致，同時省一次重複上鎖讀取)
        if (pid_input_bridge > RPM_RUNAWAY_MAX)
        {
            trip_fault(FAULT_RUNAWAY_RPM);
            continue;
        }

        speed_set_ol_pwm_cmd((uint16_t)final_pwm); // 供 sensor 做「有輸出卻停轉」判斷
        ledcWrite(MOTOR_PWM_PIN, final_pwm);

        // 更新對外穩調旗標(其他模組依 settings.speed_stable 判斷)
        update_speed_stable();
        pid_update_transient_exit(myPID, now_ms);
    }
}

void motor_PID_start()
{
    xTaskCreatePinnedToCore(
        motor_PID_init,
        "motor_PID",
        6144, // sTune 含緩衝，略增堆疊
        NULL,
        RTOS_PID_LEVEL,
        &xPIDTaskHandle,
        0);
}
