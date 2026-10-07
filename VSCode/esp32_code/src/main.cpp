#include <Arduino.h>
#include "HardwareSerial.h"
#include "driver/gpio.h"
#include "speed_sensor/speed_sensor.h"
#include "motor_PID/motor_PID.h"
#include "ina232/ina232.h"
#include "board_ui/board_ui.h"
#include "measure_seq/measure_seq.h"
#include "bt_telemetry/bt_telemetry.h"
#include "debug_tee/debug_tee.h"

// ★記憶體保護★：main.cpp 是純粹的「協調 + 顯示」層，對所有共享狀態的存取
// 一律透過 settings.h 提供的介面(speed_get/set_x、pid_get/set_x、ina_get_x、
// ui_get/set_x)；一次要更新好幾個彼此相關欄位(例如 trigger_estop)時，
// 用 SettingsLockGuard 包住整段直接寫欄位，確保其他任務不會讀到只更新一半的狀態。

static constexpr uint32_t SERIAL_DEBUG_MS = 250;
static constexpr uint32_t SERIAL_RPM_MS = 1000;

uint32_t Serial_debug_time;
uint32_t Serial_rpm_time;

static uint8_t last_phase = 0xFF;
static uint8_t last_fault_code = 0xFF;
static bool last_ol_hold = false;
static bool last_ready = false;
static bool last_probe_ready = false;
static bool last_tune_active = false;
static bool last_tune_done = false;
static bool last_speed_stable = false;
static uint8_t last_ui_state = 0xFF;
static uint8_t last_meas_phase = 0xFF;
static bool last_link_lost = false;

// ---- START 鈕(SW2 → IO15，對 GND；板端無外接上拉) ----
static volatile bool start_irq_pending = false;
static uint32_t start_last_accept_ms = 0;
static bool start_raw_last = false;
static bool start_stable = false;
static bool start_prev = false;
static uint32_t start_last_change_ms = 0;

static void IRAM_ATTR start_isr()
{
  start_irq_pending = true;
}

static bool start_raw_pressed()
{
#if START_ACTIVE_LOW
  return digitalRead(START_PIN) == LOW;
#else
  return digitalRead(START_PIN) == HIGH;
#endif
}

static void start_pin_init()
{
  pinMode(START_PIN, INPUT_PULLUP);
  gpio_pullup_en((gpio_num_t)START_PIN);
  gpio_pulldown_dis((gpio_num_t)START_PIN);
  delay(5);

  start_raw_last = start_raw_pressed();
  start_stable = start_raw_last;
  start_prev = start_stable;
  start_last_change_ms = millis();

  attachInterrupt(digitalPinToInterrupt(START_PIN), start_isr, CHANGE);

  Serial.printf("[MAIN] START pin=IO%u pullup ON level=%d pressed=%d "
                "(idle expect level=1; press→0)\n",
                (unsigned)START_PIN, digitalRead(START_PIN), (int)start_raw_pressed());
}

static void trigger_estop()
{
  // 這組欄位代表「已急停」的完整事實，整段上鎖一起寫入
  {
    SettingsLockGuard lock(g_speed_mux);
    settings.fault = true;
    settings.fault_code = FAULT_ESTOP;
    settings.const_speed_ready = false;
    settings.ol_hold = false;
    settings.ol_probe_request = false;
    settings.ol_probe_ready = false;
    settings.init_phase = SPEED_PHASE_FAULT;
    settings.speed_valid = false;
    settings.speed_stable = false;
    settings.ol_pwm_cmd = 0;
  }
  pid_set_autotune_active(false);
  ledcWrite(MOTOR_PWM_PIN, 0);

  ui_set_state(UI_ESTOP);
  Serial.println("[MAIN] ESTOP locked — reboot required");
}

static void start_motor_control()
{
  if (ui_get_motor_started())
  {
    return;
  }
  motor_PID_start();
  speed_sensor_start();
  measure_seq_start(); // 等自動調參後定速穩定，依序驅動安全電流→內阻→曲線計算
  // 這 2 個欄位代表「已啟動」的同一件事，整段上鎖一起寫入
  {
    SettingsLockGuard lock(g_ui_mux);
    ui_settings.motor_started = true;
    ui_settings.state = UI_RUNNING;
  }
  Serial.println("[MAIN] START → motor_PID + speed_sensor + measure_seq started");
}

// 發電機斷線暫停中再按 START：請 measure_seq 從中斷的那一檔／那一點續測（不從頭重跑目前模組）。
// 只負責「轉達使用者意圖」與樂觀更新 UI，實際的恢復時序(等 speed_stable 等)由 measure_seq 自行把關。
static void resume_after_link_loss()
{
  meas_set_resume_requested(true);
  // 使用者已重按 START 確認要續測：先清斷線旗標，避免 loop() 立刻把 UI 打回 LINK_LOST
  // (否則 OLED/上位機會一直顯示「斷線」，即使馬達已開始重新爬升)
  meas_set_link_lost(false);
  ui_set_state(UI_RUNNING);
  Serial.println("[MAIN] START → resume requested after clip/lead pause");
}

// 去彈跳後偵測按下邊緣；ISR 旗標加速喚醒，仍以穩定電平為準
static void handle_start_button(uint32_t now_ms)
{
  if (start_irq_pending)
  {
    start_irq_pending = false;
  }

  const bool raw = start_raw_pressed();
  if (raw != start_raw_last)
  {
    Serial.printf("[MAIN] START raw %d -> %d @%lums (gpio=%d)\n",
                  (int)start_raw_last, (int)raw, (unsigned long)now_ms,
                  digitalRead(START_PIN));
    start_raw_last = raw;
    start_last_change_ms = now_ms;
  }
  else if ((now_ms - start_last_change_ms) >= (uint32_t)START_DEBOUNCE_MS)
  {
    start_stable = start_raw_last;
  }

  const bool edge = (start_stable && !start_prev);
  start_prev = start_stable;
  if (!edge)
  {
    return;
  }

  // 額外保護：兩次有效動作至少間隔 debounce*2
  if ((now_ms - start_last_accept_ms) < (uint32_t)(START_DEBOUNCE_MS * 2))
  {
    return;
  }
  start_last_accept_ms = now_ms;

  const uint8_t cur_state = ui_get_state();
  Serial.printf("[MAIN] START edge accepted, state=%u\n", (unsigned)cur_state);

  if (cur_state == UI_WAIT_START)
  {
    if (!ui_get_motor_started())
    {
      start_motor_control();
    }
    else
    {
      meas_set_restart_requested(true);
      ui_set_state(UI_RUNNING);
      Serial.println("[MAIN] START → new measurement session (no reboot)");
    }
  }
  else if (cur_state == UI_RUNNING)
  {
    // 續測爬升中 UI 已樂觀切成 RUNNING，phase 仍是 LINK_LOST：再按 START 不當急停。
    if (meas_get_phase() == MEAS_LINK_LOST)
    {
      Serial.println("[MAIN] START ignored during resume climb (not ESTOP)");
      return;
    }
    trigger_estop();
  }
  else if (cur_state == UI_LINK_LOST)
  {
    resume_after_link_loss();
  }
}

static const char *phase_name(uint8_t phase)
{
  switch (phase)
  {
  case SPEED_PHASE_STEP_UP:
    return "STEP_UP";
  case SPEED_PHASE_PROBE:
    return "PROBE";
  case SPEED_PHASE_LEARNING:
    return "LEARNING";
  case SPEED_PHASE_READY:
    return "READY";
  case SPEED_PHASE_FAULT:
    return "FAULT";
  case SPEED_PHASE_PID_TUNE:
    return "PID_TUNE";
  case SPEED_PHASE_PID_RUN:
    return "PID_RUN";
  case SPEED_PHASE_PID_PAUSED:
    return "PID_PAUSED";
  default:
    return "UNKNOWN";
  }
}

static const char *fault_name(uint8_t code)
{
  switch (code)
  {
  case FAULT_NONE:
    return "NONE";
  case FAULT_RUNAWAY_RPM:
    return "RUNAWAY_RPM";
  case FAULT_NO_PULSE_TIMEOUT:
    return "NO_PULSE_TIMEOUT";
  case FAULT_PWM_MAX_NO_SPEED:
    return "PWM_MAX_NO_SPEED";
  case FAULT_AUTOTUNE_FAIL:
    return "AUTOTUNE_FAIL";
  case FAULT_ESTOP:
    return "ESTOP";
  case FAULT_MEASURE_SAFETY:
    return "MEASURE_SAFETY";
  default:
    return "UNKNOWN";
  }
}

static const char *ui_state_name(uint8_t st)
{
  switch (st)
  {
  case UI_WAIT_START:
    return "WAIT_START";
  case UI_RUNNING:
    return "RUNNING";
  case UI_ESTOP:
    return "ESTOP";
  case UI_LINK_LOST:
    return "LINK_LOST";
  default:
    return "UNKNOWN";
  }
}

static const char *meas_phase_name(uint8_t ph)
{
  switch (ph)
  {
  case MEAS_IDLE:
    return "IDLE";
  case MEAS_MIN_SPEED_HOLD:
    return "MIN_SPEED_HOLD";
  case MEAS_SAFE_CURRENT:
    return "SAFE_CURRENT";
  case MEAS_RESISTANCE:
    return "RESISTANCE";
  case MEAS_CURVE_CALC:
    return "CURVE_CALC";
  case MEAS_DONE:
    return "DONE";
  case MEAS_LINK_LOST:
    return "LINK_LOST";
  default:
    return "UNKNOWN";
  }
}

static void serial_print_events()
{
  const uint8_t phase = speed_get_init_phase();
  const uint8_t fcode = speed_get_fault_code();
  const bool hold = speed_get_ol_hold();
  const bool ready = speed_get_const_speed_ready();
  const bool probe_rdy = speed_get_ol_probe_ready();
  const uint8_t ui_state_now = ui_get_state();

  if (ui_state_now != last_ui_state)
  {
    Serial.printf("[EVT] ui %s -> %s\n",
                  ui_state_name(last_ui_state), ui_state_name(ui_state_now));
    last_ui_state = ui_state_now;
  }
  if (phase != last_phase)
  {
    Serial.printf("[EVT] phase %s -> %s\n",
                  phase_name(last_phase), phase_name(phase));
    last_phase = phase;
  }
  if (hold != last_ol_hold)
  {
    Serial.printf("[EVT] ol_hold %d -> %d (pwm=%u probe_rpm=%.1f)\n",
                  (int)last_ol_hold, (int)hold,
                  (unsigned)speed_get_ol_pwm_cmd(),
                  (double)speed_get_ol_probe_rpm());
    last_ol_hold = hold;
  }
  if (probe_rdy != last_probe_ready)
  {
    if (probe_rdy)
    {
      Serial.printf("[EVT] probe latched rpm=%.1f (need>=%.1f)\n",
                    (double)speed_get_ol_probe_rpm(),
                    (double)(RPM_INIT_READABLE + RPM_INIT_MARGIN));
    }
    last_probe_ready = probe_rdy;
  }
  if (ready != last_ready)
  {
    Serial.printf("[EVT] const_speed_ready %d -> %d\n",
                  (int)last_ready, (int)ready);
    last_ready = ready;
  }
  const bool tune_active_now = pid_get_autotune_active();
  if (tune_active_now != last_tune_active)
  {
    Serial.printf("[EVT] autotune_active %d -> %d\n",
                  (int)last_tune_active, (int)tune_active_now);
    last_tune_active = tune_active_now;
  }
  const bool tune_done_now = pid_get_autotune_done();
  if (tune_done_now != last_tune_done)
  {
    Serial.printf("[EVT] autotune_done %d -> %d (Kp=%.5f Ki=%.5f Kd=%.5f)\n",
                  (int)last_tune_done, (int)tune_done_now,
                  (double)pid_get_Kp(), (double)pid_get_Ki(),
                  (double)pid_get_Kd());
    last_tune_done = tune_done_now;
  }
  const bool speed_stable_now = speed_get_speed_stable();
  if (speed_stable_now != last_speed_stable)
  {
    Serial.printf("[EVT] speed_stable %d -> %d (rpm=%.1f keep=%.1f)\n",
                  (int)last_speed_stable, (int)speed_stable_now,
                  (double)speed_get_now_speed(), (double)pid_get_keep_rpm());
    last_speed_stable = speed_stable_now;
  }
  if (fcode != last_fault_code)
  {
    if (speed_get_fault())
    {
      Serial.printf("[EVT] FAULT code=%u (%s) rpm=%.1f pwm=%u\n",
                    (unsigned)fcode, fault_name(fcode),
                    (double)speed_get_now_speed(),
                    (unsigned)speed_get_ol_pwm_cmd());
    }
    else if (last_fault_code != 0xFF)
    {
      Serial.printf("[EVT] fault cleared\n");
    }
    last_fault_code = fcode;
  }

  const uint8_t meas_phase_now = meas_get_phase();
  if (meas_phase_now != last_meas_phase)
  {
    Serial.printf("[EVT] measure_seq %s -> %s\n",
                  meas_phase_name(last_meas_phase), meas_phase_name(meas_phase_now));
    last_meas_phase = meas_phase_now;
  }
  const bool link_lost_now = meas_get_link_lost();
  if (link_lost_now != last_link_lost)
  {
    Serial.printf("[EVT] gen link_lost %d -> %d\n", (int)last_link_lost, (int)link_lost_now);
    last_link_lost = link_lost_now;
  }
}

static void serial_print_debug_status()
{
  Serial.printf(
      "[DBG] t=%lu ui=%s start=%d/%d phase=%s rpm=%.1f valid=%d pwm=%u hold=%d "
      "probe_req=%d probe_rdy=%d probe_rpm=%.1f "
      "settle=%u/%u ready=%d ol_hold_ok=%d speed_stable=%d fault=%d(%s) edges=%lu "
      "keep=%.1f tune=%d/%d tunedNVS=%d Kp=%.4f Ki=%.4f Kd=%.4f "
      "V=%.2f I=%.3f P=%.2f ina=%d/%d p=%d n=%lu\n",
      (unsigned long)millis(),
      ui_state_name(ui_get_state()),
      digitalRead(START_PIN),
      (int)start_raw_pressed(),
      phase_name(speed_get_init_phase()),
      (double)speed_get_now_speed(),
      (int)speed_get_speed_valid(),
      (unsigned)speed_get_ol_pwm_cmd(),
      (int)speed_get_ol_hold(),
      (int)speed_get_ol_probe_request(),
      (int)speed_get_ol_probe_ready(),
      (double)speed_get_ol_probe_rpm(),
      (unsigned)speed_get_settle_samples(),
      (unsigned)READY_SETTLE_MIN_SAMPLES,
      (int)speed_get_const_speed_ready(),
      (int)speed_get_rpm_stable(),
      (int)speed_get_speed_stable(),
      (int)speed_get_fault(),
      fault_name(speed_get_fault_code()),
      (unsigned long)speed_get_edge_count(),
      (double)pid_get_keep_rpm(),
      (int)pid_get_autotune_active(),
      (int)pid_get_autotune_done(),
      (int)pid_get_tuned(),
      (double)pid_get_Kp(),
      (double)pid_get_Ki(),
      (double)pid_get_Kd(),
      (double)ina_get_bus_V(),
      (double)ina_get_current_A(),
      (double)ina_get_power_W(),
      (int)ina_get_online(),
      (int)ina_get_data_valid(),
      (int)ina_get_current_plausible(),
      (unsigned long)ina_get_sample_count());
}

void setup()
{
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("[BOOT] ESP32 micro-generator firmware");
  Serial.printf("[BOOT] init_need_rpm>=%.1f settle=%ums probe_timeout=%ums "
                "pwm_step=%.0f%% max_ol=%.0f%% runaway=%.0f\n",
                (double)(RPM_INIT_READABLE + RPM_INIT_MARGIN),
                (unsigned)MOTOR_STEP_SETTLE_MS,
                (unsigned)MOTOR_PROBE_TIMEOUT_MS,
                (double)(MOTOR_PWM_STEP_RATIO * 100.0f),
                (double)(MOTOR_STARTUP_PWM_MAX_RATIO * 100.0f),
                (double)RPM_RUNAWAY_MAX);
  Serial.printf("[BOOT] PID autotune en=%d every_boot=%d settle=%us test=%us "
                "samples=%u step=%.0f%% rule=NoOvershoot_PI action=direct5T pmode=pOnError "
                "gain_units=real verbose=%d\n",
                (int)PID_AUTOTUNE_ENABLE, (int)PID_AUTOTUNE_EVERY_BOOT,
                (unsigned)PID_TUNE_SETTLE_SEC, (unsigned)PID_TUNE_TEST_SEC,
                (unsigned)PID_TUNE_SAMPLES,
                (double)(PID_TUNE_STEP_RATIO * 100.0f),
                (int)PID_TUNE_SERIAL_VERBOSE);
  Serial.printf("[BOOT] tune_estop=%.0f pre_settle=%ums gain_scale=%.2f "
                "slew_max=%u/cycle trans_gain=%.2f trans_slew=%u trans_max=%ums "
                "speed_filter_alpha=%.2f load_R=%.1fΩ\n",
                (double)(RPM_RUNAWAY_MAX * PID_TUNE_ESTOP_RATIO),
                (unsigned)PID_TUNE_PRE_SETTLE_MS,
                (double)PID_TUNE_GAIN_SCALE,
                (unsigned)PID_OUTPUT_SLEW_MAX,
                (double)PID_TRANSIENT_GAIN_SCALE,
                (unsigned)PID_TRANSIENT_SLEW_MAX,
                (unsigned)PID_TRANSIENT_MAX_MS,
                (double)SPEED_FILTER_ALPHA,
                (double)LOAD_TEST_RESISTOR_OHM);
  Serial.printf("[BOOT] I2C OLED=soft INA=soft@%uHz | INA addr=0x%02X Rshunt=%.3fΩ Imax=%.2fA\n",
                (unsigned)I2C_INA_FREQ_HZ,
                (unsigned)INA232_I2C_ADDR,
                (double)INA232_RSHUNT_OHM, (double)INA232_IMAX_A);
  Serial.println("[BOOT] START handled in main; UI is display-only");

  Serial_debug_time = millis();
  Serial_rpm_time = millis();

  start_pin_init();
  board_ui_oled_init();
  board_ui_start();
  ina232_start();
  bt_telemetry_start();
  Serial.println("[BOOT] tasks started (board_ui display + ina232 + bt_telemetry)");
}

void loop()
{
  const uint32_t now = millis();

  // START：起動 / 急停
  handle_start_button(now);

  // 其他模組 fault → UI 跟著進急停畫面
  if ((ui_get_state() == UI_RUNNING) && speed_get_fault())
  {
    ui_set_state(UI_ESTOP);
  }

  // 量測序列偵測到夾子／量測線鬆脫且馬達已暫停 → UI 進入(非故障的)暫停畫面。
  // 必須同時看 pause_request：續測恢復期間 pause_request 已放行、link_lost 已清除，
  // 不應再把 UI 打回 LINK_LOST(否則 OLED 無反應、上位機異常彈窗不會消失)。
  if (meas_get_link_lost() && meas_get_pause_request())
  {
    if (ui_get_state() == UI_RUNNING)
    {
      ui_set_state(UI_LINK_LOST);
    }
  }

  serial_print_events();

  if (now - Serial_debug_time >= SERIAL_DEBUG_MS)
  {
    serial_print_debug_status();
    Serial_debug_time = now;
  }

  if (now - Serial_rpm_time >= SERIAL_RPM_MS)
  {
    Serial.println(speed_get_now_speed());
    Serial_rpm_time = now;
  }

  // ★關鍵：loop() 之前完全沒有任何讓出 CPU 的呼叫(無 delay/yield)，
  // 會把 Core1 的 IDLE 任務餓死 → 觸發 Task Watchdog，造成週期性長時間卡頓
  // (訊息斷斷續續、間隔越來越長，且 START 按鍵在卡頓期間完全偵測不到)。
  // 1ms 對按鈕手動操作毫無影響，但足以餵飽 WDT、讓排程恢復正常。
  vTaskDelay(pdMS_TO_TICKS(1));
}
