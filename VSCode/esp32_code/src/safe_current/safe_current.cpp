#include "safe_current.h"
#include "measure_seq/measure_seq.h"
#include "pins/pins.h"
#include "debug_tee/debug_tee.h"

#include <cmath>

// 本檔案只在自己的任務(measure_seq 建立)裡被呼叫，不會被其他任務並行呼叫，
// 因此這幾個「本檔案內部才需要知道」的觀察窗變數用純 static 區域變數即可，
// 不必放進共享結構(meas_settings 已經有 safe_droop_ratio 這個「結果」可供對外顯示)。
static uint32_t judge_window_start_ms = 0;
static float judge_window_start_rth = 0.0f;
static float rung_ke = 0.0f;          // 本檔開路 Voc / 開路轉速
static float rung_oc_rpm = 0.0f;
static float rth_cold_ohm = 0.0f;     // 第一檔轉速拉回後的冷態 R_th，整輪不重設
static float rth_cold_acc = 0.0f;
static uint8_t rth_cold_n = 0;
static bool soak_rth_settled = false;
static float rung_result_rpm = 0.0f;  // 判定當下的實際轉速(先快照，再斷負載)
static float stall_ref_rpm = -1.0f;
static uint32_t stall_ref_ms = 0;
static uint32_t contact_open_since_ms = 0;
static uint32_t contact_offline_since_ms = 0;
static bool contact_glitch_logged = false;
static uint32_t soak_mismatch_since_ms = 0;
static uint32_t load_applied_ms = 0;
static bool handoff_released = false; // HANDOFF：滑行結束已放行 PID，接著等 recapture

static uint8_t bj_i_hits = 0;
static uint32_t bj_i_window_ms = 0;
static uint8_t bj_v_hits = 0;
static uint32_t bj_v_window_ms = 0;
static float bj_v_ema = 0.0f;

static void finish_with_result(float i_cont_A, float i_cont_rpm);
static void fail_hard(const char *reason);
static void begin_handoff_to_resistance(float i_cont_A, float last_pass_rpm, const char *why);
static bool finish_because_drive_limit(const char *why);

static void brush_jump_reset()
{
    bj_i_hits = 0;
    bj_i_window_ms = 0;
    bj_v_hits = 0;
    bj_v_window_ms = 0;
    bj_v_ema = 0.0f;
}

// 只抓發電機跳刷：看 INA 帶載電流閃斷／開路電壓閃爍。
// 主動力側 775 與光柵離群不算跳刷（那是速度感測雜訊）。
static bool brush_jump_detected(uint32_t now_ms, bool loaded, uint32_t phase_elapsed_ms)
{
    const float rpm = speed_get_now_speed();
    if (rpm < (float)BRUSH_JUMP_MIN_RPM)
    {
        return false;
    }
    if (loaded && load_applied_ms != 0 &&
        (now_ms - load_applied_ms) < (uint32_t)BRUSH_JUMP_IGNORE_AFTER_CONNECT_MS)
    {
        return false;
    }
    if (loaded && load_applied_ms == 0 &&
        phase_elapsed_ms < (uint32_t)BRUSH_JUMP_IGNORE_AFTER_CONNECT_MS)
    {
        return false;
    }
    if (!(ina_get_online() && ina_get_data_valid()))
    {
        return false;
    }
    const float i_now = fabsf(ina_get_current_A());
    const float v_now = ina_get_bus_V();

    if (loaded)
    {
        // 帶載時電壓仍被測試電阻壓在低位、只有電流掉光：那是 INA 分流雜訊，不是跳刷。
        const float voc = meas_get_safe_oc_voltage_V();
        if (voc > 1.0f && v_now < voc * (float)SAFE_I_OPEN_V_RATIO)
        {
            const float i_from_v = v_now / (float)LOAD_TEST_RESISTOR_OHM;
            if (i_from_v >= (float)SAFE_I_MIN_VALID_A &&
                i_now < i_from_v * (float)INA_I_VS_V_MIN_RATIO)
            {
                return false;
            }
        }
        const float ref = fmaxf(meas_get_safe_electrical_A(), 0.05f);
        if (i_now < ref * (float)BRUSH_JUMP_I_DROP_RATIO)
        {
            if (bj_i_window_ms == 0 || (now_ms - bj_i_window_ms) > (uint32_t)BRUSH_JUMP_I_WINDOW_MS)
            {
                bj_i_window_ms = now_ms;
                bj_i_hits = 0;
            }
            bj_i_hits++;
            if (bj_i_hits >= (uint8_t)BRUSH_JUMP_I_HITS)
            {
                Serial.printf("[SAFE_I] brush jump: load current flicker I=%.3f ref=%.3f hits=%u @ %.0fRPM\n",
                              (double)i_now, (double)ref, (unsigned)bj_i_hits, (double)rpm);
                return true;
            }
        }
    }
    else
    {
        if (bj_v_ema < 0.2f)
        {
            bj_v_ema = v_now;
        }
        else
        {
            bj_v_ema += 0.15f * (v_now - bj_v_ema);
        }
        if (bj_v_ema > 0.5f && v_now < bj_v_ema * (1.0f - (float)BRUSH_JUMP_V_OC_RATIO))
        {
            if (bj_v_window_ms == 0 || (now_ms - bj_v_window_ms) > (uint32_t)BRUSH_JUMP_V_WINDOW_MS)
            {
                bj_v_window_ms = now_ms;
                bj_v_hits = 0;
            }
            bj_v_hits++;
            if (bj_v_hits >= (uint8_t)BRUSH_JUMP_V_HITS)
            {
                Serial.printf("[SAFE_I] brush jump: Voc flicker V=%.2f ema=%.2f hits=%u @ %.0fRPM\n",
                              (double)v_now, (double)bj_v_ema, (unsigned)bj_v_hits, (double)rpm);
                return true;
            }
        }
    }
    return false;
}

// 跳刷：第一檔 → 硬故障(step 結束)。已有通過檔 → 交接進內阻(step 未結束)。
// 回傳 true：呼叫端必須立刻 return *out_done，不可繼續目前子階段。
static bool finish_if_brush_jump(uint32_t now_ms, bool loaded, uint32_t phase_elapsed_ms,
                                 bool *out_done)
{
    if (!brush_jump_detected(now_ms, loaded, phase_elapsed_ms))
    {
        return false;
    }
    load_switch_set(false);
    if (meas_get_safe_pass_any())
    {
        const float rpm = meas_get_safe_last_pass_rpm();
        const float amp = meas_get_safe_last_pass_A();
        meas_set_brush_jump_rpm(rpm);
        begin_handoff_to_resistance(amp, rpm, "brush jump, last pass");
        *out_done = false;
        return true;
    }
    fail_hard("brush jump on first rung — cannot establish any safe continuous current");
    *out_done = true;
    return true;
}

static uint16_t motor_pwm_full_scale()
{
    const uint8_t res = pid_get_pwm_res();
    if (res == 0 || res > 15)
    {
        return 1023;
    }
    return (uint16_t)((1u << res) - 1u);
}

// 帶載後轉速要回到目標附近才採熱穩。容許帶比 speed_stable 更緊，
// 不能寬到把「卡在上一檔」也算成這一檔已經到了。
static bool speed_within_target(float target_rpm, float rpm)
{
    const float err = fabsf(target_rpm - rpm);
    const float abs_eps = (float)SPEED_STABLE_ABS_EPS +
                          target_rpm * ((float)SPEED_STABLE_ABS_EPS_PER_1000RPM / 1000.0f);
    const float rel_eps = (target_rpm > 1.0f) ? (target_rpm * (float)SPEED_STABLE_REL_EPS) : abs_eps;
    float band = fmaxf(abs_eps, rel_eps);
    const float step_cap = (float)SAFE_RPM_STEP * (float)SAFE_SPEED_RECOVER_STEP_FRAC;
    if (band > step_cap)
    {
        band = step_cap;
    }
    return err <= band;
}

// 轉不到目標：PWM 頂滿，或轉速已經不再往上爬。後者不必再空等整段逾時。
static bool drive_cannot_reach_target(uint32_t elapsed_ms, float target_rpm)
{
    if (elapsed_ms < (uint32_t)SAFE_DRIVE_STALL_HOLD_MS)
    {
        stall_ref_rpm = -1.0f;
        return false;
    }
    const float rpm = speed_get_now_speed();
    if (speed_within_target(target_rpm, rpm))
    {
        stall_ref_rpm = -1.0f;
        return false;
    }
    const uint16_t pwm = speed_get_ol_pwm_cmd();
    const uint16_t full = motor_pwm_full_scale();
    if (pwm + (uint16_t)SAFE_DRIVE_STALL_PWM_SLACK >= full)
    {
        return true;
    }
    const uint32_t now = millis();
    if (stall_ref_rpm < 0.0f)
    {
        stall_ref_rpm = rpm;
        stall_ref_ms = now;
        return false;
    }
    if ((now - stall_ref_ms) < (uint32_t)SAFE_DRIVE_STALL_HOLD_MS)
    {
        return false;
    }
    const bool flat = (rpm - stall_ref_rpm) < (float)SAFE_DRIVE_STALL_MIN_GAIN_RPM;
    stall_ref_rpm = rpm;
    stall_ref_ms = now;
    return flat;
}

// R_th = (k_e·n − V) / I。k_e 用本檔開路，n、V、I 用同一筆，轉速被拉低再爬回來不會被看成電流下垂。
static bool loaded_winding_rth(float *rth_out, float *i_out)
{
    if (!(rung_ke > 0.0f) || !(ina_get_online() && ina_get_data_valid() && ina_get_current_plausible()))
    {
        return false;
    }
    const float n = speed_get_now_speed();
    const float i = fabsf(ina_get_current_A());
    if (n < 1.0f || i < (float)SAFE_I_MIN_VALID_A)
    {
        return false;
    }
    const float v = ss54_compensate_voltage_V(ina_get_bus_V(), i);
    const float emf = rung_ke * n;
    if (!(emf > v + 0.02f))
    {
        return false;
    }
    const float rth = (emf - v) / i;
    if (!isfinite(rth) || rth < (float)RES_MIN_OHM || rth > (float)RES_MAX_OHM)
    {
        return false;
    }
    *rth_out = rth;
    if (i_out != nullptr)
    {
        *i_out = i;
    }
    return true;
}

static float winding_rise_limit_c()
{
    const float lim = meas_get_safe_winding_rise_limit_C();
    if (lim >= 10.0f)
    {
        return lim;
    }
    return (float)SAFE_WINDING_RISE_MAX_C;
}

static void publish_winding(float rth)
{
    meas_set_safe_rth_ohm(rth);
    if (rth_cold_ohm > 0.05f)
    {
        const float ratio = (rth / rth_cold_ohm) - 1.0f;
        meas_set_safe_droop_ratio(ratio);
        meas_set_safe_winding_rise_C(ratio / (float)COPPER_TEMPCO_PER_C);
    }
}

static void clear_rung_sample()
{
    rung_ke = 0.0f;
    rung_oc_rpm = 0.0f;
    judge_window_start_ms = 0;
    judge_window_start_rth = 0.0f;
    soak_rth_settled = false;
    rung_result_rpm = 0.0f;
    stall_ref_rpm = -1.0f;
    stall_ref_ms = 0;
}

// PWM 頂滿仍轉不到：不可帶著飽和積分／滿 PWM 直接進內阻(開路會飛車、光柵掉脈衝)。
// 已有通過檔 → 交接(PWM=0 滑行再拉回上一檔)。第一檔就轉不到 → 硬故障。
static bool finish_because_drive_limit(const char *why)
{
    load_switch_set(false);
    if (meas_get_safe_pass_any())
    {
        const float rpm = meas_get_safe_last_pass_rpm();
        const float amp = meas_get_safe_last_pass_A();
        meas_set_drive_limit_rpm(rpm);
        begin_handoff_to_resistance(amp, rpm, why);
        return false;
    }
    fail_hard("cannot reach first rung target — motor cannot establish any safe current speed");
    return true;
}

static void goto_phase(uint8_t ph)
{
    meas_set_safe_phase(ph);
    meas_set_safe_phase_start_ms(millis());
}

void safe_current_reset()
{
    load_switch_set(false);
    // ★第 0 檔的起點必須是「這次開機、自動調參後系統自然停下來的轉速」(pid_get_keep_rpm())，
    // 不可以用 MEASURE_MIN_RPM 這個常數：本機台增益極不均勻，sTune 是對自然停下來的那個
    // 工作點做特性化，硬把系統拉去另一個沒被特性化過的轉速，實測會長時間震盪不收斂。
    const float start_rpm = pid_get_keep_rpm();
    {
        SettingsLockGuard lock(g_meas_mux);
        meas_settings.safe_phase = SAFE_PH_PREP;
        meas_settings.safe_start_rpm = start_rpm;
        meas_settings.safe_target_rpm = start_rpm;
        meas_settings.safe_oc_voltage_V = 0.0f;
        meas_settings.safe_electrical_A = 0.0f;
        meas_settings.safe_hot_A = 0.0f;
        meas_settings.safe_droop_ratio = 0.0f;
        meas_settings.safe_phase_start_ms = millis();
        meas_settings.safe_done = false;
        meas_settings.safe_pass_any = false;
        meas_settings.safe_i_cont_A = 0.0f;
        meas_settings.safe_i_cont_rpm = 0.0f;
        meas_settings.safe_last_pass_rpm = 0.0f;
        meas_settings.safe_last_pass_A = 0.0f;
        meas_settings.safe_last_pass_oc_V = 0.0f;
        meas_settings.brush_jump_rpm = 0.0f;
        meas_settings.drive_limit_rpm = 0.0f;
        meas_settings.thermal_limit_rpm = 0.0f;
        meas_settings.safe_rth_ohm = 0.0f;
        meas_settings.safe_winding_rise_C = 0.0f;
    }
    rth_cold_ohm = 0.0f;
    rth_cold_acc = 0.0f;
    rth_cold_n = 0;
    clear_rung_sample();
    contact_open_since_ms = 0;
    contact_offline_since_ms = 0;
    contact_glitch_logged = false;
    soak_mismatch_since_ms = 0;
    load_applied_ms = 0;
    handoff_released = false;
    brush_jump_reset();
}

void safe_current_rewind_current_rung()
{
    load_switch_set(false);
    if (rth_cold_ohm <= 0.0f)
    {
        rth_cold_acc = 0.0f;
        rth_cold_n = 0;
    }
    clear_rung_sample();
    contact_open_since_ms = 0;
    contact_offline_since_ms = 0;
    contact_glitch_logged = false;
    soak_mismatch_since_ms = 0;
    load_applied_ms = 0;
    handoff_released = false;
    brush_jump_reset();
    meas_set_safe_electrical_A(0.0f);
    meas_set_safe_hot_A(0.0f);
    meas_set_safe_droop_ratio(0.0f);
    meas_set_safe_oc_voltage_V(0.0f);
    meas_set_safe_phase(SAFE_PH_PREP);
    meas_set_safe_phase_start_ms(millis());
    Serial.printf("[SAFE_I] rewind current rung PREP target=%.0f last_pass=%.0f/%.3fA\n",
                  (double)meas_get_safe_target_rpm(),
                  (double)meas_get_safe_last_pass_rpm(),
                  (double)meas_get_safe_last_pass_A());
}

static void finish_with_result(float i_cont_A, float i_cont_rpm)
{
    SettingsLockGuard lock(g_meas_mux);
    meas_settings.safe_i_cont_A = i_cont_A;
    meas_settings.safe_i_cont_rpm = i_cont_rpm;
    meas_settings.safe_done = true;
}

// 完全失敗(連第一檔都不通過，或途中過電流／負載黏死等硬異常)：整機安全鎖定，比照 ESTOP，需重開機。
// 鱷魚夾沒夾好不走這裡，改走 fail_contact() 暫停等 START。
// ★呼叫 speed_trigger_fault() 後立即自行 ledcWrite(0)：不等 motor_PID 任務下一輪才反應。
static void fail_hard(const char *reason)
{
    load_switch_set(false);
    Serial.printf("[SAFE_I] HARD FAIL: %s\n", reason);
    speed_trigger_fault(FAULT_MEASURE_SAFETY);
    ledcWrite(MOTOR_PWM_PIN, 0);
}

// 進內阻前必須先切斷 PWM、倒掉積分：光柵可能讀偏低，只改 keep 仍會頂滿開路飛車。
// Voc 才是真實轉速。滑行到上一檔 Voc 後放行 PID，等 recapture 穩調才回傳 true。
static void begin_handoff_to_resistance(float i_cont_A, float last_pass_rpm, const char *why)
{
    load_switch_set(false);
    finish_with_result(i_cont_A, last_pass_rpm);
    meas_set_safe_target_rpm(last_pass_rpm);
    pid_set_keep_rpm(last_pass_rpm);
    handoff_released = false;
    meas_set_pause_request(true);
    speed_set_ol_pwm_cmd(0); // 看門狗看的是 ol_pwm_cmd，不能只寫 LEDC
    ledcWrite(MOTOR_PWM_PIN, 0);
    goto_phase(SAFE_PH_HANDOFF);
    Serial.printf("[SAFE_I] handoff: PWM cut, coast then recapture %.0fRPM (%s) I_cont=%.3fA\n",
                  (double)last_pass_rpm, why, (double)i_cont_A);
}

// 電流碰到硬上限：已有通過檔 → 斷負載，用上一通過檔進內阻(這就是這台機能測到的上限，不是故障)；
// 第一檔就超過才鎖定。回傳值沿用步進函式慣例：true=已鎖定整機，false=已改走交接流程。
static bool stop_at_current_ceiling(const char *reason)
{
    if (meas_get_safe_pass_any())
    {
        begin_handoff_to_resistance(meas_get_safe_last_pass_A(),
                                    meas_get_safe_last_pass_rpm(), reason);
        return false;
    }
    fail_hard(reason);
    return true;
}

// 線圈溫升碰到上限：已有通過檔 → 用上一檔；第一檔就超過才鎖定。
static bool stop_at_winding_rise(const char *reason)
{
    Serial.printf("[SAFE_I] winding rise %.1fC Rth=%.2f cold=%.2f (%s)\n",
                  (double)meas_get_safe_winding_rise_C(),
                  (double)meas_get_safe_rth_ohm(),
                  (double)rth_cold_ohm,
                  reason);
    if (meas_get_safe_pass_any())
    {
        const float rpm = meas_get_safe_last_pass_rpm();
        meas_set_thermal_limit_rpm(rpm);
        begin_handoff_to_resistance(meas_get_safe_last_pass_A(), rpm, reason);
        return false;
    }
    fail_hard(reason);
    return true;
}

// 鱷魚夾／負載沒接上：暫停等 START，不鎖定、不丢掉已通過檔。
static bool fail_contact(const char *reason)
{
    load_switch_set(false);
    Serial.printf("[SAFE_I] contact pause: %s\n", reason);
    meas_request_contact_pause(reason);
    return false;
}

static bool fail_ina_mismatch(const char *reason)
{
    load_switch_set(false);
    Serial.printf("[SAFE_I] INA V/I mismatch pause: %s\n", reason);
    meas_request_ina_mismatch_pause(reason);
    return false;
}

// 真的負載沒接上：電流接近 0，且電壓回到接近剛才量的 Voc。
// 帶載時端子會被拉到 ~1.5V；INA 電流單筆跳到 0 但電壓仍低，是量測雜訊，不是夾子鬆脫。
static bool load_looks_open_circuit(float i_now, float v_now)
{
    if (i_now >= (float)SAFE_I_MIN_VALID_A)
    {
        return false;
    }
    const float voc = meas_get_safe_oc_voltage_V();
    if (voc > 1.0f)
    {
        return v_now >= voc * (float)SAFE_I_OPEN_V_RATIO;
    }
    return false;
}

static bool confirm_contact_open(uint32_t now_ms, float i_now, float v_now)
{
    if (!load_looks_open_circuit(i_now, v_now))
    {
        contact_open_since_ms = 0;
        if (i_now < (float)SAFE_I_MIN_VALID_A && !contact_glitch_logged)
        {
            contact_glitch_logged = true;
            Serial.printf("[SAFE_I] ignore I glitch I=%.3f V=%.2f Voc=%.2f (load still pulling voltage down)\n",
                          (double)i_now, (double)v_now, (double)meas_get_safe_oc_voltage_V());
        }
        if (i_now >= (float)SAFE_I_MIN_VALID_A)
        {
            contact_glitch_logged = false;
        }
        return false;
    }
    if (contact_open_since_ms == 0)
    {
        contact_open_since_ms = now_ms;
        return false;
    }
    return (now_ms - contact_open_since_ms) >= (uint32_t)SAFE_I_CONTACT_CONFIRM_MS;
}

static bool confirm_ina_offline(uint32_t now_ms, bool online_ok)
{
    if (online_ok)
    {
        contact_offline_since_ms = 0;
        return false;
    }
    if (contact_offline_since_ms == 0)
    {
        contact_offline_since_ms = now_ms;
        return false;
    }
    return (now_ms - contact_offline_since_ms) >= (uint32_t)SAFE_I_CONTACT_CONFIRM_MS;
}

static bool confirm_ina_mismatch(uint32_t now_ms, float i_now, float v_now)
{
    if (!ina_loaded_vi_mismatch(v_now, i_now))
    {
        soak_mismatch_since_ms = 0;
        return false;
    }
    if (soak_mismatch_since_ms == 0)
    {
        soak_mismatch_since_ms = now_ms;
        return false;
    }
    return (now_ms - soak_mismatch_since_ms) >= (uint32_t)INA_I_INCONSISTENT_PAUSE_MS;
}

static bool sample_usable_for_test(float i_now, float v_now)
{
    if (!(ina_get_online() && ina_get_data_valid() && ina_get_current_plausible()))
    {
        return false;
    }
    if (i_now < (float)SAFE_I_MIN_VALID_A)
    {
        return false;
    }
    if (ina_loaded_vi_mismatch(v_now, i_now))
    {
        return false;
    }
    return true;
}

bool safe_current_step(uint32_t now_ms)
{
    const uint8_t ph = meas_get_safe_phase();
    const uint32_t elapsed = now_ms - meas_get_safe_phase_start_ms();

    switch (ph)
    {
    case SAFE_PH_PREP:
    {
        load_switch_set(false);
        const float target = meas_get_safe_target_rpm();
        pid_set_keep_rpm(target);
        goto_phase(SAFE_PH_WAIT_SPEED);
        return false;
    }

    case SAFE_PH_WAIT_SPEED:
        if (speed_get_fault())
        {
            return false; // 交給 measure_seq 外層的 fault 檢查處理，這裡不重複判斷
        }
        {
            bool done = false;
            if (finish_if_brush_jump(now_ms, false, elapsed, &done))
            {
                return done;
            }
        }
        {
            const float target = meas_get_safe_target_rpm();
            if (speed_get_speed_stable() && speed_within_target(target, speed_get_now_speed()))
            {
                goto_phase(SAFE_PH_OC_SAMPLE);
                return false;
            }
            if (drive_cannot_reach_target(elapsed, target))
            {
                return finish_because_drive_limit("speed not reaching target");
            }
            if (elapsed > speed_wait_timeout_ms(target))
            {
                return finish_because_drive_limit("timeout waiting for speed_stable");
            }
        }
        return false;

    case SAFE_PH_OC_SAMPLE:
        // 負載在 PREP 已確保斷開；這裡只需等電氣穩定夠久再讀 Voc，並確認負載真的沒接上
        if (elapsed < (uint32_t)SAFE_OC_SAMPLE_MS)
        {
            return false;
        }
        if (confirm_ina_offline(now_ms, ina_get_online() && ina_get_data_valid()))
        {
            return fail_contact("INA offline during open-circuit sample");
        }
        if (!(ina_get_online() && ina_get_data_valid()))
        {
            return false;
        }
        {
            const float oc_a = fabsf(ina_get_current_A());
            if (oc_a > (float)SAFE_OC_MAX_CURRENT_A)
            {
                fail_hard("open-circuit current not near zero (load stuck closed?)");
                return true;
            }
            // 補償發電機→防反接 SS54→INA232 這條路徑上的順向壓降，換回發電機端子的
            // 真實 Voc(見 settings.h 的 ss54_compensate_voltage_V() 說明)。開路時電流
            // 很小，補償量本來就不大，但一起做才能跟帶載那筆用同一套基準。
            const float voc = ss54_compensate_voltage_V(ina_get_bus_V(), oc_a);
            const float n_oc = speed_get_now_speed();
            meas_set_safe_oc_voltage_V(voc);
            rung_oc_rpm = n_oc;
            rung_ke = (n_oc > 1.0f) ? (voc / n_oc) : 0.0f;
        }
        goto_phase(SAFE_PH_CONNECT);
        return false;

    case SAFE_PH_CONNECT:
        if (!load_switch_is_connected())
        {
            load_switch_set(true);
            load_applied_ms = millis();
            return false; // 下一輪開始累計 LOAD_SWITCH_SETTLE_MS+SAFE_ELECTRICAL_SETTLE_MS
        }
        // 已接通：settle 期間也持續監看硬電流，不必等窗口結束才檢查
        if (ina_get_online() && ina_get_data_valid())
        {
            const float i_now = fabsf(ina_get_current_A());
            if (i_now > (float)SAFE_I_HARD_CEILING_A)
            {
                return stop_at_current_ceiling("hard current ceiling hit while settling, last pass");
            }
        }
        {
            bool done = false;
            if (finish_if_brush_jump(now_ms, true, elapsed, &done))
            {
                return done;
            }
        }
        if (elapsed < (uint32_t)(LOAD_SWITCH_SETTLE_MS + SAFE_ELECTRICAL_SETTLE_MS))
        {
            return false;
        }
        if (confirm_ina_offline(now_ms, ina_get_online() && ina_get_data_valid()))
        {
            return fail_contact("INA offline right after connecting load");
        }
        if (!(ina_get_online() && ina_get_data_valid()))
        {
            return false;
        }
        {
            const float i_elec = fabsf(ina_get_current_A());
            const float v_now = ina_get_bus_V();
            if (confirm_contact_open(now_ms, i_elec, v_now))
            {
                return fail_contact("current too small right after connecting (load not making contact?)");
            }
            if (confirm_ina_mismatch(now_ms, i_elec, v_now))
            {
                return fail_ina_mismatch("V still loaded but I disagrees with V/R after connect — not recorded");
            }
            if (!sample_usable_for_test(i_elec, v_now))
            {
                return false; // 單筆雜訊：再等，不把 0A 當成電氣穩態
            }
            meas_set_safe_electrical_A(i_elec);
            judge_window_start_ms = 0;
            judge_window_start_rth = 0.0f;
            soak_rth_settled = false;
            contact_open_since_ms = 0;
            contact_glitch_logged = false;
        }
        goto_phase(SAFE_PH_THERMAL_SOAK);
        return false;

    case SAFE_PH_THERMAL_SOAK:
    {
        if (speed_get_fault())
        {
            return false;
        }
        if (confirm_ina_offline(now_ms, ina_get_online() && ina_get_data_valid()))
        {
            return fail_contact("INA offline during thermal soak");
        }
        if (!(ina_get_online() && ina_get_data_valid()))
        {
            return false;
        }
        const float i_now = fabsf(ina_get_current_A());
        const float v_now = ina_get_bus_V();
        const uint32_t phase_elapsed = now_ms - meas_get_safe_phase_start_ms();
        if (i_now > (float)SAFE_I_HARD_CEILING_A)
        {
            return stop_at_current_ceiling("hard current ceiling hit during thermal soak, last pass");
        }
        if (confirm_contact_open(now_ms, i_now, v_now))
        {
            return fail_contact("current vanished during thermal soak (load not making contact?)");
        }
        if (confirm_ina_mismatch(now_ms, i_now, v_now))
        {
            return fail_ina_mismatch("V still loaded but I disagrees with V/R during soak — droop not recorded");
        }
        if (!sample_usable_for_test(i_now, v_now))
        {
            judge_window_start_ms = 0;
            if (phase_elapsed >= (uint32_t)SAFE_THERMAL_SOAK_MAX_MS)
            {
                return fail_ina_mismatch("thermal soak ended without a consistent current sample");
            }
            return false;
        }
        {
            bool done = false;
            if (finish_if_brush_jump(now_ms, true, phase_elapsed, &done))
            {
                return done;
            }
        }

        float rth = 0.0f;
        float i_rth = i_now;
        const bool have_rth = loaded_winding_rth(&rth, &i_rth);
        const float target = meas_get_safe_target_rpm();
        const bool speed_back = speed_within_target(target, speed_get_now_speed());

        if (have_rth && speed_back && rth_cold_ohm <= 0.0f)
        {
            rth_cold_acc += rth;
            rth_cold_n++;
            if (rth_cold_n >= (uint8_t)SAFE_RTH_COLD_SAMPLES)
            {
                rth_cold_ohm = rth_cold_acc / (float)rth_cold_n;
                Serial.printf("[SAFE_I] cold Rth=%.2f ohm (%u samples)\n",
                              (double)rth_cold_ohm, (unsigned)rth_cold_n);
            }
        }
        if (have_rth)
        {
            publish_winding(rth);
            if (rth_cold_ohm > 0.05f &&
                meas_get_safe_winding_rise_C() >= winding_rise_limit_c())
            {
                return stop_at_winding_rise("winding temperature rise at limit, last pass");
            }
        }

        // 轉速還沒拉回：熱穩窗先不算。爬不動就收掉，不再空等下一檔。
        if (!speed_back)
        {
            judge_window_start_ms = 0;
            if (drive_cannot_reach_target(phase_elapsed, target))
            {
                return finish_because_drive_limit("loaded speed not recovering to target");
            }
            if (phase_elapsed >= (uint32_t)SAFE_THERMAL_SOAK_MAX_MS)
            {
                soak_rth_settled = false;
                rung_result_rpm = speed_get_now_speed();
                meas_set_safe_hot_A(i_now);
                goto_phase(SAFE_PH_JUDGE);
            }
            return false;
        }

        if (!have_rth || rth_cold_ohm <= 0.0f)
        {
            if (phase_elapsed >= (uint32_t)SAFE_THERMAL_SOAK_MAX_MS)
            {
                return fail_ina_mismatch("thermal soak ended without a winding resistance sample");
            }
            return false;
        }

        if (judge_window_start_ms == 0 || judge_window_start_rth <= 0.0f)
        {
            judge_window_start_ms = now_ms;
            judge_window_start_rth = rth;
            return false;
        }

        if ((now_ms - judge_window_start_ms) >= (uint32_t)SAFE_THERMAL_CHECK_WINDOW_MS)
        {
            const float ratio = (rth - judge_window_start_rth) / judge_window_start_rth;
            if (fabsf(ratio) <= (float)SAFE_RTH_STABLE_RATIO)
            {
                meas_set_safe_hot_A(i_rth);
                soak_rth_settled = true;
                rung_result_rpm = speed_get_now_speed();
                Serial.printf("[SAFE_I] Rth settled %.2f ohm rise %.1fC @ %.0fRPM I=%.3f\n",
                              (double)rth, (double)meas_get_safe_winding_rise_C(),
                              (double)rung_result_rpm, (double)i_rth);
                goto_phase(SAFE_PH_JUDGE);
                return false;
            }
            judge_window_start_ms = now_ms;
            judge_window_start_rth = rth;
        }

        if (phase_elapsed >= (uint32_t)SAFE_THERMAL_SOAK_MAX_MS)
        {
            meas_set_safe_hot_A(i_rth);
            soak_rth_settled = false;
            rung_result_rpm = speed_get_now_speed();
            goto_phase(SAFE_PH_JUDGE);
            return false;
        }
        return false;
    }

    case SAFE_PH_JUDGE:
    {
        load_switch_set(false); // 判定前先斷開，不再繼續加熱
        const float rpm_cmd = meas_get_safe_target_rpm();
        const float rpm_pass = (rung_result_rpm > 1.0f) ? rung_result_rpm : rpm_cmd;
        const float hot_a = meas_get_safe_hot_A();
        const float rise = meas_get_safe_winding_rise_C();

        if (soak_rth_settled && rise < winding_rise_limit_c())
        {
            const float oc_v = meas_get_safe_oc_voltage_V();
            {
                SettingsLockGuard lock(g_meas_mux);
                meas_settings.safe_pass_any = true;
                meas_settings.safe_last_pass_rpm = rpm_pass;
                meas_settings.safe_last_pass_A = hot_a;
                meas_settings.safe_last_pass_oc_V = oc_v;
            }
            const float next_rpm = rpm_cmd + (float)SAFE_RPM_STEP;
            const float predicted_next_a = (rpm_pass > 1.0f) ? (hot_a * (next_rpm / rpm_pass)) : hot_a;
            if (predicted_next_a >= (float)SAFE_I_HARD_CEILING_A)
            {
                begin_handoff_to_resistance(hot_a, rpm_pass, "predicted next current at ceiling");
                return false;
            }
            clear_rung_sample();
            meas_set_safe_target_rpm(next_rpm);
            goto_phase(SAFE_PH_COOLDOWN);
            return false;
        }

        if (meas_get_safe_pass_any())
        {
            const float rpm = meas_get_safe_last_pass_rpm();
            meas_set_thermal_limit_rpm(rpm);
            begin_handoff_to_resistance(meas_get_safe_last_pass_A(), rpm,
                                       "winding resistance still rising, last pass");
            return false;
        }
        fail_hard("first rung winding resistance did not settle — cannot establish any safe continuous current");
        return true;
    }

    case SAFE_PH_COOLDOWN:
        if (elapsed >= (uint32_t)SAFE_COOLDOWN_MS)
        {
            goto_phase(SAFE_PH_PREP);
        }
        return false;

    case SAFE_PH_HANDOFF:
    {
        if (speed_get_fault())
        {
            return false;
        }
        const float last_pass = meas_get_safe_i_cont_rpm();
        if (!handoff_released)
        {
            meas_set_pause_request(true);
            const float voc_goal = meas_get_safe_last_pass_oc_V();
            const bool ina_ok = ina_get_online() && ina_get_data_valid();
            float v_now = 0.0f;
            if (ina_ok)
            {
                v_now = ss54_compensate_voltage_V(ina_get_bus_V(), fabsf(ina_get_current_A()));
            }
            const bool voc_ok = (voc_goal < 0.5f) ||
                                (ina_ok && (v_now <= voc_goal * (float)SAFE_HANDOFF_VOC_RATIO));
            const bool min_done = elapsed >= (uint32_t)SAFE_HANDOFF_COAST_MIN_MS;
            const bool max_done = elapsed >= (uint32_t)SAFE_HANDOFF_COAST_MAX_MS;
            if (max_done && !voc_ok)
            {
                fail_hard("handoff coast: Voc still high with PWM requested off");
                return true;
            }
            if (min_done && voc_ok)
            {
                pid_set_keep_rpm(last_pass);
                meas_set_pause_request(false);
                handoff_released = true;
                meas_set_safe_phase_start_ms(now_ms);
                Serial.printf("[SAFE_I] handoff: coast done Voc=%.2f goal=%.2f -> recapture %.0fRPM\n",
                              (double)v_now, (double)voc_goal, (double)last_pass);
            }
            return false;
        }
        if (speed_get_speed_stable())
        {
            Serial.printf("[SAFE_I] handoff recapture stable @ %.0fRPM -> resistance\n",
                          (double)last_pass);
            return true;
        }
        if (elapsed > speed_wait_timeout_ms(last_pass))
        {
            fail_hard("handoff recapture timeout at last-pass speed");
            return true;
        }
        return false;
    }

    default:
        goto_phase(SAFE_PH_PREP);
        return false;
    }
}
