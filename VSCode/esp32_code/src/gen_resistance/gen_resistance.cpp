#include "gen_resistance.h"
#include "measure_seq/measure_seq.h"
#include "pins/pins.h"
#include "debug_tee/debug_tee.h"

static float res_oc_rpm = 0.0f; // 開路取樣當下的轉速，帶載時用來把 Voc 對到同一轉速

// 高/中/低三個轉速點：低＝安全電流階梯實際的起始轉速(自動調參後系統自然停下來的那個
// 轉速，不是任何寫死的常數)；高＝安全電流實際通過的最高檔(I_cont 對應的轉速)；
// 中＝兩者平均。都是安全電流已經驗證過、可以安全帶同一顆測試電阻的轉速。
// ★量測順序刻意設為 高→中→低：安全電流結束時馬達多半還在最高檔轉速，若第一點
// 就命令降到 low，會出現數千 RPM 的階躍、20s 內收斂不了而觸發 MEASURE_SAFETY。
static float target_rpm_for_point(uint8_t idx)
{
    const float low = meas_get_safe_start_rpm();
    const float high = fmaxf(meas_get_safe_i_cont_rpm(), low + 1.0f); // +1 避免與 low 完全重疊
    const float mid = 0.5f * (low + high);
    switch (idx)
    {
    case 0:
        return high;
    case 1:
        return mid;
    default:
        return low;
    }
}

static void goto_phase(uint8_t ph)
{
    meas_set_res_phase(ph);
    meas_set_res_phase_start_ms(millis());
}

void gen_resistance_reset()
{
    res_oc_rpm = 0.0f;
    load_switch_set(false);
    SettingsLockGuard lock(g_meas_mux);
    meas_settings.res_phase = RES_PH_PREP;
    meas_settings.res_point_index = 0;
    meas_settings.res_target_rpm = 0.0f;
    meas_settings.res_oc_voltage_V = 0.0f;
    meas_settings.res_load_V = 0.0f;
    meas_settings.res_load_A = 0.0f;
    meas_settings.res_phase_start_ms = millis();
    meas_settings.res_done = false;
    meas_settings.res_valid_points = 0;
    meas_settings.res_rth_ohm = 0.0f;
    meas_settings.res_ke_v_per_rpm = 0.0f;
    for (uint8_t i = 0; i < RES_MAX_POINTS; i++)
    {
        meas_settings.res_point_rpm[i] = 0.0f;
        meas_settings.res_point_rth[i] = 0.0f;
        meas_settings.res_point_ke[i] = 0.0f;
    }
}

void gen_resistance_rewind_current_point()
{
    res_oc_rpm = 0.0f;
    load_switch_set(false);
    goto_phase(RES_PH_PREP);
    Serial.printf("[RES] rewind current point PREP idx=%u valid_points=%u\n",
                  (unsigned)meas_get_res_point_index(),
                  (unsigned)meas_get_res_valid_points());
}

// 整機安全鎖定：比照安全電流模組的 fail_hard()，效果等同 ESTOP，需重開機
static void fail_hard(const char *reason)
{
    load_switch_set(false);
    Serial.printf("[RES] HARD FAIL: %s\n", reason);
    speed_trigger_fault(FAULT_MEASURE_SAFETY);
    ledcWrite(MOTOR_PWM_PIN, 0);
}

static bool fail_contact(const char *reason)
{
    load_switch_set(false);
    Serial.printf("[RES] contact pause: %s\n", reason);
    meas_request_contact_pause(reason);
    return false;
}

static bool fail_ina_mismatch(const char *reason)
{
    load_switch_set(false);
    Serial.printf("[RES] INA V/I mismatch pause: %s\n", reason);
    meas_request_ina_mismatch_pause(reason);
    return false;
}

static void finish_averaging()
{
    float sum_rth = 0.0f;
    float sum_ke = 0.0f;
    uint8_t counted = 0;
    for (uint8_t i = 0; i < RES_MAX_POINTS; i++)
    {
        float rpm, rth, ke;
        if (meas_res_get_point(i, rpm, rth, ke) && rpm > 0.0f)
        {
            sum_rth += rth;
            sum_ke += ke;
            counted++;
        }
    }
    if (counted == 0)
    {
        fail_hard("no valid resistance points out of RES_MAX_POINTS attempts");
        return;
    }
    SettingsLockGuard lock(g_meas_mux);
    meas_settings.res_rth_ohm = sum_rth / (float)counted;
    meas_settings.res_ke_v_per_rpm = sum_ke / (float)counted;
    meas_settings.res_done = true;
}

bool gen_resistance_step(uint32_t now_ms)
{
    const uint8_t ph = meas_get_res_phase();
    const uint32_t elapsed = now_ms - meas_get_res_phase_start_ms();
    const uint8_t idx = meas_get_res_point_index();

    switch (ph)
    {
    case RES_PH_PREP:
    {
        if (idx >= (uint8_t)RES_MAX_POINTS)
        {
            finish_averaging();
            return true; // 成功(res_done=true)或硬故障(speed_get_fault()=true)都算「本模組結束」
        }
        load_switch_set(false);
        const float target = target_rpm_for_point(idx);
        meas_set_res_target_rpm(target);
        pid_set_keep_rpm(target);
        goto_phase(RES_PH_WAIT_SPEED);
        return false;
    }

    case RES_PH_WAIT_SPEED:
        if (speed_get_fault())
        {
            return false;
        }
        if (speed_get_speed_stable())
        {
            goto_phase(RES_PH_OC_SAMPLE);
            return false;
        }
        if (elapsed > speed_wait_timeout_ms(meas_get_res_target_rpm()))
        {
            fail_hard("timeout waiting for speed_stable at resistance point");
            return true;
        }
        return false;

    case RES_PH_OC_SAMPLE:
        if (elapsed < (uint32_t)RES_OC_SAMPLE_MS)
        {
            return false;
        }
        if (!(ina_get_online() && ina_get_data_valid()))
        {
            return fail_contact("INA offline during open-circuit sample");
        }
        {
            const float oc_a = fabsf(ina_get_current_A());
            if (oc_a > (float)SAFE_OC_MAX_CURRENT_A)
            {
                fail_hard("open-circuit current not near zero (load stuck closed?)");
                return true;
            }
            // 補償防反接 SS54 的順向壓降，換回發電機端子的真實 Voc；下面帶載那筆(RES_PH_LOAD_SAMPLE)
            // 也用同一套補償，R_th=(Voc-V)/I 兩邊才是同一個基準，不會因為補償不一致而算錯。
            const float voc = ss54_compensate_voltage_V(ina_get_bus_V(), oc_a);
            res_oc_rpm = speed_get_now_speed();
            meas_set_res_oc_voltage_V(voc);

            // 接通前預檢：帶載電流 I ≈ Voc/(R_th+R_load)，不可用 Voc/R_load(當 R_th=0)——
            // 會嚴重高估電流。安全電流已在各檔實測帶載 I_cont，同負載下 I 約正比轉速，
            // 故用 I_cont 按目標轉速外推；僅在尚無 I_cont 時才退回 Voc/R_load 粗估。
            const float i_cont = meas_get_safe_i_cont_A();
            const float i_allow = (i_cont > 0.0f) ? fminf(i_cont, (float)SAFE_I_HARD_CEILING_A)
                                                   : (float)SAFE_I_HARD_CEILING_A;
            const float target = meas_get_res_target_rpm();
            const float rpm_ref = meas_get_safe_i_cont_rpm();
            float predicted_i;
            if (i_cont > 0.0f && rpm_ref > 1.0f)
            {
                predicted_i = i_cont * (target / rpm_ref);
            }
            else
            {
                predicted_i = voc / (float)LOAD_TEST_RESISTOR_OHM;
            }
            if (predicted_i > i_allow)
            {
                fail_hard("predicted current at this speed exceeds allowed ceiling — should not happen "
                          "since this rpm was already validated by safe-current staircase");
                return true;
            }
        }
        goto_phase(RES_PH_CONNECT);
        return false;

    case RES_PH_CONNECT:
        if (!load_switch_is_connected())
        {
            load_switch_set(true);
            return false;
        }
        if (ina_get_online() && ina_get_data_valid())
        {
            const float i_now = fabsf(ina_get_current_A());
            if (i_now > (float)SAFE_I_HARD_CEILING_A)
            {
                fail_hard("hard current ceiling exceeded while connecting at resistance point");
                return true;
            }
        }
        // 接上負載會讓轉速掉一點，必須等 PID 把它拉回、speed_stable 重新成立，
        // 開路 Voc 與帶載 V/I 才是「同一轉速」下的兩個點，R_th 才準
        if (!speed_get_speed_stable())
        {
            if (elapsed > speed_wait_timeout_ms(meas_get_res_target_rpm()))
            {
                return fail_contact("speed did not restabilize after connecting load");
            }
            return false;
        }
        goto_phase(RES_PH_LOAD_SAMPLE);
        return false;

    case RES_PH_LOAD_SAMPLE:
        if (elapsed < (uint32_t)RES_LOAD_SAMPLE_MS)
        {
            if (ina_get_online() && ina_get_data_valid())
            {
                const float i_now = fabsf(ina_get_current_A());
                if (i_now > (float)SAFE_I_HARD_CEILING_A)
                {
                    fail_hard("hard current ceiling exceeded during load sample");
                    return true;
                }
            }
            return false;
        }
        {
            const bool ok = ina_get_online() && ina_get_data_valid();
            float v = 0.0f, i = 0.0f;
            if (ok)
            {
                i = fabsf(ina_get_current_A());
                // 同一顆 SS54，補償方式跟開路那筆(RES_PH_OC_SAMPLE)一致，只是這裡電流
                // 大很多，補償量也大很多——這正是不補償就會讓 R_th 系統性算錯的地方。
                v = ss54_compensate_voltage_V(ina_get_bus_V(), i);
            }
            if (!ok)
            {
                return fail_contact("INA offline during load sample");
            }
            const float voc = meas_get_res_oc_voltage_V();
            const bool looks_open = (i < (float)RES_MIN_VALID_A) && (voc > 1.0f) &&
                                    (ina_get_bus_V() >= voc * (float)SAFE_I_OPEN_V_RATIO);
            if (looks_open)
            {
                return fail_contact("load current too small during sample (load not connected?)");
            }
            if (!ina_get_current_plausible() || ina_loaded_vi_mismatch(ina_get_bus_V(), i) ||
                i < (float)RES_MIN_VALID_A)
            {
                if (elapsed >= (uint32_t)(RES_LOAD_SAMPLE_MS + INA_I_INCONSISTENT_PAUSE_MS))
                {
                    return fail_ina_mismatch("V still loaded but I disagrees with V/R — Rth not recorded");
                }
                Serial.printf("[RES] ignore I glitch I=%.3f V=%.2f Voc=%.2f, retry sample\n",
                              (double)i, (double)v, (double)voc);
                return false;
            }
            load_switch_set(false); // 取完立刻斷開，不做熱浸泡(內阻要快，避免量到熱態)
            meas_set_res_load_V(v);
            meas_set_res_load_A(i);
        }
        goto_phase(RES_PH_COMPUTE);
        return false;

    case RES_PH_COMPUTE:
    {
        const float voc = meas_get_res_oc_voltage_V();
        const float v = meas_get_res_load_V();
        const float i = meas_get_res_load_A();
        const float n_load = speed_get_now_speed();
        const float n_oc = res_oc_rpm;
        // 開路與帶載的轉速若還有一點差，把 Voc 乘上 n_load/n_oc，避免轉速沒完全拉回就把差算進 R_th。
        const float voc_at_load = (n_oc > 1.0f && n_load > 1.0f) ? (voc * (n_load / n_oc)) : voc;
        const float rpm_point = (n_load > 1.0f) ? n_load : meas_get_res_target_rpm();
        const float rth = (i > 1e-6f) ? ((voc_at_load - v) / i) : -1.0f;
        const float ke = (n_oc > 1.0f) ? (voc / n_oc) : 0.0f;

        if (!(isfinite(rth) && isfinite(ke)) || rth < (float)RES_MIN_OHM ||
            rth > (float)RES_MAX_OHM || ke <= 0.0f)
        {
            // 單點失效不當硬故障：跳過這點，繼續下一點；只有「全部點都失效」才在
            // finish_averaging() 判定失敗，避免一次雜訊就整個鎖死重來
            Serial.printf("[RES] point %u INVALID (discarded): Voc=%.3f V=%.3f I=%.3f -> Rth=%.4f\n",
                          (unsigned)idx, (double)voc, (double)v, (double)i, (double)rth);
        }
        else
        {
            meas_res_set_point(idx, rpm_point, rth, ke);
            SettingsLockGuard lock(g_meas_mux);
            meas_settings.res_valid_points = meas_settings.res_valid_points + 1;
            Serial.printf("[RES] point %u ok: rpm=%.0f Voc=%.3f(aligned %.3f) V=%.3f I=%.3f Rth=%.4f ke=%.6f\n",
                          (unsigned)idx, (double)rpm_point, (double)voc, (double)voc_at_load,
                          (double)v, (double)i, (double)rth, (double)ke);
        }

        meas_set_res_point_index(idx + 1);
        goto_phase(RES_PH_PREP);
        return false;
    }

    default:
        goto_phase(RES_PH_PREP);
        return false;
    }
}
