#include "curve_calc.h"
#include "debug_tee/debug_tee.h"

static float round_up_to_step(float v, float step)
{
    if (step <= 0.0f)
    {
        return v;
    }
    return ceilf(v / step) * step;
}

void curve_calc_run()
{
    const float rth = meas_get_res_rth_ohm();
    const float ke = meas_get_res_ke_v_per_rpm();
    const float i_cont = meas_get_safe_i_cont_A();
    // I_cont 理論上此時必為有效值(measure_seq 只有在安全電流成功完成後才會走到內阻→曲線；
    // 任一步失敗都會 fault 鎖定、不會推進到這裡)，這裡的 fallback 純屬防禦性寫法。
    const float i_allow = (i_cont > 0.0f) ? i_cont : (float)SAFE_I_HARD_CEILING_A;

    if (!(rth > 0.0f) || !(ke > 0.0f))
    {
        SettingsLockGuard lock(g_meas_mux);
        meas_settings.curve_point_count = 0;
        meas_settings.curve_n_lim = 0.0f;
        meas_settings.curve_limit_reason = LIMIT_REASON_NONE;
        Serial.println("[CURVE] aborted: invalid Rth/ke from resistance module");
        return;
    }

    // ---- 三個電氣封頂轉速(見 LIMIT_RPM_ARCH.md) ----
    const float n_knee = (2.0f * i_allow * rth) / ke;    // 最大功率點合法上限
    const float n_voc = (float)CURVE_V_ALLOW / ke;       // 開路耐壓上限(本機會開路，取這個而非 n_sat)
    const float n_rl = (i_allow * (rth + (float)LOAD_TEST_RESISTOR_OHM)) / ke; // 固定測試電阻功率封頂，僅供參考

    float n_lim = fminf(fminf(n_knee, n_voc), (float)SAFE_RPM_MAX_CEILING);
    const float brush_cap = meas_get_brush_jump_rpm();
    const float drive_cap = meas_get_drive_limit_rpm();
    const float thermal_cap = meas_get_thermal_limit_rpm();
    if (brush_cap > 1.0f)
    {
        n_lim = fminf(n_lim, brush_cap);
    }
    if (drive_cap > 1.0f)
    {
        n_lim = fminf(n_lim, drive_cap);
    }
    if (thermal_cap > 1.0f)
    {
        n_lim = fminf(n_lim, thermal_cap);
    }
    uint8_t reason;
    if (thermal_cap > 1.0f && n_lim >= thermal_cap - 1e-3f)
    {
        reason = LIMIT_REASON_WINDING_TEMP;
    }
    else if (drive_cap > 1.0f && n_lim >= drive_cap - 1e-3f)
    {
        reason = LIMIT_REASON_DRIVE_LIMIT;
    }
    else if (brush_cap > 1.0f && n_lim >= brush_cap - 1e-3f)
    {
        reason = LIMIT_REASON_BRUSH_JUMP;
    }
    else if (n_lim >= n_knee - 1e-3f)
    {
        reason = LIMIT_REASON_MPP_KNEE;
    }
    else if (n_lim >= n_voc - 1e-3f)
    {
        reason = LIMIT_REASON_OPEN_VOLTAGE;
    }
    else
    {
        reason = LIMIT_REASON_CEILING;
    }

    {
        SettingsLockGuard lock(g_meas_mux);
        meas_settings.curve_n_rl = n_rl;
        meas_settings.curve_n_knee = n_knee;
        meas_settings.curve_n_voc = n_voc;
        meas_settings.curve_n_lim = n_lim;
        meas_settings.curve_limit_reason = reason;
    }

    // ---- 逐點計算：從安全電流階梯實際的起始轉速(進位到 100 的倍數)開始，
    // 每 CURVE_RPM_STEP 一點，算到 n_lim。優先用實測到的起點(safe_start_rpm)，
    // 這樣報表上每一點都落在「這台機實際有驗證過會穩定運轉」的範圍以上；
    // 萬一(理論上不會發生)還沒有實測起點，才退回 MEASURE_MIN_RPM 這個常數當備援。
    const float measured_start = meas_get_safe_start_rpm();
    const float base_rpm = (measured_start > 1.0f) ? measured_start : (float)MEASURE_MIN_RPM;
    const float start_rpm = round_up_to_step(base_rpm, (float)CURVE_RPM_STEP);
    uint16_t count = 0;
    for (float rpm = start_rpm;
         rpm <= n_lim && count < (uint16_t)MAX_CURVE_POINTS;
         rpm += (float)CURVE_RPM_STEP)
    {
        const float voc = ke * rpm;
        const float i_mpp = voc / (2.0f * rth); // 若接 R_opt=R_th 時会流过的电流

        float v, p, r_opt;
        if (i_mpp <= i_allow)
        {
            // 仍在合法最大功率點範圍：R_opt=R_th，V=Voc/2，P=Voc^2/(4*Rth)
            r_opt = rth;
            v = voc * 0.5f;
            p = (voc * voc) / (4.0f * rth);
        }
        else
        {
            // 超過 n_knee：改成「電流卡在 I_allow」下可達到的最大功率
            r_opt = (voc / i_allow) - rth;
            v = voc - i_allow * rth;
            p = v * i_allow;
        }
        meas_curve_set_point(count, rpm, v, p, r_opt);
        count++;
    }
    meas_set_curve_point_count(count);

    Serial.printf("[CURVE] Rth=%.4f ke=%.6f I_allow=%.3f -> n_knee=%.0f n_voc=%.0f n_rl=%.0f "
                  "n_lim=%.0f(reason=%u) points=%u\n",
                  (double)rth, (double)ke, (double)i_allow, (double)n_knee, (double)n_voc,
                  (double)n_rl, (double)n_lim, (unsigned)reason, (unsigned)count);
}
