#include "bt_telemetry.h"
#include "BluetoothSerial.h"
#include "debug_tee/debug_tee.h"
#include <cstdarg>
#include <cstdio>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled on this build (BluetoothSerial requires Bluedroid classic BT).
#endif

// 藍牙裝置名稱：上位機掃描配對時會看到這個名字
static const char *BT_DEVICE_NAME = "MicroGenerator-ESP32";

static BluetoothSerial SerialBT;
volatile bt_telemetry_state bt_settings{};

static constexpr uint32_t LIVE_PERIOD_MS = 200;   // 即時欄位推播週期(5Hz)
static constexpr uint32_t CURVE_PERIOD_MS = 3000; // 曲線表重送週期(靜態資料，較慢即可)

// ★固定大小、檔案層級(非堆疊、非動態配置)緩衝區，供 snprintf 現場組字串：
// 避免在 FreeRTOS 任務堆疊上放大陣列(有溢出風險)，也避免 String/malloc 造成的
// heap 碎片化與配置失敗風險——整個量測序列在跑的過程中，這兩塊緩衝區大小固定、
// 生命週期等於整個程式執行期，記憶體占用可以在編譯期就精確算出來。
static char live_buf[2560];
static char curve_buf[6144];

// 邊界安全的字串附加：用剩餘容量夾住 snprintf，任何情況都不會寫出 buf 範圍，
// 也不會因為單次 snprintf 被截斷就讓後面的欄位對不上逗號(用回傳值检查是否被截斷)。
static void append(char *buf, size_t buf_size, size_t &len, const char *fmt, ...)
{
    if (len >= buf_size)
    {
        return;
    }
    va_list args;
    va_start(args, fmt);
    const int n = vsnprintf(buf + len, buf_size - len, fmt, args);
    va_end(args);
    if (n > 0)
    {
        len += (size_t)n;
        if (len > buf_size)
        {
            len = buf_size; // 已被截斷：之後的 append 會因為 len>=buf_size 直接跳過，避免拼出破損 JSON
        }
    }
}

static const char *speed_phase_str(uint8_t ph)
{
    switch (ph)
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

static const char *fault_code_str(uint8_t code)
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

static const char *ui_state_str(uint8_t st)
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

static const char *meas_phase_str(uint8_t ph)
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

static void build_live_message()
{
    size_t len = 0;
    live_buf[0] = '\0';

    append(live_buf, sizeof(live_buf), len,
           "{\"t\":\"live\",\"ts\":%lu,"
           "\"ui_state\":%u,\"ui_state_name\":\"%s\","
           "\"fault\":%d,\"fault_code\":%u,\"fault_name\":\"%s\","
           "\"phase\":%u,\"phase_name\":\"%s\","
           "\"now_speed\":%.1f,\"keep_rpm\":%.1f,"
           "\"speed_valid\":%d,\"speed_stable\":%d,\"const_speed_ready\":%d,",
           (unsigned long)millis(),
           (unsigned)ui_get_state(), ui_state_str(ui_get_state()),
           (int)speed_get_fault(), (unsigned)speed_get_fault_code(), fault_code_str(speed_get_fault_code()),
           (unsigned)speed_get_init_phase(), speed_phase_str(speed_get_init_phase()),
           (double)speed_get_now_speed(), (double)pid_get_keep_rpm(),
           (int)speed_get_speed_valid(), (int)speed_get_speed_stable(), (int)speed_get_const_speed_ready());

    append(live_buf, sizeof(live_buf), len,
           "\"ina_online\":%d,\"ina_valid\":%d,\"ina_plausible\":%d,"
           "\"bus_V\":%.3f,\"current_A\":%.4f,\"power_W\":%.3f,",
           (int)ina_get_online(), (int)ina_get_data_valid(), (int)ina_get_current_plausible(),
           (double)ina_get_bus_V(), (double)ina_get_current_A(), (double)ina_get_power_W());

    append(live_buf, sizeof(live_buf), len,
           "\"meas_phase\":%u,\"meas_phase_name\":\"%s\",\"resume_phase\":%u,"
           "\"link_lost\":%d,\"pause_cause\":%u,\"load_connected\":%d,\"session_active\":%d,",
           (unsigned)meas_get_phase(), meas_phase_str(meas_get_phase()), (unsigned)meas_get_resume_phase(),
           (int)meas_get_link_lost(), (unsigned)meas_get_pause_cause(),
           (int)meas_get_load_connected(), (int)meas_get_session_active());

    append(live_buf, sizeof(live_buf), len,
           "\"safe_phase\":%u,\"safe_target_rpm\":%.0f,\"safe_oc_V\":%.3f,"
           "\"safe_electrical_A\":%.4f,\"safe_hot_A\":%.4f,\"safe_droop_ratio\":%.4f,"
           "\"safe_rth_ohm\":%.3f,\"safe_winding_rise_C\":%.1f,\"safe_winding_rise_limit_C\":%.1f,"
           "\"safe_phase_elapsed_ms\":%lu,\"safe_done\":%d,\"safe_pass_any\":%d,"
           "\"safe_i_cont_A\":%.4f,\"safe_i_cont_rpm\":%.0f,",
           (unsigned)meas_get_safe_phase(), (double)meas_get_safe_target_rpm(),
           (double)meas_get_safe_oc_voltage_V(), (double)meas_get_safe_electrical_A(),
           (double)meas_get_safe_hot_A(), (double)meas_get_safe_droop_ratio(),
           (double)meas_get_safe_rth_ohm(), (double)meas_get_safe_winding_rise_C(),
           (double)meas_get_safe_winding_rise_limit_C(),
           (unsigned long)(millis() - meas_get_safe_phase_start_ms()),
           (int)meas_get_safe_done(), (int)meas_get_safe_pass_any(),
           (double)meas_get_safe_i_cont_A(), (double)meas_get_safe_i_cont_rpm());

    append(live_buf, sizeof(live_buf), len,
           "\"res_phase\":%u,\"res_point_index\":%u,\"res_target_rpm\":%.0f,"
           "\"res_oc_V\":%.3f,\"res_load_V\":%.3f,\"res_load_A\":%.4f,"
           "\"res_phase_elapsed_ms\":%lu,\"res_done\":%d,\"res_valid_points\":%u,"
           "\"res_rth_ohm\":%.4f,\"res_ke_v_per_rpm\":%.6f,",
           (unsigned)meas_get_res_phase(), (unsigned)meas_get_res_point_index(),
           (double)meas_get_res_target_rpm(), (double)meas_get_res_oc_voltage_V(),
           (double)meas_get_res_load_V(), (double)meas_get_res_load_A(),
           (unsigned long)(millis() - meas_get_res_phase_start_ms()),
           (int)meas_get_res_done(), (unsigned)meas_get_res_valid_points(),
           (double)meas_get_res_rth_ohm(), (double)meas_get_res_ke_v_per_rpm());

    append(live_buf, sizeof(live_buf), len,
           "\"curve_done\":%d,\"curve_n_rl\":%.0f,\"curve_n_knee\":%.0f,"
           "\"curve_n_voc\":%.0f,\"curve_n_lim\":%.0f,\"curve_limit_reason\":%u,"
           "\"curve_point_count\":%u}",
           (int)meas_get_curve_done(), (double)meas_get_curve_n_rl(), (double)meas_get_curve_n_knee(),
           (double)meas_get_curve_n_voc(), (double)meas_get_curve_n_lim(),
           (unsigned)meas_get_curve_limit_reason(), (unsigned)meas_get_curve_point_count());
}

static void build_curve_message()
{
    size_t len = 0;
    curve_buf[0] = '\0';
    const uint16_t count = meas_get_curve_point_count();

    append(curve_buf, sizeof(curve_buf), len, "{\"t\":\"curve\",\"n\":%u,\"pts\":[", (unsigned)count);
    for (uint16_t i = 0; i < count; i++)
    {
        float rpm = 0, v = 0, p = 0, r_opt = 0;
        if (!meas_curve_get_point(i, rpm, v, p, r_opt))
        {
            break;
        }
        append(curve_buf, sizeof(curve_buf), len, "%s[%.0f,%.3f,%.3f,%.4f]",
               (i == 0) ? "" : ",", (double)rpm, (double)v, (double)p, (double)r_opt);
    }
    append(curve_buf, sizeof(curve_buf), len, "]}");
}

// settings.h 的巨集只產生單純 get/set，遞增計數比照 ina_increment_sample_count() 的寫法自行實作，
// 避免又在 settings.h 為了這一個欄位多開一個巨集分支。
static inline void bt_increment_publish_count_wrapper()
{
    SettingsLockGuard lock(g_bt_mux);
    bt_settings.publish_count = bt_settings.publish_count + 1;
}

static char cmd_buf[48];
static size_t cmd_len = 0;

// 上位機一行：RISE <溫升°C>。只接受 10～200，避免空字串或亂碼把上限清掉。
static void bt_handle_command(const char *line)
{
    float rise = 0.0f;
    if (sscanf(line, "RISE %f", &rise) != 1)
    {
        return;
    }
    if (rise < 10.0f)
    {
        rise = 10.0f;
    }
    else if (rise > 200.0f)
    {
        rise = 200.0f;
    }
    meas_set_safe_winding_rise_limit_C(rise);
    Serial.printf("[BT] winding rise limit %.1f C\n", (double)rise);
}

static void bt_poll_commands()
{
    while (SerialBT.available() > 0)
    {
        const int c = SerialBT.read();
        if (c < 0)
        {
            break;
        }
        if (c == '\r')
        {
            continue;
        }
        if (c == '\n')
        {
            cmd_buf[cmd_len] = '\0';
            if (cmd_len > 0)
            {
                bt_handle_command(cmd_buf);
            }
            cmd_len = 0;
            continue;
        }
        if (cmd_len + 1 < sizeof(cmd_buf))
        {
            cmd_buf[cmd_len++] = (char)c;
        }
        else
        {
            cmd_len = 0;
        }
    }
}

static void bt_telemetry_task(void *pvParameters)
{
    (void)pvParameters;

    SerialBT.begin(BT_DEVICE_NAME);
    Serial.printf("[BT] SerialBT started, device name=\"%s\"\n", BT_DEVICE_NAME);

    uint32_t last_live_ms = 0;
    uint32_t last_curve_ms = 0;

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(20));
        const uint32_t now = millis();

        const bool has_client = SerialBT.hasClient();
        bt_set_client_connected(has_client);
        if (!has_client)
        {
            continue; // 沒人連線就留在佇列裡；連上後再把最近的除錯行送出
        }

        bt_poll_commands();
        debug_log_drain(SerialBT, 6);

        if ((now - last_live_ms) >= LIVE_PERIOD_MS)
        {
            last_live_ms = now;
            build_live_message();
            SerialBT.print(live_buf);
            SerialBT.print('\n');
            bt_increment_publish_count_wrapper();
        }

        if (meas_get_curve_done() && ((now - last_curve_ms) >= CURVE_PERIOD_MS))
        {
            last_curve_ms = now;
            build_curve_message();
            SerialBT.print(curve_buf);
            SerialBT.print('\n');
        }
    }
}

void bt_telemetry_start()
{
    xTaskCreatePinnedToCore(
        bt_telemetry_task,
        "bt_telemetry",
        6144,
        NULL,
        RTOS_BT_TELEMETRY_LEVEL,
        NULL,
        1);
}
