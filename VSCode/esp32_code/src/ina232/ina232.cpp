#include "ina232.h"
#include "soft_i2c/soft_i2c.h"

// 共享感測結果(其他模組只讀)
volatile ina232_sensor ina_settings{};

// ---- 暫存器位址(INA232 datasheet Table 7-2) ----
static constexpr uint8_t INA_REG_CONFIG = 0x00;
static constexpr uint8_t INA_REG_SHUNT = 0x01;
static constexpr uint8_t INA_REG_BUS = 0x02;
static constexpr uint8_t INA_REG_CURRENT = 0x04;
static constexpr uint8_t INA_REG_CAL = 0x05;
static constexpr uint8_t INA_REG_MFG_ID = 0x3E;
static constexpr uint16_t INA_MFG_ID_TI = 0x5449; // "TI" ASCII

static constexpr float INA_BUS_LSB_V = 0.0016f;
#if INA232_ADCRANGE_80MV
static constexpr float INA_SHUNT_LSB_V = 0.0000025f;
#else
static constexpr float INA_SHUNT_LSB_V = 0.000000625f;
#endif

static SoftI2C ina_bus;
static uint8_t ina_addr = INA232_I2C_ADDR;
static float current_lsb_A = 0.0f;
static float ema_V = 0.0f;
static float ema_I = 0.0f;
static float ema_P = 0.0f;
static bool ema_inited = false;
static uint32_t i_mismatch_last_log_ms = 0;

// ★記憶體保護★：本檔案是 ina_settings 的唯一寫入者，對外發布一律透過
// settings.h 提供的 ina_get_x()/ina_set_x()/ina_increment_sample_count() 介面，
// 一次要發布好幾個彼此相關欄位(一次成功讀值的 V/I/P/shunt/data_valid)時，
// 用 SettingsLockGuard 包住整段直接寫欄位，確保其他任務讀到的是同一次讀值的結果。

static bool ina_write16(uint8_t reg, uint16_t value)
{
    return ina_bus.writeReg16(ina_addr, reg, value);
}

static bool ina_read16(uint8_t reg, uint16_t &value)
{
    return ina_bus.readReg16(ina_addr, reg, value);
}

static bool ina_configure()
{
    current_lsb_A = INA232_IMAX_A / 32768.0f;
    if (current_lsb_A < 1e-9f)
    {
        return false;
    }

    float cal_f = 0.00512f / (current_lsb_A * INA232_RSHUNT_OHM);
#if !INA232_ADCRANGE_80MV
    cal_f *= 0.25f;
#endif
    uint16_t cal = (uint16_t)(cal_f + 0.5f);
    if (cal == 0)
    {
        cal = 1;
    }
    if (cal > 0x7FFF)
    {
        cal = 0x7FFF;
    }

    if (!ina_write16(INA_REG_CONFIG, 0x8000))
    {
        return false;
    }
    delay(2);

    // bit14-13 保留=10b、ADCRANGE、AVG、VBUSCT=140µs、VSHCT=140µs、MODE=連續分流+匯流排
    uint16_t config = 0x4007;
    config &= (uint16_t)~(7u << 9);
    config |= (uint16_t)(((unsigned)INA232_AVG_CODE & 7u) << 9);
#if !INA232_ADCRANGE_80MV
    config |= (1u << 12);
#endif
    if (!ina_write16(INA_REG_CONFIG, config) || !ina_write16(INA_REG_CAL, cal))
    {
        return false;
    }

    Serial.printf("[INA] cfg=0x%04X cal=%u Current_LSB=%.6e A Rshunt=%.3fΩ Imax=%.2fA\n",
                  (unsigned)config, (unsigned)cal,
                  (double)current_lsb_A,
                  (double)INA232_RSHUNT_OHM,
                  (double)INA232_IMAX_A);
    return true;
}

static bool ina_probe_at(uint8_t addr)
{
    uint16_t mfg = 0;
    const uint8_t saved = ina_addr;
    ina_addr = addr;
    const bool ok = ina_read16(INA_REG_MFG_ID, mfg) && (mfg == INA_MFG_ID_TI);
    if (!ok)
    {
        ina_addr = saved;
    }
    return ok;
}

static bool ina_sample_once()
{
    uint16_t raw_shunt = 0;
    uint16_t raw_bus = 0;
    uint16_t raw_current = 0;

    if (!ina_read16(INA_REG_SHUNT, raw_shunt) ||
        !ina_read16(INA_REG_BUS, raw_bus) ||
        !ina_read16(INA_REG_CURRENT, raw_current))
    {
        return false;
    }

    const float shunt_mV = (float)(int16_t)raw_shunt * INA_SHUNT_LSB_V * 1000.0f;
    const float bus_V = (float)(raw_bus & 0x7FFF) * INA_BUS_LSB_V;
    const float i_reg = (float)(int16_t)raw_current * current_lsb_A;
    const float i_shunt = (shunt_mV * 0.001f) / (float)INA232_RSHUNT_OHM;
    float current_A = i_reg;
    // CURRENT 暫存器偶發 I2C/校正亂數時，分流電壓仍是類比量；兩者差太多就信分流。
    {
        const float scale = fmaxf(fabsf(i_reg), fabsf(i_shunt));
        if (fabsf(i_reg - i_shunt) > fmaxf(0.04f, 0.25f * scale))
        {
            current_A = i_shunt;
        }
    }

    // 負載接通且端子已被測試電阻拉低時，I 必須約等於 V/R。對不上就標不可信、發布原始 0A（讓你看得見），
    // 但不把 0 寫進 EMA，也不准量測模組拿去當 I_cont。不可用上一筆電流假裝還在測。
    const bool mismatch = ema_inited && meas_get_load_connected() &&
                          ina_loaded_vi_mismatch(bus_V, current_A);
    if (mismatch)
    {
        const uint32_t now_ms = millis();
        if (i_mismatch_last_log_ms == 0 || (now_ms - i_mismatch_last_log_ms) >= 200u)
        {
            i_mismatch_last_log_ms = now_ms;
            Serial.printf("[INA] V/I mismatch (not used for test) I=%.3f V=%.2f expect=%.3f\n",
                          (double)current_A, (double)bus_V,
                          (double)(bus_V / (float)LOAD_TEST_RESISTOR_OHM));
        }
    }
    else
    {
        i_mismatch_last_log_ms = 0;
    }

    const float power_W = fabsf(bus_V * current_A);

    if (!ema_inited)
    {
        ema_V = bus_V;
        ema_I = current_A;
        ema_P = power_W;
        ema_inited = true;
    }
    else
    {
        const float a = INA232_FILTER_ALPHA;
        ema_V += a * (bus_V - ema_V);
        if (!mismatch)
        {
            ema_I += a * (current_A - ema_I);
            ema_P += a * (power_W - ema_P);
        }
    }

    // 這組欄位代表「這一次成功讀值」的完整結果，整段上鎖一起發布。
    // mismatch 時 current_A 發原始讀值（可能是 0），current_plausible=false。
    {
        SettingsLockGuard lock(g_ina_mux);
        ina_settings.shunt_mV = shunt_mV;
        ina_settings.bus_V = ema_V;
        ina_settings.current_A = mismatch ? current_A : ema_I;
        ina_settings.power_W = mismatch ? power_W : ema_P;
        ina_settings.data_valid = true;
        ina_settings.current_plausible = !mismatch;
    }
    ina_increment_sample_count();
    return true;
}

static void ina232_task(void *pvParameters)
{
    (void)pvParameters;

    // 等 OLED 先完成開機畫面，再佔用 CPU 做 bitbang
    vTaskDelay(pdMS_TO_TICKS(300));

    // ★軟體 I2C：完全不走 Wire/硬體 I2C NG，避免 INVALID_STATE 卡死刷屏
    ina_bus.begin(SDA2_PIN, SCL2_PIN, (uint32_t)I2C_INA_FREQ_HZ);
    Serial.printf("[INA] soft-I2C SDA=%u SCL=%u freq=%uHz try_addr=0x%02X\n",
                  (unsigned)SDA2_PIN, (unsigned)SCL2_PIN,
                  (unsigned)I2C_INA_FREQ_HZ, (unsigned)INA232_I2C_ADDR);

    bool ok = ina_probe_at(INA232_I2C_ADDR);
    if (!ok)
    {
        // A0 接法可能不是 GND：掃一遍找 TI ID
        Serial.println("[INA] default addr miss, scanning...");
        for (uint8_t a = 0x40; a <= 0x4F; a++)
        {
            if (ina_probe_at(a))
            {
                ok = true;
                Serial.printf("[INA] found at 0x%02X\n", (unsigned)a);
                break;
            }
        }
        if (!ok)
        {
            const uint8_t any = ina_bus.scanFirst();
            Serial.printf("[INA] not found. first_ack=0x%02X (FF=none). "
                          "Check soldering / A0 / add 4.7k pullups on SDA2/SCL2\n",
                          (unsigned)any);
        }
    }

    ina_set_online(ok);
    if (!ok)
    {
        while (1)
        {
            vTaskDelay(pdMS_TO_TICKS(2000));
            ina_bus.begin(SDA2_PIN, SCL2_PIN, (uint32_t)I2C_INA_FREQ_HZ);
            if (ina_probe_at(INA232_I2C_ADDR) || ina_probe_at(0x41) ||
                ina_probe_at(0x42) || ina_probe_at(0x43))
            {
                ina_set_online(true);
                Serial.printf("[INA] recovered addr=0x%02X\n", (unsigned)ina_addr);
                break;
            }
        }
    }
    else
    {
        Serial.printf("[INA] online addr=0x%02X (soft-I2C)\n", (unsigned)ina_addr);
    }

    while (!ina_configure())
    {
        Serial.println("[INA] configure failed, retry...");
        ina_set_online(false);
        vTaskDelay(pdMS_TO_TICKS(1000));
        ina_bus.begin(SDA2_PIN, SCL2_PIN, (uint32_t)I2C_INA_FREQ_HZ);
        if (!ina_probe_at(ina_addr))
        {
            continue;
        }
        ina_set_online(true);
    }

    uint32_t fail_streak = 0;

    while (1)
    {
        if (ina_sample_once())
        {
            fail_streak = 0;
            ina_set_online(true);
        }
        else
        {
            fail_streak++;
            if (fail_streak == 50u)
            {
                ina_set_online(false);
                Serial.println("[INA] read fail streak");
            }
            if ((fail_streak % 100u) == 0u)
            {
                ina_bus.begin(SDA2_PIN, SCL2_PIN, (uint32_t)I2C_INA_FREQ_HZ);
                if (ina_probe_at(ina_addr) && ina_configure())
                {
                    fail_streak = 0;
                    ema_inited = false;
                    i_mismatch_last_log_ms = 0;
                    ina_set_online(true);
                    Serial.println("[INA] reconfigure ok");
                }
            }
            if (!ina_get_online())
            {
                vTaskDelay(pdMS_TO_TICKS(200));
                continue;
            }
        }

        // ★務必用 vTaskDelay(絕對會讓出 CPU)，不可用 vTaskDelayUntil：
        // 軟體 I2C bit-bang 讀 3 個暫存器實際耗時常 > INA232_TASK_PERIOD_MS(1ms)，
        // vTaskDelayUntil 一旦「已經過期」會直接返回、完全不讓出 CPU，
        // 導致本任務(優先權 12，Core1)把 loop()/board_ui 都餓死(START 按鍵偵測失效)。
        vTaskDelay(pdMS_TO_TICKS(INA232_TASK_PERIOD_MS));
    }
}

void ina232_start()
{
    // ★釘在 Core 0(與 PID/測速同核心，但優先權較低)：
    // 讓 Core 1(Arduino loop() 的 START 按鍵輪詢 + board_ui)完全不受軟體 I2C bit-bang 影響。
    xTaskCreatePinnedToCore(
        ina232_task,
        "ina232",
        4096,
        NULL,
        RTOS_INA232_LEVEL,
        NULL,
        0);
}
