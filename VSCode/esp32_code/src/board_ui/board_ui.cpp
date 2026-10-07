#include "board_ui.h"
#include "soft_i2c/soft_i2c.h"
#include <U8g2lib.h>
#include "debug_tee/debug_tee.h"

volatile board_ui ui_settings{};

static SoftI2C oled_bus;
static bool oled_tx_ok = false;

static uint8_t u8x8_byte_soft_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    switch (msg)
    {
    case U8X8_MSG_BYTE_INIT:
        oled_bus.begin(SDA_PIN, SCL_PIN, (uint32_t)I2C_OLED_FREQ_HZ);
        return 1;
    case U8X8_MSG_BYTE_SET_DC:
        return 1;
    case U8X8_MSG_BYTE_START_TRANSFER:
    {
        const uint8_t addr7 = (uint8_t)(u8x8_GetI2CAddress(u8x8) >> 1);
        oled_tx_ok = oled_bus.startWrite(addr7);
        return oled_tx_ok ? 1 : 0;
    }
    case U8X8_MSG_BYTE_SEND:
    {
        const uint8_t *data = static_cast<uint8_t *>(arg_ptr);
        while (arg_int > 0)
        {
            if (!oled_bus.writeRaw(*data++))
            {
                oled_tx_ok = false;
            }
            arg_int--;
        }
        return 1;
    }
    case U8X8_MSG_BYTE_END_TRANSFER:
        oled_bus.endWrite();
        return oled_tx_ok ? 1 : 0;
    default:
        return 0;
    }
}

class U8G2_SSD1306_128X64_SOFT : public U8G2
{
public:
    U8G2_SSD1306_128X64_SOFT(const u8g2_cb_t *rotation) : U8G2()
    {
        u8g2_Setup_ssd1306_i2c_128x64_noname_f(
            &u8g2, rotation, u8x8_byte_soft_i2c, u8x8_gpio_and_delay_arduino);
    }
};

class U8G2_SH1106_128X64_SOFT : public U8G2
{
public:
    U8G2_SH1106_128X64_SOFT(const u8g2_cb_t *rotation) : U8G2()
    {
        u8g2_Setup_sh1106_i2c_128x64_noname_f(
            &u8g2, rotation, u8x8_byte_soft_i2c, u8x8_gpio_and_delay_arduino);
    }
};

static U8G2_SSD1306_128X64_SOFT u8g2_ssd(U8G2_R0);
static U8G2_SH1106_128X64_SOFT u8g2_sh(U8G2_R0);
static U8G2 *u8g2 = &u8g2_ssd;

static const char *phase_label(uint8_t phase)
{
    switch (phase)
    {
    case SPEED_PHASE_STEP_UP:
        return "STEP";
    case SPEED_PHASE_PROBE:
        return "PROBE";
    case SPEED_PHASE_LEARNING:
        return "LEARN";
    case SPEED_PHASE_READY:
        return "READY";
    case SPEED_PHASE_FAULT:
        return "FAULT";
    case SPEED_PHASE_PID_TUNE:
        return "TUNE";
    case SPEED_PHASE_PID_RUN:
        return "RUN";
    case SPEED_PHASE_PID_PAUSED:
        return "PAUSE";
    default:
        return "?";
    }
}

static const char *meas_phase_label(uint8_t ph)
{
    switch (ph)
    {
    case MEAS_MIN_SPEED_HOLD:
        return "MIN_SPD";
    case MEAS_SAFE_CURRENT:
        return "SAFE_I";
    case MEAS_RESISTANCE:
        return "RES";
    case MEAS_CURVE_CALC:
        return "CURVE";
    case MEAS_DONE:
        return "DONE";
    default:
        return "?";
    }
}

static void draw_wait_start()
{
    u8g2->clearBuffer();
    u8g2->setFont(u8g2_font_6x12_tf);
    u8g2->drawStr(0, 24, "Press START");
    u8g2->drawStr(0, 44, "to begin test");
    u8g2->sendBuffer();
}

// ★記憶體保護★：本檔案只「讀」其他模組的共享狀態拿去畫面顯示，
// 一律透過 settings.h 提供的 speed_get_x()/pid_get_x()/ina_get_x() 介面存取，
// 不直接碰 settings/PID_settings/ina_settings 的欄位。
static void draw_running()
{
    char line[32];

    u8g2->clearBuffer();
    u8g2->setFont(u8g2_font_6x12_tf);

    const uint8_t mphase = meas_get_phase();
    if (speed_get_fault())
    {
        snprintf(line, sizeof(line), "STAT FAULT#%u",
                 (unsigned)speed_get_fault_code());
    }
    else if (mphase == MEAS_SAFE_CURRENT || mphase == MEAS_RESISTANCE ||
             mphase == MEAS_CURVE_CALC || mphase == MEAS_DONE)
    {
        // 量測序列已進入具名階段後，顯示這個比底層測速/PID 階段更有意義
        snprintf(line, sizeof(line), "STAT %s", meas_phase_label(mphase));
    }
    else
    {
        snprintf(line, sizeof(line), "STAT %s%s",
                 phase_label(speed_get_init_phase()),
                 speed_get_speed_stable() ? " OK" : "");
    }
    u8g2->drawStr(0, 10, line);

    snprintf(line, sizeof(line), "RPM  %.0f/%.0f",
             (double)speed_get_now_speed(), (double)pid_get_keep_rpm());
    u8g2->drawStr(0, 22, line);

    if (ina_get_online() && ina_get_data_valid())
    {
        snprintf(line, sizeof(line), "I    %.3f A", (double)ina_get_current_A());
        u8g2->drawStr(0, 34, line);
        snprintf(line, sizeof(line), "V    %.2f V", (double)ina_get_bus_V());
        u8g2->drawStr(0, 46, line);
        snprintf(line, sizeof(line), "P    %.2f W", (double)ina_get_power_W());
        u8g2->drawStr(0, 58, line);
    }
    else
    {
        u8g2->drawStr(0, 34, "I/V/P  -- (INA offline)");
        u8g2->drawStr(0, 58, "START=ESTOP");
    }

    u8g2->sendBuffer();
}

static void draw_estop()
{
    u8g2->clearBuffer();
    u8g2->setFont(u8g2_font_6x12_tf);
    u8g2->drawStr(0, 20, "EMERGENCY STOP");
    u8g2->drawStr(0, 40, "ESTOP LOCKED");
    u8g2->drawStr(0, 56, "Reboot to reset");
    u8g2->sendBuffer();
}

// 夾子／量測線鬆脫暫停：非故障，不需重開機，畫面刻意與 ESTOP 不同
static void draw_link_lost()
{
    char line[32];
    u8g2->clearBuffer();
    u8g2->setFont(u8g2_font_6x12_tf);
    if (meas_get_pause_cause() == PAUSE_CAUSE_INA_MISMATCH)
    {
        u8g2->drawStr(0, 12, "INA I/V BAD");
        u8g2->drawStr(0, 26, "Sample discarded");
    }
    else
    {
        u8g2->drawStr(0, 12, "CLIPS LOOSE");
        u8g2->drawStr(0, 26, "Check gen/load clips");
    }
    snprintf(line, sizeof(line), "was: %s", meas_phase_label(meas_get_resume_phase()));
    u8g2->drawStr(0, 40, line);
    u8g2->drawStr(0, 56, "START continues");
    u8g2->sendBuffer();
}

static void draw_frame()
{
    switch (ui_get_state())
    {
    case UI_WAIT_START:
        draw_wait_start();
        break;
    case UI_RUNNING:
        draw_running();
        break;
    case UI_ESTOP:
        draw_estop();
        break;
    case UI_LINK_LOST:
        draw_link_lost();
        break;
    default:
        break;
    }
}

bool board_ui_oled_init()
{
    Serial.printf("[UI] OLED probe SoftI2C SDA=%u SCL=%u freq=%uHz\n",
                  (unsigned)SDA_PIN, (unsigned)SCL_PIN, (unsigned)I2C_OLED_FREQ_HZ);

    oled_bus.begin(SDA_PIN, SCL_PIN, (uint32_t)I2C_OLED_FREQ_HZ);

    uint8_t addr = 0xFF;
    if (oled_bus.probe(OLED_I2C_ADDR))
    {
        addr = OLED_I2C_ADDR;
    }
    else if (oled_bus.probe(0x3D))
    {
        addr = 0x3D;
    }
    else
    {
        addr = oled_bus.scanFirst();
    }

    if (addr == 0xFF)
    {
        Serial.println("[UI] OLED no ACK — check VCC/GND/wiring");
        ui_set_oled_ok(false);
        return false;
    }
    Serial.printf("[UI] OLED ACK addr=0x%02X\n", (unsigned)addr);

#if OLED_CONTROLLER_FORCE == 2
    // 強制 SH1106(settings.h 手動切換，用於排除 SSD1306 假成功的情況)
    Serial.println("[UI] OLED_CONTROLLER_FORCE=2 -> force SH1106");
    u8g2 = &u8g2_sh;
    u8g2->setI2CAddress(addr << 1);
    if (!u8g2->begin())
    {
        ui_set_oled_ok(false);
        Serial.println("[UI] OLED begin failed (forced SH1106)");
        return false;
    }
    Serial.printf("[UI] OLED ok SH1106(forced) addr=0x%02X\n", (unsigned)addr);
#elif OLED_CONTROLLER_FORCE == 1
    // 強制 SSD1306
    Serial.println("[UI] OLED_CONTROLLER_FORCE=1 -> force SSD1306");
    u8g2 = &u8g2_ssd;
    u8g2->setI2CAddress(addr << 1);
    if (!u8g2->begin())
    {
        ui_set_oled_ok(false);
        Serial.println("[UI] OLED begin failed (forced SSD1306)");
        return false;
    }
    Serial.printf("[UI] OLED ok SSD1306(forced) addr=0x%02X\n", (unsigned)addr);
#else
    // 自動：先試 SSD1306，失敗才試 SH1106
    // ★注意：兩種控制器初始化指令大多共通，若晶片實際是 SH1106，
    // SSD1306 的 begin() 仍可能回傳 true(I2C 有 ACK)，但畫面不會顯示。
    // 若遇到「addr 找到、begin 成功、卻不顯示」，請改用 OLED_CONTROLLER_FORCE=2 測試。
    u8g2 = &u8g2_ssd;
    u8g2->setI2CAddress(addr << 1);
    Serial.println("[UI] begin SSD1306...");
    if (!u8g2->begin())
    {
        Serial.println("[UI] try SH1106...");
        u8g2 = &u8g2_sh;
        u8g2->setI2CAddress(addr << 1);
        if (!u8g2->begin())
        {
            ui_set_oled_ok(false);
            Serial.println("[UI] OLED begin failed");
            return false;
        }
        Serial.printf("[UI] OLED ok SH1106 addr=0x%02X\n", (unsigned)addr);
    }
    else
    {
        Serial.printf("[UI] OLED ok SSD1306 addr=0x%02X\n", (unsigned)addr);
    }
#endif

    u8g2->setPowerSave(0);
    u8g2->setContrast(255);
    ui_set_oled_ok(true);
    ui_set_state(UI_WAIT_START);
    draw_wait_start();
    return true;
}

static void board_ui_task(void *pvParameters)
{
    (void)pvParameters;
    uint32_t last_draw_ms = 0;

    while (1)
    {
        const uint32_t now = millis();
        if (ui_get_oled_ok() &&
            ((now - last_draw_ms) >= (uint32_t)UI_REFRESH_MS))
        {
            last_draw_ms = now;
            draw_frame();
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void board_ui_start()
{
    xTaskCreatePinnedToCore(
        board_ui_task,
        "board_ui",
        6144,
        NULL,
        RTOS_UI_LEVEL,
        NULL,
        1);
}
