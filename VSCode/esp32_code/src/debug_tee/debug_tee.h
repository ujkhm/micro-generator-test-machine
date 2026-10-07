#pragma once

#include <Arduino.h>

// 把 Serial.print/printf/println 同時寫進 UART0（USB 線還能看）和一條固定佇列。
// 藍牙任務再把佇列裡的整行送出。不要在別的任務直接寫 SerialBT，否則會和 JSON 交錯。
class DebugTee : public Print
{
public:
    size_t write(uint8_t c) override;
    size_t write(const uint8_t *buffer, size_t size) override;
    void begin(unsigned long baud);
    int available();
    int read();
    int peek();
    void flush();
    explicit operator bool() const;
};

extern DebugTee DebugOut;

// 只應由 bt_telemetry 任務呼叫。每次最多送 max_lines 行，避免堵住即時 JSON。
void debug_log_drain(Print &out, int max_lines);

// 核心標頭把 Serial 定義成 Serial0。改成 DebugOut 之後，Serial.printf 會進佇列。
// 只替換完整 token「Serial」，不會碰到 SerialBT / Serial_debug_time。
#undef Serial
#define Serial DebugOut
