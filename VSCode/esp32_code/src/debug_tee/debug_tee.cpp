#include "debug_tee.h"

#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

DebugTee DebugOut;

static constexpr int LOG_SLOTS = 24;
static constexpr int LOG_LEN = 480;
static constexpr int LOG_TASKS = 12;

struct LogAcc
{
    TaskHandle_t task;
    uint16_t len;
    char buf[LOG_LEN];
};

static LogAcc g_acc[LOG_TASKS];
static char g_slots[LOG_SLOTS][LOG_LEN];
static uint16_t g_head = 0;
static uint16_t g_tail = 0;
static portMUX_TYPE g_log_mux = portMUX_INITIALIZER_UNLOCKED;

static void push_line(const char *text, uint16_t len)
{
    if (text == nullptr || len == 0)
    {
        return;
    }
    if (len >= LOG_LEN)
    {
        len = LOG_LEN - 1;
    }
    portENTER_CRITICAL(&g_log_mux);
    const uint16_t next = (uint16_t)((g_head + 1) % LOG_SLOTS);
    if (next == g_tail)
    {
        g_tail = (uint16_t)((g_tail + 1) % LOG_SLOTS);
    }
    memcpy(g_slots[g_head], text, len);
    g_slots[g_head][len] = '\0';
    g_head = next;
    portEXIT_CRITICAL(&g_log_mux);
}

static LogAcc *acc_for_this_task()
{
    const TaskHandle_t self = xTaskGetCurrentTaskHandle();
    portENTER_CRITICAL(&g_log_mux);
    LogAcc *free_slot = nullptr;
    for (int i = 0; i < LOG_TASKS; ++i)
    {
        if (g_acc[i].task == self)
        {
            portEXIT_CRITICAL(&g_log_mux);
            return &g_acc[i];
        }
        if (g_acc[i].task == nullptr && free_slot == nullptr)
        {
            free_slot = &g_acc[i];
        }
    }
    if (free_slot != nullptr)
    {
        free_slot->task = self;
        free_slot->len = 0;
        free_slot->buf[0] = '\0';
    }
    portEXIT_CRITICAL(&g_log_mux);
    return free_slot;
}

static void feed_log(const uint8_t *buffer, size_t size)
{
    LogAcc *acc = acc_for_this_task();
    if (acc == nullptr)
    {
        return;
    }
    for (size_t i = 0; i < size; ++i)
    {
        const char c = (char)buffer[i];
        if (c == '\r')
        {
            continue;
        }
        if (c == '\n')
        {
            push_line(acc->buf, acc->len);
            acc->len = 0;
            acc->buf[0] = '\0';
            continue;
        }
        if (acc->len >= LOG_LEN - 1)
        {
            push_line(acc->buf, acc->len);
            acc->len = 0;
        }
        acc->buf[acc->len++] = c;
        acc->buf[acc->len] = '\0';
    }
}

size_t DebugTee::write(uint8_t c)
{
    return write(&c, 1);
}

size_t DebugTee::write(const uint8_t *buffer, size_t size)
{
    if (buffer == nullptr || size == 0)
    {
        return 0;
    }
    const size_t n = Serial0.write(buffer, size);
    feed_log(buffer, n);
    return n;
}

void DebugTee::begin(unsigned long baud)
{
    Serial0.begin(baud);
}

int DebugTee::available()
{
    return Serial0.available();
}

int DebugTee::read()
{
    return Serial0.read();
}

int DebugTee::peek()
{
    return Serial0.peek();
}

void DebugTee::flush()
{
    Serial0.flush();
}

DebugTee::operator bool() const
{
    return (bool)Serial0;
}

void debug_log_drain(Print &out, int max_lines)
{
    if (max_lines <= 0)
    {
        return;
    }
    char line[LOG_LEN];
    for (int n = 0; n < max_lines; ++n)
    {
        portENTER_CRITICAL(&g_log_mux);
        if (g_tail == g_head)
        {
            portEXIT_CRITICAL(&g_log_mux);
            return;
        }
        memcpy(line, g_slots[g_tail], LOG_LEN);
        g_tail = (uint16_t)((g_tail + 1) % LOG_SLOTS);
        portEXIT_CRITICAL(&g_log_mux);
        line[LOG_LEN - 1] = '\0';
        if (line[0] == '\0')
        {
            continue;
        }
        out.print(line);
        out.print('\n');
    }
}
