#pragma once

#include <Arduino.h>
#include "settings/settings.h"
#include <QuickPID.h>
#include "pins/pins.h"
#include "RTOS/RTOS.h"
#include "esp32-hal.h"

// 馬達 PWM / 定速 PID：
// 階梯開環 → 測速就緒 → (可選)sTune 自動調參 → 閉環定速；與測速模組共用失控保護
// 閉環期間：負載通斷與換轉速檔會短暫提高 PI 增益與輸出斜率，其餘時間維持調參值
void motor_PID_start();

// 帶載定速中要斷開測試負載時呼叫。PID 任務會在同一個控制週期裡先把 PWM 拉回
// 「接通負載之前」記住的空載值，再斷開負載，避免高轉時帶載 duty 還在、負載已開而爆衝。
// 拉回成功後 pid_unload_holding() 為 true，直到重新 speed_stable，或目標轉速明顯提高。
void pid_request_unload();
bool pid_unload_holding();