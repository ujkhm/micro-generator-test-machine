#pragma once

#include <Arduino.h>
#include "settings/settings.h"

// 安全電流識別(SAFE_CURRENT_ARCH.md)：
// 負載固定為 LOAD_TEST_RESISTOR_OHM，轉速由「本次開機自動調參後系統自然停下來的
// 轉速」(見 safe_current_reset() 讀 pid_get_keep_rpm())開始逐檔往上加，
// 每檔用當下轉速、電壓、電流換算 R_th，對冷態基準看銅溫升，找出可長期使用的安全電流 I_cont。
// 呼叫端(measure_seq)須確保呼叫本函式時已經在 MEAS_SAFE_CURRENT 階段；
// 本模組自己的第一步就是設定目標轉速並等待 speed_stable，不要求呼叫當下就穩調。
// 識別成功結束前會先 PWM=0 滑行、再拉回上一通過檔，避免滿 PWM 開路飛車後才進內阻。
void safe_current_reset();               // 重置回本模組最初狀態(全新開始)
void safe_current_rewind_current_rung(); // 夾子鬆脫續測：回到本檔 PREP，保留已通過檔
bool safe_current_step(uint32_t now_ms); // 執行一次狀態機步進；回傳 true=識別已結束(成功或硬故障)
