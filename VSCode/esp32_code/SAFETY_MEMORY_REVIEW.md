# 韌體記憶體與時間軸安全覆查報告

適用範圍：本次新增的 6 個模組(`measure_seq`、`safe_current`、`gen_resistance`、`curve_calc`、
`bt_telemetry`)＋對既有 `settings.h`／`motor_PID.cpp`／`speed_sensor.cpp`／`main.cpp`／
`board_ui.cpp`／`pins.h`／`RTOS.h` 的修改。

結論先講：**已用 PlatformIO 實際編譯成功**(見第 7 節數據)，**鎖的使用經逐一追蹤，全專案任何一條路徑都不會同時持有兩把鎖**(第 2 節)，**RPM 飛車／馬達通電看門狗／ESTOP 三道既有防線完全沒被修改或繞過**(第 4 節)，新增的「發電機斷線暫停」與「量測安全鎖定」兩個機制都是疊加在既有防線之上、不是取代。以下逐項說明依據，不是只有結論。

---

## 0. 修訂記錄

**2026-08-27　修正：定速目標曾被硬蓋成固定常數，導致調參完成後長時間震盪不收斂**

初版曾在 `motor_PID_init()` 裡加一行 `pid_set_keep_rpm(MEASURE_MIN_RPM)`，想讓每次開機的
量測起點是一個確定的數字。這個改動忽略了本機台的一個既有、在 `motor_PID.cpp` 開頭就
大量註解過的特性：**PWM→RPM 增益極不均勻**，sTune 自動調參是針對「開環階梯自然停下來
的那個轉速附近」做特性化，算出的 Kp/Ki 只在那個工作點附近有效。硬把 `keep_rpm` 蓋成
一個固定常數後，`begin_autotune()` 裡原本「`keep_rpm<=1` 時鎖存目前速度」的自我一致
邏輯被繞過(因為 `keep_rpm` 已經 >1)，於是調參完成、切入閉環的那一刻，系統被命令從
「剛特性化過的工作點」跳去「這個寫死的常數」，兩者可能差距不小，PID 用著在別的工作點
算出來的增益去追一個沒被特性化過的目標，在這種增益極不均勻的機台上就是實測會長時間
震盪不收斂的典型情境。

**修正方式**：拿掉那一行強制賦值，恢復原始「`keep_rpm` 鎖存自動調參當下的實際轉速」的
自我一致行為；`safe_current_reset()` 的第 0 檔目標也從固定常數改成讀取當下的
`pid_get_keep_rpm()`(新增 `safe_start_rpm` 欄位保存這個起點，供 `gen_resistance`／
`curve_calc` 之後取用，取代原本各自使用的 `MEASURE_MIN_RPM` 常數)。`MEASURE_MIN_RPM`
現在只當作事件訊息/報表備援下限的參考值，不再是任何地方拿去強制命令 PID 的目標。

第 3、4 節與第 8 節下方原本描述「定速在 `MEASURE_MIN_RPM`」的地方已一併更新用詞，但
為了不讓覆查記錄失真，沒有回頭改寫整份文件的敘事順序，只在對應段落加註更正。

**2026-08-27　新增：發電機防反接 SS54 二極體的電壓補償**

硬體上在發電機與 INA232 之間加了一顆 SS54 蕭特基二極體防反接。新增 `settings.h` 的
`ss54_compensate_voltage_V()`(純函式，無共享狀態、無鎖，不影響第 2 節的鎖分析)，
在 `safe_current.cpp`(開路 Voc)與 `gen_resistance.cpp`(開路 Voc、帶載 V)三個讀值點
補償二極體順向壓降。只動到「怎麼把 INA 讀數換算成發電機端子電壓」這一步，不影響任何
電流門檻判斷、狀態機轉移條件、鎖的使用方式；補償模型是近似值(校準範圍 0～0.6A，對應
本應用實際會用到的電流)，常數集中在 `settings.h` 開頭並附上如何用實測值校正的說明。

**2026-08-27　修正：`safe_current_reset()` 在 SAFE_CURRENT 真正開始前太早呼叫，導致
安全電流一開始就把轉速拉到 0、觸發無脈衝看門狗**

上一輪修正把「安全電流第 0 檔的起點」從固定常數改成讀取 `pid_get_keep_rpm()`，方向是對的，
但呼叫的**時機**錯了：`safe_current_reset()` 原本只在 `measure_seq_task()` 剛啟動時呼叫一次
(見第 3.1 節)，那個時間點開環爬升、自動調參都還沒開始，`keep_rpm` 當下多半是 0 或
NVS 殘留的舊值。這個(錯誤的)0 就被存進 `safe_start_rpm`，一路帶到 `MEAS_SAFE_CURRENT`
階段才被 `SAFE_PH_PREP` 拿去下令 `pid_set_keep_rpm(0)`──實測現象正是「進入安全電流測試
後轉速逐漸變慢、歸零、最後跳 `FAULT_NO_PULSE_TIMEOUT`」，是既有無脈衝看門狗正確攔下來的
(這道防線本身沒有問題，是量測序列自己下錯指令)。

**修正方式**：在 `measure_seq.cpp` 的 `case MEAS_MIN_SPEED_HOLD:` 分支裡，確認
`speed_get_speed_stable()` 為真的那一刻(這才是「自動調參後系統自然停下來的轉速」第一次
真正確立的時刻)才呼叫 `safe_current_reset()`，取代開機時那次過早的呼叫(該次呼叫繼續保留，
只是它捕捉到的值現在會被這裡蓋掉，不影響正確性)。斷線暫停後的續測路徑
(`handle_link_lost_state()`)本來就是在 `wait_for_resume_stable()` 成功後才呼叫
`safe_current_reset()`，時機一直是對的，不需要修改，現在兩條路徑的時機終於一致。

---

## 1. 記憶體分析

### 1.1 只用靜態配置，沒有動態記憶體

- 新增的 6 個模組**沒有任何一處呼叫 `malloc`／`new`／Arduino `String`**。字串一律用固定大小的 `char[]` 緩衝區 + `snprintf`/`vsnprintf`。
- `bt_telemetry.cpp` 的 JSON 組字串緩衝區(`live_buf[2048]`、`curve_buf[6144]`)刻意宣告成**檔案層級的 `static`**，不是函式內區域變數：
  - 若宣告成區域變數(放在任務堆疊上)，`curve_buf` 一個就要 6KB，而該任務堆疊只給 6144 bytes，會直接把整個堆疊吃光，甚至溢位到隔壁記憶體(FreeRTOS 堆疊溢位是韌體最常見的隨機當機／飛車成因之一)。
  - 改成 `static` 後，這兩塊緩衝區落在 `.bss` 區段，大小在**編譯期**就固定、生命週期等於整個程式執行期，不會有「用到才配置、配置失敗」的問題，也不會被任何其他任務誤用(因為是各自檔案內的 `static`，連結時對其他編譯單元不可見)。
  - `append()` 輔助函式每次呼叫都用「剩餘容量」夾住 `vsnprintf`，任何情況都不會寫出陣列邊界；一旦被截斷(`len` 會被夾到等於 `buf_size`)，後續 `append()` 呼叫會因為 `len>=buf_size` 直接跳過，不會用負數/溢位長度去算位置。
- `measure_settings` 結構體裡的陣列(`res_point_*[RES_MAX_POINTS]`、`curve_*[MAX_CURVE_POINTS]`)全部是結構體內的**固定大小陣列**，隨結構體一起以 `volatile measure_settings meas_settings{};` 的方式在 `measure_seq.cpp` 靜態配置，不是指標＋動態配置。所有存取函式(`meas_res_get/set_point`、`meas_curve_get/set_point`)都先檢查 `idx >= 上限` 才動作，越界存取會直接回傳 `false`、不會寫壞記憶體。

### 1.2 陣列容量與實際使用量

| 陣列 | 容量 | 單筆大小 | 總大小 | 實際會用到 |
|---|---|---|---|---|
| `res_point_rpm/rth/ke` | `RES_MAX_POINTS`=3 | 3×float=12B | 36B | 固定 3 點(低/中/高) |
| `curve_rpm/v_at_maxp/p_max/r_opt` | `MAX_CURVE_POINTS`=180 | 4×float=16B | 2880B | 依 `n_lim` 而定，通常遠小於 180 |

`MAX_CURVE_POINTS=180` 是刻意抓到「就算 `n_lim` 逼近 `RPM_RUNAWAY_MAX`(17000)、起點抓最低 1000 附近、每 100RPM 一點」都還有餘裕的容量((17000-1000)/100=160 < 180)，`curve_calc_run()` 裡的 `for` 迴圈本身也用 `count < MAX_CURVE_POINTS` 再上一道保險，兩層保護不會寫出陣列。

### 1.3 實際編譯結果(PlatformIO，`pio run`，2026-08-27)

```
RAM:   17.5% (used 57500 bytes from 327680 bytes)
Flash: 90.1% (used 1181311 bytes from 1310720 bytes)
```

- **RAM 非常寬裕**(剩餘約 270KB)，新增的所有共享結構＋緩衝區＋6 個任務堆疊(見 1.4)全部算進去只佔整體 17.5%，沒有 RAM 不足風險。
- **Flash 只剩約 129KB(9.9%)餘裕**，這是本次新增後才浮現的真實限制，主因是 `U8g2`(內建幾百種面板驅動，即使只用到 SSD1306/SH1106 兩種)與 `BluetoothSerial`(完整 Bluedroid classic 協定疊)本身就很佔空間。**目前這個容量還能正常燒錄執行**(已實測)，但若之後還要繼續加功能，建議：
  1. 把 `platformio.ini` 的 `board_build.partitions` 換成 `huge_app.csv` 或自訂分割表，把 OTA/次要 app 分割區讓給主程式(4MB flash 通常還能再擠出幾百 KB)。
  2. 或改用 `U8x8`(純文字模式，不含圖形/大量字型)取代 `U8g2`，可省下可觀的 Flash，但目前的圖形化 OLED 畫面需要小幅改寫。
  3 本次**沒有**做這兩項優化，因為目前仍在可燒錄範圍內、不屬於「會不會死機/飛車」的安全性問題，屬於之後功能擴充前才需要處理的空間規劃。

### 1.4 六個 FreeRTOS 任務的堆疊配置

| 任務 | 核心 | 優先權 | 堆疊 | 內部有無大型區域變數 |
|---|---|---|---|---|
| `motor_PID`(既有) | 0 | 16 | 6144 | 無(本次未改動大小) |
| `speed_sensor`(既有) | 0 | 15 | 4096 | 無(本次未改動大小) |
| `ina232`(既有) | 0 | 4 | 4096 | 無 |
| `board_ui`(既有) | 1 | 8 | 6144 | 無 |
| **`measure_seq`(新增)** | 1 | 3 | 4096 | 無(`safe_current_step`/`gen_resistance_step` 內都只有幾個 `float`/`bool` 區域變數) |
| **`bt_telemetry`(新增)** | 1 | 2 | 6144 | 無(2KB/6KB 緩衝區皆為 `static`，不占堆疊) |

新增兩個任務都放在 **Core 1**、優先權刻意低於 `board_ui`(8)、高於 Arduino `loop()`(預設 1)，不會搶佔 Core 0 上 `motor_PID`(16)／`speed_sensor`(15) 的即時控制路徑，符合既有的「量測/顯示類任務不干擾控制迴路」設計原則(`ina232` 當初也是同樣考量放到低優先權)。

---

## 2. 鎖與資料競爭分析

### 2.1 現有 6 把自旋鎖

| 鎖 | 保護的結構 | 唯一寫入者(任務) |
|---|---|---|
| `g_speed_mux`(既有) | `settings`(speed_sensor) | `speed_sensor` 任務為主，`motor_PID`/`main`/新模組的 `fail_hard()`/`speed_trigger_fault()` 在觸發故障時也會寫 |
| `g_pid_mux`(既有) | `PID_settings` | `motor_PID` 任務；`measure_seq`/`safe_current`/`gen_resistance` 只會呼叫 `pid_set_keep_rpm()` 這一個外部入口 |
| `g_ina_mux`(既有) | `ina_settings` | `ina232` 任務 |
| `g_ui_mux`(既有) | `ui_settings` | `main`(Arduino `loop()`) |
| **`g_meas_mux`(新增)** | `measure_settings` | **只有 `measure_seq` 任務**(內含呼叫 `safe_current_step()`/`gen_resistance_step()`/`curve_calc_run()`，這 3 個函式都是被 `measure_seq` 同一個任務「呼叫」執行，不是獨立任務) |
| **`g_bt_mux`(新增)** | `bt_telemetry_state` | **只有 `bt_telemetry` 任務** |

`fault` 相關欄位(`settings.fault`/`fault_code`/...)雖然本來就有 3 個既有寫入點(`motor_PID.cpp`、`speed_sensor.cpp` 各自的 `trip_fault()`，加上 `main.cpp` 的 `trigger_estop()`)，本次新增了第 4 個寫入點：`settings.h` 新增的共用函式 `speed_trigger_fault()`，供 `safe_current.cpp`/`gen_resistance.cpp` 的 `fail_hard()` 呼叫。**四個寫入點各自完整地用同一把 `g_speed_mux`、寫入同一組欄位、寫入內容彼此一致(都是「故障鎖定」這個事實的完整快照)**，不會因為「這次是誰觸發的」而出現欄位互相矛盾的中間狀態。這是刻意的設計：**新模組重用既有、已經過審視的鎖定機制，而不是自己發明一套平行的安全鎖定路徑**。

### 2.2 逐一確認：新增程式碼從未同時持有兩把鎖

這是本次覆查最重要的一項，因為「巢狀持有不同鎖」是死結(deadlock)最典型的成因。逐檔案追蹤如下(每個函式呼叫都是「進入→做完→返回」，不會把鎖跨函式呼叫帶著走)：

- **`measure_seq.cpp`**：
  - `enter_pause()`：`meas_set_resume_phase()`(鎖/解鎖) → `meas_set_pause_request()`(鎖/解鎖) → `load_switch_set()`(內部呼叫 `meas_set_load_connected()`，鎖/解鎖) → 一個獨立的 `SettingsLockGuard(g_meas_mux)` 區塊只寫 `link_lost`/`phase` 兩個欄位。全部依序執行，沒有任何一步在鎖還沒放開前又去呼叫別的加鎖函式。
  - `handle_link_lost_state()`：同樣模式，`wait_for_resume_stable()` 迴圈裡只呼叫 `speed_get_fault()`/`speed_get_speed_stable()`(各自獨立鎖 `g_speed_mux`)，且這段迴圈**完全沒有持有 `g_meas_mux`**(呼叫這個函式前後才各自獨立加解鎖)。
  - `measure_seq_task()` 主迴圈：`speed_get_fault()`、`meas_get_phase()`、`check_generator_link()`(內部只呼叫 `speed_get_speed_stable()`/`ina_get_online()`/`ina_get_data_valid()`/`ina_get_bus_V()`，全部獨立鎖)、以及呼叫 `safe_current_step()`/`gen_resistance_step()`/`curve_calc_run()`——這些呼叫「當下」都沒有持有 `g_meas_mux`。
- **`safe_current.cpp`**：每個 `case` 分支都是「呼叫若干個 `meas_get_x()`/`ina_get_x()`/`speed_get_x()`(各自獨立鎖) → 算一算 → 呼叫 `meas_set_x()`/`goto_phase()`(各自獨立鎖)」，`finish_with_result()`/`fail_hard()` 內部的 `SettingsLockGuard` 區塊都只包住「純欄位賦值」，不在區塊內呼叫任何其他加鎖函式。
- **`gen_resistance.cpp`**：與上面同一種模式，`finish_averaging()` 的迴圈(呼叫 `meas_res_get_point()`，各自獨立鎖)在迴圈外面、跟後面「寫入平均值」的 `SettingsLockGuard(g_meas_mux)` 區塊是兩個獨立、不重疊的臨界區。
- **`curve_calc.cpp`**：讀值(`meas_get_res_rth_ohm()` 等)、寫入中繼結果(一個 `SettingsLockGuard(g_meas_mux)` 區塊)、逐點計算迴圈裡呼叫 `meas_curve_set_point()`(各自獨立鎖)，同樣沒有巢狀。
- **`bt_telemetry.cpp`**：`build_live_message()`/`build_curve_message()` 呼叫的每一個 `xxx_get_y()` 都是獨立的一次鎖/解鎖，兩個函式之間也不會互相巢狀呼叫。

**結論：新增程式碼的鎖使用完全符合既有規範「同一時間只持有一顆鎖」，不存在巢狀持鎖，因此不可能因為鎖的順序不同而死結。**

### 2.3 多欄位一致性(避免「新A配舊B」)

凡是「好幾個欄位代表同一件事實、必須一起看才有意義」的地方，都用 `SettingsLockGuard` 包住整段直接寫欄位，而不是呼叫多次獨立的 `set_x()`(那樣中間會有其他任務讀到「一半新一半舊」的風險)。本次新增程式碼中這樣處理的地方：

- `safe_current.cpp::finish_with_result()`：`safe_i_cont_A`＋`safe_i_cont_rpm`＋`safe_done` 三個欄位一起寫。
- `safe_current.cpp` 判定通過時：`safe_pass_any`＋`safe_last_pass_rpm`＋`safe_last_pass_A` 三個欄位一起寫。
- `gen_resistance.cpp::gen_resistance_reset()`／`finish_averaging()`：重置/寫入結果時，相關欄位都在同一個鎖區塊內。
- `curve_calc.cpp`：`curve_n_rl`/`n_knee`/`n_voc`/`n_lim`/`curve_limit_reason` 五個欄位一起寫，讀取端(`bt_telemetry`／OLED)不會看到「`n_lim` 已更新但 `curve_limit_reason` 還是舊的」這種組合。
- `measure_seq.cpp::enter_pause()`：`link_lost`＋`phase` 一起寫。

---

## 3. 時間軸／狀態機分析

### 3.1 暫停(發電機斷線)→ 續測 的完整時序驗證

這是本次最容易出錯、也是我在實作前就先驗證過的一段(過程見對話紀錄)，這裡摘要成可檢核的時間軸：

1. `measure_seq` 偵測到 `bus_V` 連續 `GEN_LINK_LOST_TIMEOUT_MS` 低於門檻 → 呼叫 `enter_pause()`：`pause_request=true`、負載斷開、`phase=MEAS_LINK_LOST`。
2. **同一瞬間**，`motor_PID` 任務下一次被喚醒(最多等 `read_space`≈15ms)時，第一個 `if(speed_get_fault())` 為 false，接著新增的 `if(meas_get_pause_request())` 為 true → PWM 立刻歸零、`init_phase=SPEED_PHASE_PID_PAUSED`、**刻意不清 `const_speed_ready`／`autotune_done`**。
3. 馬達實際轉速歸零後，`speed_sensor` 既有的「真停轉」偵測(`SPEED_STALL_TIMEOUT_MS`=300ms 無新脈衝)會自然把 `const_speed_ready` 置 false──**這一步我特別修改了 `speed_sensor.cpp::update_phase_after_sample()`**，讓它同時認得 `SPEED_PHASE_PID_PAUSED`(比照原本就有特殊處理的 `PID_TUNE`/`PID_RUN`)，否則這個既有函式會在暫停期間每輪把 `init_phase` 搶回 `READY`、把 `const_speed_ready` 搶回 `true`，兩個任務對同一組欄位「互踩」，畫面會每 15ms 閃爍一次(已在實作中發現並修正，見 `speed_sensor.cpp` 的修改註解)。這個閃爍**不影響安全**(因為 `motor_PID` 的暫停判斷只看 `pause_request`，不看 `const_speed_ready`)，但會誤導顯示，所以已經修正。
4. 使用者排除斷線、按下 START：`main.cpp` 偵測到 `ui_state==UI_LINK_LOST` 時的按鈕邊緣 → 呼叫 `resume_after_link_loss()`：只做 `meas_set_resume_requested(true)` 與樂觀地把 `ui_state` 設回 `UI_RUNNING`。
5. `measure_seq` 任務下一輪迴圈看到 `resume_requested` → 清掉它、把 `pause_request` 設回 false、重置斷線監測的「已穩過一次」旗標(`reset_link_monitor()`，避免把「還沒重新穩定前的低電壓」誤判成又斷線一次)。
6. `motor_PID` 看到 `pause_request` 變 false，加上 `const_speed_ready` 這時多半也已經是 false(因為第 3 步的真停轉偵測)，於是走既有的「測速尚未完成」分支呼叫 `open_loop_step_control()`——**因為 `ol_hold` 與 `step_pwm`(本地靜態變數)全程沒被清除**，這個既有函式的第一個判斷式 `if (speed_get_ol_hold())` 會直接重新套用「上次已知可行」的 PWM，不必重新從最低階爬升。
7. 轉速回升、`speed_sensor` 量到新的同位置整圈資料 → 因為 `settle_samples_local` 也全程沒被歸零，`const_speed_ready` 幾乎立刻(下一筆有效樣本)重新變 true。
8. `motor_PID` 的 `!pid_get_autotune_done()` 為 false(因為 `autotune_done` 全程沒被暫停邏輯動過)，直接跳過整段自動調參，進入「PID 定速」，用暫停前就已經算好的 `Kp`/`Ki`/`Kd` 把轉速拉回 `keep_rpm`(這是本次開機自動調參後系統自然停下來的轉速，同樣全程沒被暫停邏輯動過──**不是**任何寫死的常數，見下方 2026-08-27 修訂記錄)。
9. `measure_seq::wait_for_resume_stable()` 等到 `speed_get_speed_stable()` 變 true(有逾時保護 `SAFE_SPEED_WAIT_TIMEOUT_MS`)，才呼叫「目前模組」的 `_reset()`(整段重來)並把 `phase` 設回該模組，重新開始量測。

**這段時序的關鍵安全性質**：從第 1 步到第 9 步，`speed_get_fault()`／既有的飛車保護／既有的無脈衝看門狗**全程持續有效**(它們檢查的是 `now_speed`/`ol_pwm_cmd` 等欄位，跟本次新增的暫停邏輯用的是完全不同的欄位與判斷路徑，兩者是「疊加」關係，不是「取代」關係)。就算暫停/恢復的邏輯本身有沒想到的漏洞，既有這兩道防線仍然會在轉速異常或馬達卡死時獨立動作。

### 3.2 量測序列三模組交接的一致性

`measure_seq` 是**唯一**呼叫 `safe_current_step()`/`gen_resistance_step()`/`curve_calc_run()` 的地方，而且是**同一個任務、依序**呼叫(不會同時呼叫兩個)，所以三個模組之間不需要額外的鎖來保護「交接」──`meas_settings.phase` 這一個欄位本身就是交接的協議：`measure_seq` 只有在確認 `safe_current_step()`/`gen_resistance_step()` 回傳 `true`**且 `speed_get_fault()` 仍為 false**(這是我在整合時額外加的守門判斷，見下方 3.3)才會把 `phase` 切到下一個模組；下一個模組的 `_reset()`／`_step()` 從呼叫的那一刻才開始執行，不會有「還在跑安全電流，內阻卻已經開始改變目標轉速」這種交叉執行的情況。

### 3.3 「硬故障但函式仍回傳 true」的收尾陷阱(已修正)

`safe_current_step()`/`gen_resistance_step()` 在呼叫 `fail_hard()`(觸發整機安全鎖定)的分支裡也回傳 `true`，語意是「本模組的狀態機已經跑完，不用再呼叫我了」──但 `measure_seq` 原本若只看「回傳 true 就進下一階段」，會在**硬故障當下**還誤把 `phase` 推進到 `MEAS_RESISTANCE`/`MEAS_CURVE_CALC`。這在實作時已經發現並修正：`measure_seq.cpp` 在兩個轉階段的地方都先檢查 `if (speed_get_fault()) { break; }` 才真正切換 `phase`。硬故障發生時，`phase` 會停在原地，下一輪迴圈最上方的 `if (speed_get_fault()) continue;` 會接手，整個任務之後只做「什麼都不做地空轉」，等待重開機。

### 3.4 內阻模組單點失效 vs. 全部失效

`gen_resistance.cpp::RES_PH_COMPUTE` 對算出來不合理的 `R_th`(負值/太大/太小/非有限數)採取「丟棄這一點、繼續下一點」而不是立刻整機鎖定，只有 `finish_averaging()` 發現**三點全部失效**時才會 `fail_hard()`。這是刻意設計：單一取樣點偶發雜訊(換向瞬間、INA 抖動)不應該讓已經跑了很久的安全電流+內阻識別直接報廢重來；但如果連一個有效點都拿不到，代表接線或方法本身有問題，不應該用「猜」的數字去算後面的曲線，因此才整機鎖定。

---

## 4. 既有安全機制保留確認

逐一核對本次修改是否影響三道既有防線：

| 既有機制 | 檢查點 | 是否被本次修改影響 |
|---|---|---|
| **RPM 飛車保護**(`RPM_RUNAWAY_MAX`) | `speed_sensor.cpp::runaway_protect()`、`motor_PID.cpp` 閉環段、sTune 調參段 | **完全未修改**這三處判斷式本身；新增的 `SAFE_RPM_MAX_CEILING`(預設 6000RPM)是量測序列**自己額外疊加**的、更保守的人為上限，不會、也不能讓系統跑到比既有 17000RPM 更高 |
| **馬達通電看門狗**(`NO_PULSE_TIMEOUT_MS`) | `speed_sensor.cpp::runaway_protect()` | **完全未修改**判斷式；本次新增的暫停邏輯會主動把 `ol_pwm_cmd` 設回 0，讓這個看門狗在暫停期間正確地「不判定為卡死」(這本來就是看門狗設計的本意：命令停止就不該被當成異常) |
| **ESTOP(板上 START 鈕)** | `main.cpp::trigger_estop()` | **完全未修改**；新增的 `UI_LINK_LOST` 是與 `UI_ESTOP` 平行的新狀態，兩者用不同旗標(`link_lost`/`pause_request` vs `fault`)驅動，互不干擾。使用者在 `UI_RUNNING` 按 START 仍然是 ESTOP，在 `UI_LINK_LOST` 按 START 才是「請求恢復」，語意上不會混淆(且恢復流程本身也會受既有 `speed_get_fault()` 檢查把關，見 3.1 第 9 步) |
| **開環起動階梯／sTune 自動調參** | `motor_PID.cpp` | 開環爬升與調參邏輯**逐行未動**；曾經加過一行 `pid_set_keep_rpm(MEASURE_MIN_RPM)` 想讓每次開機的定速目標是確定的常數，但這會讓系統在調參完成瞬間被命令跳去一個沒被特性化過的工作點，實測長時間震盪不收斂，已於 2026-08-27 移除，改回原始的「keep_rpm 鎖存自然停下來的轉速」邏輯，見下方修訂記錄 |

---

## 5. 新增的失效模式與因應方式

| 情境 | 偵測方式 | 因應 |
|---|---|---|
| 發電機鱷魚夾脫落(電壓歸零) | `bus_V` 連續 `GEN_LINK_LOST_TIMEOUT_MS` 低於 `GEN_LINK_LOST_V_MAX` | 非故障暫停(`UI_LINK_LOST`)，重按 START 整段重跑目前模組 |
| INA232 中途離線 | 各模組每個關鍵步驟都檢查 `ina_get_online() && ina_get_data_valid()` | 視同硬故障，`fail_hard()` 整機鎖定(量測期間沒有電流回饋等於沒有安全網，不可以繼續帶載) |
| 負載開關卡死(斷開時仍量到電流) | 開路取樣步驟檢查 `oc_a > SAFE_OC_MAX_CURRENT_A` | `fail_hard()` 整機鎖定 |
| 帶載後電流過小(接觸不良/沒接上) | 檢查 `< SAFE_I_MIN_VALID_A` / `RES_MIN_VALID_A` | `fail_hard()` 整機鎖定 |
| 任何時刻電流超過硬天花板 | 每個等待迴圈都即時檢查 `SAFE_I_HARD_CEILING_A`，不是只在取樣瞬間檢查 | 立即斷負載＋`fail_hard()` |
| 安全電流第一檔就不通過 | `safe_pass_any==false` 時判定不通過 | 無法給出任何安全電流，`fail_hard()`(不會硬套用一個不安全的數字) |
| 內阻三點全部無效 | `finish_averaging()` 的 `counted==0` | `fail_hard()`(不會用無效數字算曲線) |
| 曲線計算輸入無效(理論上不會發生) | `curve_calc_run()` 開頭檢查 `rth>0 && ke>0` | 寫入空結果(`curve_point_count=0`)並印訊息，防禦性寫法，不會用垃圾數字硬算 |
| 等待穩調逾時(轉速一直拉不上去) | `SAFE_SPEED_WAIT_TIMEOUT_MS`/`RES_SPEED_WAIT_TIMEOUT_MS` | `fail_hard()` |
| 安全電流階梯撞到 `SAFE_RPM_MAX_CEILING` | 已通過至少一檔 → 用該檔收尾；一檔都沒過 → `fail_hard()`(代表天花板設太低，需要調常數) |

---

## 6. 已知限制／需要依實際機台調整的常數

以下常數已在 `settings.h` 內逐一加註「★依實際機台調整」，這裡集中列出，實際上機前務必檢視：

- `SAFE_I_HARD_CEILING_A`(預設 1.2A，須低於 INA232 1.6A 量程；已有通過檔時碰到上限改為斷負載、用上一檔進內阻，只有第一檔就超過才鎖定)
- `SAFE_RPM_STEP`(預設 300RPM)、`SAFE_RPM_MAX_CEILING`(預設 6000RPM，務必遠低於 `RPM_RUNAWAY_MAX`=17000)
- `SAFE_COOLDOWN_MS`(預設 30 秒，依實際散熱調整)
- `GEN_LINK_LOST_V_MAX`(預設 0.15V，須遠小於最低轉速下的開路電壓)
- `CURVE_V_ALLOW`(預設 40V，須低於 INA232 與後級真正的耐壓)
- `LOAD_TEST_RESISTOR_OHM`(預設 3Ω，三顆 1Ω 串聯，需與實際外接電阻一致；改了必須同步更新，程式本身不會自動偵測外接電阻的實際阻值)

另外兩個**設計上的簡化**，不是缺陷，但要知道：

- 本次**沒有**把安全電流/內阻/曲線的結果存進 NVS(Flash)。每次重開機都會整段重新量測。這是因為使用情境是「每次都重新裝上一顆待測發電機」，不是「量一次、之後開機沿用」；若之後需要跨開機沿用，`ARCH.md` 文件裡有預留這個彈性設定的位置，但目前程式碼還沒實作。
- 內阻模組的三個轉速點(高/中/低)是**由安全電流階梯的結果反推**(`I_cont` 對應的最高通過檔、與起始轉速的中點、起始轉速)，**量測順序為高→中→低**(避免安全電流結束後立刻大幅降速逾時)；如果之後想要指定別的轉速點策略，要改 `gen_resistance.cpp::target_rpm_for_point()`。

---

## 7. 建構驗證紀錄

```
$ pio run
...
Compiling .pio/build/mhetesp32devkit/src/measure_seq/measure_seq.cpp.o
Compiling .pio/build/mhetesp32devkit/src/safe_current/safe_current.cpp.o
Compiling .pio/build/mhetesp32devkit/src/gen_resistance/gen_resistance.cpp.o
Compiling .pio/build/mhetesp32devkit/src/curve_calc/curve_calc.cpp.o
Compiling .pio/build/mhetesp32devkit/src/bt_telemetry/bt_telemetry.cpp.o
...
RAM:   [==        ]  17.5% (used 57500 bytes from 327680 bytes)
Flash: [========= ]  90.1% (used 1181311 bytes from 1310720 bytes)
========================= [SUCCESS] Took 35.58 seconds =========================
```

唯一的編譯警告是 `BluetoothSerial` 本身標了 `[[deprecated]]`(官方預告未來版本可能預設拿掉 classic BT 支援)，不影響本次功能，純粹是上游函式庫的長期规划提示。

---

## 8. 建議的上機測試順序(對應各 ARCH.md 文件的「建議實作順序」，這裡是整合後的版本)

1. **先不接發電機負載**，只確認 `SERVO_PIN` 的電位方向對(高電位量到通/斷是否符合 `LOAD_SWITCH_ACTIVE_HIGH` 的預期)。
2. 確認開環爬升＋自動調參本身能正常收斂到穩定閉環(這部分邏輯完全沿用既有、已經跑得動的開環+調參+閉環，未被本次改動觸碰)，且 `speed_stable` 會在合理時間內變 true。
3. 把 `SAFE_THERMAL_SOAK_MAX_MS`／`SAFE_THERMAL_CHECK_WINDOW_MS` 暫時改成幾十秒(乾跑)，確認整個 `MIN_SPEED_HOLD → SAFE_CURRENT → RESISTANCE → CURVE_CALC → DONE` 的階段真的會依序推進，序列埠(`[MEAS]`/`[SAFE_I]`/`[RES]`/`[CURVE]` 開頭的訊息)可以逐行對照。
4. 用真實電阻接上，跑一次完整(分鐘級)識別，核對 `[SAFE_I]`/`[RES]` 印出的每檔/每點數字是否合理。
5. 手動把鱷魚夾拔掉，確認 3 秒後進入 `UI_LINK_LOST`(OLED 顯示變化、序列埠印 `[MEAS] generator link lost...`)，接回並按 START，確認會重新爬升並整段重跑「當時正在跑的那個模組」。
6. 最後才連藍牙，用序列埠工具(或先用手機藍牙序列 App)確認每 200ms 收到一行 `{"t":"live",...}`，`curve_done` 之後每 3 秒收到一行 `{"t":"curve",...}`，再進上位機端的開發。
