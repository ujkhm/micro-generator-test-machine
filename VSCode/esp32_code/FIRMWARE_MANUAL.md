# ESP32 Micro-Generator 韌體說明書（完整版）

> 適用路徑：`VSCode/esp32_code/`  
> 平台：PlatformIO + Arduino-ESP32（底層 ESP-IDF 5.5.x）  
> 目的：讀完本文件應能完整理解「系統在做什麼、各模組如何協作、保護如何觸發、參數要改哪裡」。

---

## 0. 這台機器在幹嘛？

本專案是以 **ESP32** 為核心的 **微型發電機 / MPPT 量測平台**：

1. 用有刷直流馬達當 **主動力馬達**（PWM 驅動，目標是把轉速穩在某 RPM）。
2. 另一側（或同一系統）量測發電側電氣參數（INA232：電壓／電流／功率）。
3. 先把主動力馬達 **定速**，再去做功率曲線／MPPT 相關量測（MPPT 算法本身仍在發展中）。

韌體目前已完成的核心是：

- 光柵測速（MCPWM Capture）
- 階梯開環起動 → 就緒觀察 → sTune 自動調 PID → 閉環定速
- 飛車／無脈衝看門狗／急停等保護
- OLED 顯示 + START 鈕起動／急停
- INA232 電氣量測

---

## 1. 倉庫與檔案架構

```
micro generator/
├── README.md                          # 專案總覽（硬體/結構）
├── KiCad/                             # 原理圖、PCB
├── BOM/
├── 3D_print_Component/                # 光柵等 3D 件
├── Host computer/                     # 上位機（讀每秒一行 RPM）
└── VSCode/esp32_code/                 # ★韌體（本文件焦點）
    ├── platformio.ini
    └── src/
        ├── main.cpp                   # 開機、START、Serial 除錯、協調急停畫面
        ├── settings/settings.h        # ★所有常數、狀態結構、NVS 欄位、記憶體保護介面
        ├── settings/settings.cpp      # ★4 顆自旋鎖(portMUX_TYPE)的實體
        ├── pins/pins.h                # GPIO 對照
        ├── RTOS/RTOS.h                # 任務優先權／核心
        ├── soft_i2c/soft_i2c.h        # 軟體 I2C（OLED／INA）
        ├── speed_sensor/              # 測速模組
        ├── motor_PID/                 # 開環起動 + sTune + PID 定速
        ├── board_ui/                  # OLED 顯示
        └── ina232/                    # 電壓電流功率
    ```

**改參數的第一入口永遠是：`src/settings/settings.h`。**  
腳位改 `pins/pins.h`，任務優先權改 `RTOS/RTOS.h`。  
**讀寫任何共享狀態一律透過 `settings.h` 提供的 `xxx_get_yyy()`/`xxx_set_yyy()` 介面**（詳見第 6 章），不要直接碰 `settings`/`PID_settings`/`ina_settings`/`ui_settings` 的欄位。

---

## 2. 硬體接線（與程式對應）

| 功能 | GPIO | 定義 | 備註 |
|------|------|------|------|
| 光柵測速 | 36 | `SPEED_SENSOR_PIN` | MCPWM Capture，雙沿 |
| 馬達 PWM | 26 | `MOTOR_PWM_PIN` | LEDC，20 kHz，10-bit（0~1023） |
| START 鈕 | 15 | `START_PIN` | 對 GND；內部上拉；按下=LOW |
| OLED SDA/SCL | 33/25 | `SDA_PIN`/`SCL_PIN` | 軟體 I2C；對應 PCB `/SDA2`/`/SCL2` |
| INA232 SDA/SCL | 21/22 | `SDA2_PIN`/`SCL2_PIN` | 軟體 I2C；板端可能無外接上拉 |
| Servo / Buck | 23 / 14 | 已定義，目前韌體未驅動 | 預留給後續 MPPT／致動 |

INA232 預設位址 `0x40`（A0→GND），分流電阻 `0.05Ω`（滿量程約 1.64A）。

---

## 3. 執行環境與依賴

`platformio.ini`：

- board：`mhetesp32devkit`
- framework：`arduino`（pioarduino，底層 IDF ~5.5.4）
- monitor：`115200`
- 函式庫：
  - `QuickPID`：閉環 PID
  - `sTune`：開環自動調參
  - `U8g2`：OLED

---

## 4. RTOS 任務地圖

| 任務 | 核心 | 優先權 | 誰啟動 | 職責 |
|------|------|--------|--------|------|
| `motor_PID` | Core 0 | 16 | START 後 `motor_PID_start()` | 開環階梯、sTune、PID、PWM 輸出 |
| `speed_sensor` | Core 0 | 15 | START 後 `speed_sensor_start()` | Capture ISR、轉速計算、無脈衝看門狗、喚醒 PID |
| `ina232` | Core 0 | 4 | `setup()` 即啟動 | 週期讀 INA232 + EMA |
| `board_ui` | Core 1 | 8 | `setup()` 即啟動 | OLED 刷新（純顯示） |
| Arduino `loop()` | Core 1 | 1（預設） | 框架 | START 鈕、Serial DBG、急停協調 |

重點：

- **按 START 之前不會轉馬達**（測速／PID 任務尚未建立）。
- `speed_sensor` 每個 `read_space`（預設 15 ms）對 PID 做 `xTaskNotifyGive`。
- `loop()` 必須 `vTaskDelay(1)`，否則會餓死 IDLE、觸發 Task WDT、造成卡頓與按鍵失靈。

---

## 5. 整機生命週期（從上電到定速）

```
上電
  ├─ setup(): Serial / START / OLED / INA232 / UI 任務
  └─ UI = WAIT_START（螢幕：Press START）

按 START（第一次）
  ├─ motor_PID_start() + speed_sensor_start()
  └─ UI = RUNNING

馬達控制狀態機（settings.init_phase）
  STEP_UP  → 每次升高一階 PWM，等 settle
  PROBE    → settle 後抓「之後第一筆」同位置轉速
  LEARNING → 轉速夠了就固定 PWM，觀察 N 圈就緒
  READY    → const_speed_ready=true
  PID_TUNE → sTune 自動調參（可跳過）
  PID_RUN  → QuickPID 閉環；可能置位 speed_stable=true

再按 START（RUNNING 中）
  └─ ESTOP 鎖定（fault=FAULT_ESTOP），需重開機
```

任一保護觸發 → `FAULT`，PWM=0，需重開機。

---

## 6. 共享狀態（其他模組該讀什麼）

都在 `settings.h`。**這 4 個結構體都被多個 FreeRTOS 任務、甚至 ISR 同時存取，
一律不要直接讀寫欄位，改用下面的 thread-safe 介面。**

### 6.0 執行緒安全存取介面（新模組必讀）

**為什麼需要它**：`volatile` 只保證編譯器不會把值快取在暫存器裡，並不能保證
「一次改好幾個相關欄位」不會被其他任務／ISR 插隊到一半，也不保證「同時讀好幾個
欄位」彼此是同一個時間點的值。因此 `settings.h` 用 4 顆獨立的自旋鎖
(`portMUX_TYPE`，實體在 `settings.cpp`) + 一套產生好的 get/set 函式來取代直接
存取：

| 鎖 | 保護的結構 | 保護的全域變數 |
|----|-----------|---------------|
| `g_speed_mux` | `speed_sensor` | `settings` |
| `g_pid_mux` | `motor_PID` | `PID_settings` |
| `g_ina_mux` | `ina232_sensor` | `ina_settings` |
| `g_ui_mux` | `board_ui` | `ui_settings` |

分成 4 顆而非共用 1 顆，是讓互不相關的模組不會互相卡住（例如 ina232 更新電壓
電流時，不該讓 speed_sensor 的 ISR 或 motor_PID 的控制迴圈等待）。

**API 命名規則**（由 `settings.h` 內的巨集自動產生，每個欄位都有）：

```cpp
speed_get_now_speed()          / speed_set_now_speed(v)      // speed_sensor 欄位
pid_get_keep_rpm()             / pid_set_keep_rpm(v)          // motor_PID 欄位
ina_get_bus_V()                / ina_set_bus_V(v)             // ina232_sensor 欄位
ui_get_state()                 / ui_set_state(v)              // board_ui 欄位
```

即「前綴(struct 簡稱) + `_get_`/`_set_` + 欄位名稱」，對照 `settings.h` 裡
`SETTINGS_SCALAR_ACCESSOR(...)` / `SETTINGS_SCALAR_GETTER(...)`（唯讀）那幾行
就能查到每個欄位實際有哪些函式可用。測速環形緩衝這種好幾個欄位綁在一起才有意義
的資料，改走專用函式：`speed_capture_push_edge()`（只給 ISR 寫入用）、
`speed_capture_snapshot()`（任務端整組讀出）、`speed_capture_mark_used()`。

**需要一次讀寫多個相關欄位、要求彼此一致時**，用 `SettingsLockGuard`（RAII，
離開作用域自動解鎖）直接包住整段：

```cpp
{
    SettingsLockGuard lock(g_speed_mux);   // 依你要動的結構選對應的鎖
    settings.fault = true;
    settings.fault_code = code;
    settings.init_phase = SPEED_PHASE_FAULT;
} // 離開這裡自動解鎖
```

**使用規則（務必遵守）**：

1. 鎖內只能放「幾行欄位讀寫」，**嚴禁**在鎖內呼叫 `Preferences`(NVS/Flash)、
   `Serial`、`delay()` 等耗時或會被中斷的函式（鎖住期間會關閉本核心中斷）。
   需要存 Flash 時，先在鎖內把值複製到區域變數，解鎖後再對區域變數做 I/O
   （寫法參考 `settings.h` 內 `motor_PID::load()` / `::save()`）。
2. **不同的鎖不要巢狀持有**（例如持有 `g_pid_mux` 時又去呼叫會拿 `g_speed_mux`
   的函式）。目前全專案都遵守「同一時間只持有一顆鎖」，避免兩顆鎖交叉鎖死。
   自己結構內部的欄位可以巢狀存取同一顆鎖沒關係（ESP-IDF 自旋鎖對同核心可重入）。
3. 誰是這個結構的「擁有模組」（`speed_sensor.cpp` 擁有 `settings`、
   `motor_PID.cpp` 擁有 `PID_settings`…）可以用 `SettingsLockGuard` 直接讀寫
   自己結構的多個欄位；**跨模組**一律只用 get/set 函式，不要直接碰欄位。

**新增模組照抄這個模式即可**：宣告自己的 `struct` + 一顆 `portMUX_TYPE`
（在 `settings.cpp` 定義實體），欄位一行一個 `SETTINGS_SCALAR_ACCESSOR(...)`，
就能自動獲得同一等級的保護，不必重新設計鎖的邏輯。

### 6.1 `settings`（`speed_sensor` 結構）— 測速與協調核心

| 欄位 | 意義 | 存取函式 | 誰寫 | 誰讀 |
|------|------|---------|------|------|
| `now_speed` | 目前轉速 RPM（EMA 後） | `speed_get/set_now_speed` | speed_sensor | 全員 |
| `speed_valid` | 轉速是否可信 | `speed_get/set_speed_valid` | speed_sensor | PID／UI |
| `read_space` | 測速／PID 週期 ms（預設 15） | `speed_get/set_read_space` | 預設／可改 | 兩模組 |
| `init_phase` | 階段碼 | `speed_get/set_init_phase` | 兩邊 | UI／DBG |
| `const_speed_ready` | 測速就緒，可進調參／定速 | `speed_get/set_const_speed_ready` | speed_sensor | motor_PID |
| `rpm_stable` | **內部**：開環已 hold（不是對外穩調） | `speed_get/set_rpm_stable` | 兩邊 | DBG |
| **`speed_stable`** | **對外穩調旗標**（其他模組請用這個） | `speed_get/set_speed_stable` | motor_PID 為主 | 任何模組 |
| `ol_pwm_cmd` | 目前 PWM 指令（開環或閉環） | `speed_get/set_ol_pwm_cmd` | motor_PID | 看門狗／DBG |
| `ol_hold` | 固定開環 PWM 中 | `speed_get/set_ol_hold` | motor_PID | speed_sensor |
| `ol_probe_request/ready/rpm` | 階梯 probe 協議 | `speed_get/set_ol_probe_*` | 兩邊 | DBG |
| `settle_samples` | LEARNING 已觀察圈數 | `speed_get/set_settle_samples` | speed_sensor | DBG |
| `fault` / `fault_code` | 故障鎖定 | `speed_get/set_fault[_code]` | 任一保護路徑 | 全員 |
| `edge_count` | 絕對邊緣數 | `speed_get_edge_count`（唯讀） | ISR | DBG |
| `buf_idx`/`count_number` | 環形緩衝指標／快照 | `speed_capture_*()` 專用函式 | ISR／speed_sensor | DBG |

### 6.2 `PID_settings`（`motor_PID` 結構）

| 欄位 | 意義 | 存取函式 |
|------|------|---------|
| `keep_rpm` | 定速目標 RPM（NVS 可存；0 時調參前會鎖當前轉速） | `pid_get/set_keep_rpm` |
| `Kp/Ki/Kd` | PID 增益（NVS） | `pid_get/set_Kp`／`Ki`／`Kd` |
| `tuned` | NVS：是否調過 | `pid_get/set_tuned` |
| `autotune_active/done` | 本次上電調參狀態 | `pid_get/set_autotune_active`／`_done` |
| `pwm_freq=20000`、`pwm_res=10` | LEDC 參數 | `pid_get/set_pwm_freq`／`pwm_res` |

NVS namespace：`pid_cfg`（`Kp`,`Ki`,`Kd`,`tuned`,`keep_rpm`）。  
`load()`/`save()` 內部已處理好鎖與 Flash I/O 的順序，直接呼叫 `PID_settings.load()` /
`PID_settings.save()` 即可，不需額外加鎖。

### 6.3 `ina_settings`

| 欄位 | 存取函式 |
|------|---------|
| `bus_V`／`current_A`／`power_W`／`shunt_mV` | `ina_get/set_bus_V` 等 |
| `online`／`data_valid` | `ina_get/set_online`／`data_valid` |
| `sample_count` | `ina_get_sample_count`（唯讀）；遞增用 `ina_increment_sample_count()` |

### 6.4 `ui_settings`

| 欄位 | 存取函式 |
|------|---------|
| `state`：`WAIT_START` / `RUNNING` / `ESTOP` | `ui_get/set_state` |
| `motor_started` | `ui_get/set_motor_started` |
| `oled_ok` | `ui_get/set_oled_ok` |

---

## 7. 測速模組（`speed_sensor`）詳解

### 7.1 硬體原理

- MCPWM Capture，時鐘 **80 MHz**
- GPIO36，**上升沿 + 下降沿**
- 光柵格數：`space_number = 2` → 每圈邊緣數 `EDGES_PER_REV = 4`
- 環形緩衝：`2^buf_idx_bit = 16` 筆時間戳

公式：

\[
\text{RPM} = \frac{80\times10^6 \times 60}{\text{同一物理位置相隔一整圈的 ticks}}
\]

程式常數：`RPM_CONSTANT = 80e6 * 60`。

### 7.2 為什麼堅持「同位置整圈」？

3D 列印光柵每格寬度有誤差。若用「相鄰格」算瞬時轉速，誤差會很大。  
用 **同一個遮光/透光位置，隔一整圈** 的時間差，幾何誤差會互相抵銷。

早期曾做「格距誤差補償 + 單格量測」以提高更新率，但實測：

- 單格時間基準只有 1/4 圈，雜訊被放大
- 本機台 PWM→RPM 增益極高（約每 1 count 十幾 RPM）
- 雜訊進 PID 會造成 0↔滿載硬切

**現行策略：控制迴路一律只用同位置整圈 + 輕量 EMA。**  
`LEARNING` 不再學格距權重，只做「固定 PWM 後觀察夠多圈」。

### 7.3 ISR 去彈跳

若兩次邊緣間隔對應轉速 > `RPM_RUNAWAY_MAX × MIN_EDGE_TICKS_SAFETY_RPM_MULT`（預設 2.5 倍），視為彈跳，**丟棄不入緩衝**。

### 7.4 低轉速「probe 補丁」

開環每升一階 PWM 後：

1. 等 `MOTOR_STEP_SETTLE_MS`（250 ms）
2. `ol_probe_request=true`
3. 只接受 **請求之後** 的第一筆新同位置轉速 → `ol_probe_rpm`
4. 不拿「保持舊值」冒充當前轉速（避免低轉速誤判）

門檻：`probe_rpm >= RPM_INIT_READABLE + RPM_INIT_MARGIN`（1000+150=**1150 RPM**）  
達標 → `ol_hold=true`，進入 LEARNING。

### 7.5 LEARNING / READY

固定 PWM 後，每來一個有效同位置樣本：`settle_samples++`  
達 `READY_SETTLE_MIN_SAMPLES`（24）→ `const_speed_ready=true`。

### 7.6 EMA 濾波

```
speed_filter_ema += SPEED_FILTER_ALPHA * (rpm_same - speed_filter_ema)
```

預設 `SPEED_FILTER_ALPHA = 0.3`。  
`now_speed` 輸出的是濾波後值（給 keep_rpm 鎖定與 PID）。

### 7.7 無新邊緣時的行為

| 情況 | 行為 |
|------|------|
| probe 等待中 | `speed_valid=false`，不拿舊值決策 |
| 距上邊緣 ≤ 300 ms | 保持上次轉速 |
| > 300 ms | 視為停轉，`now_speed=0`；若尚未就緒則重置觀察進度 |

---

## 8. 馬達控制模組（`motor_PID`）詳解

### 8.1 PWM

- LEDC attach：`MOTOR_PWM_PIN`，20 kHz，10-bit → **0~1023**
- 開環上限：`MOTOR_STARTUP_PWM_MAX_RATIO = 0.70` → 最大約 716
- 每階：`MOTOR_PWM_STEP_RATIO = 0.05` → 約 51 count／階

### 8.2 階梯開環流程

```
pwm += step
等 250ms
請求 probe
  ├─ 逾時 400ms 無資料 → 當 0 RPM → 再升階
  ├─ < 1150 → 再升階
  ├─ 已到 70% 仍不夠 → FAULT_PWM_MAX_NO_SPEED
  └─ ≥ 1150 → hold，進 LEARNING
```

### 8.3 sTune 自動調參（測速就緒後）

**啟用條件**

- `PID_AUTOTUNE_ENABLE=1`
- `PID_AUTOTUNE_EVERY_BOOT=1` → 每次上電都調  
  （改 `0` 則只在 NVS `tuned=false` 時調）

**為何不用早期的 ZN_PID + directIP？**

實測問題：

- 反曲點法容易被超調／震盪騙到過大製程增益
- Kd 在 15 ms 取樣下被放大，造成 bang-bang
- sTune 的 Kp 是「正規化增益」，直接餵 QuickPID 會再放大約 `inputSpan/outputSpan` 倍

**現行策略**

| 項目 | 現行 |
|------|------|
| 動作 | `direct5T`（完整 5τ 測試） |
| 規則 | `NoOvershoot_PI`（**只用 PI，Kd=0**） |
| 步階 | 目前 duty + 8% max（`PID_TUNE_STEP_RATIO`） |
| settle | 3 s |
| test 上限 | 25 s |
| pre-settle | 就緒後先等 500 ms 再鎖 `keep_rpm` |
| inputSpan | `max(2000, now_speed×4)`，且 ≤ runaway |
| eStop | `RPM_RUNAWAY_MAX × 0.75` |
| 增益換算 | 用真實製程增益重算 Kp/Ki，再 × `PID_TUNE_GAIN_SCALE(0.8)` |
| Tau/td 上限 | 20（防死時間量太小暴衝） |
| 總逾時 | settle+test+10s |

調參完成 → 存 NVS → `PID_RUN`。

### 8.4 閉環 PID

- `QuickPID`，anti-windup：`iAwClamp`
- 取樣：`read_space × 1000` µs
- **輸出斜率限制**：每週期最多變 `PID_OUTPUT_SLEW_MAX`（60 count）  
  避免 0↔1023 硬切。
- 目標：`PID_settings.keep_rpm`

### 8.5 `speed_stable`（對外穩調旗標）

**只有全部成立才可能變 true：**

1. 無 fault  
2. `const_speed_ready`  
3. `autotune_done` 且非調參中  
4. `init_phase == PID_RUN`  
5. `speed_valid` 且 `now_speed ≥ 1000`  
6. `keep_rpm > 1`  
7. 連續 `SPEED_STABLE_NEED_HITS`(20) 次貼近目標  
   （|err|≤40 RPM **或** 相對誤差≤2.5%）

**立刻變 false：**

- 換 `keep_rpm`
- 偏離目標
- 初始化／調參／故障／停轉／不可信

其他模組（例如未來 MPPT）應讀：

```cpp
if (settings.speed_stable) { /* 可以開始依賴定速的工作 */ }
```

不要誤用 `rpm_stable`（那只是開環 hold 內部旗標）。

---

## 9. 保護程序總表

| 代號 | 名稱 | 觸發條件 | 動作 |
|------|------|----------|------|
| 1 | `FAULT_RUNAWAY_RPM` | `now_speed > RPM_RUNAWAY_MAX`（現行 **17000**） | PWM=0，鎖定 |
| 2 | `FAULT_NO_PULSE_TIMEOUT` | **只要 `ol_pwm_cmd>0`**，連續 `NO_PULSE_TIMEOUT_MS`(200ms) 無新邊緣 | 同上（通電看門狗，全階段） |
| 3 | `FAULT_PWM_MAX_NO_SPEED` | 開環到 70% 仍达不到 1150 RPM | 同上 |
| 4 | `FAULT_AUTOTUNE_FAIL` | 增益無效／製程參數無效／調參總逾時 | 同上 |
| 5 | `FAULT_ESTOP` | RUNNING 時再按 START | 同上 + UI_ESTOP |

共同後果：

- `fault=true`，`init_phase=FAULT`
- `const_speed_ready=false`，`speed_stable=false`
- PWM 強制 0
- **需重開機**才能恢復（刻意設計）

診斷用（不鎖機）：

- `SPEED_JUMP_WARN_RPM`：定速中單週期跳變過大 → `[EVT] speed jump ...`
- `SPEED_EDGE_GAP_WARN_MS`：定速中邊緣間隔過長 → `[EVT] edge gap warn ...`

---

## 10. UI / START / INA232

### 10.1 START（`main.cpp`）

- `INPUT_PULLUP` + IDF pull-up  
- ISR 僅設旗標；`loop` 做 debounce（40 ms）  
- `WAIT_START` + 按下邊緣 → 啟動馬達任務  
- `RUNNING` + 按下邊緣 → ESTOP  
- 其他模組 fault 時，UI 自動切 `ESTOP` 畫面

### 10.2 OLED（`board_ui`）

- 軟體 I2C，可強制／自動 SSD1306 或 SH1106（`OLED_CONTROLLER_FORCE`）  
- 顯示：狀態、RPM、PWM、V/I/P、fault  
- **不負責按鍵**（避免軟 I2C 卡住漏按）

### 10.3 INA232

- 軟體 I2C @ 100 kHz（無外接上拉時必須慢）  
- 晶片 AVG=1；ESP32 做 EMA（`INA232_FILTER_ALPHA=0.2`）  
- 任務週期 1 ms，優先權低，不搶控制迴路

### 10.4 SoftI2C 為何自己寫？

Arduino-ESP32 3.x 硬體 I2C 在 NACK 後可能卡 `INVALID_STATE`。  
且熱路徑若每 bit 呼叫 `pinMode()` 會極慢（OLED 一幀可卡數百 ms）。  
現行：開汲極 + 內部上拉，熱路徑只動 `gpio_set_level`。

---

## 11. Serial 除錯協定

鮑率 **115200**。

| 前綴 | 內容 |
|------|------|
| `[BOOT]` | 上電參數摘要 |
| `[MAIN]` | START／ESTOP |
| `[EVT]` | 階段／旗標／故障／跳變警告（事件） |
| `[DBG]` | 每 250 ms 狀態總覽 |
| `[TUNE]` | 調參進度／結果 |
| `[SPD]` | 看門狗等 |
| 純數字行 | 每秒一行 `now_speed`（上位機相容；非數字行會被上位機忽略） |

`[DBG]` 重要欄位解讀：

- `phase`：狀態機  
- `rpm` / `keep`：實際／目標  
- `pwm`：目前 duty  
- `settle=a/b`：就緒觀察進度  
- `ready`：`const_speed_ready`  
- `speed_stable`：對外穩調  
- `tune=active/done`  
- `V/I/P`：INA232  

---

## 12. 預設值速查（現行 `settings.h`）

### 測速／起動

| 常數 | 預設 | 意義 |
|------|------|------|
| `space_number` | 2 | 光柵格數 |
| `buf_idx_bit` | 4 | 緩衝深度 16 |
| `read_space` | 15 ms | 測速／PID 週期 |
| `RPM_INIT_READABLE` | 1000 | 可讀門檻 |
| `RPM_INIT_MARGIN` | 150 | → 實際要 ≥1150 |
| `MOTOR_PWM_STEP_RATIO` | 0.05 | 每階 5% |
| `MOTOR_STEP_SETTLE_MS` | 250 | 升階後等待 |
| `MOTOR_PROBE_TIMEOUT_MS` | 400 | probe 逾時 |
| `MOTOR_STARTUP_PWM_MAX_RATIO` | 0.70 | 開環上限 |
| `READY_SETTLE_MIN_SAMPLES` | 24 | LEARNING 圈數 |
| `SPEED_STALL_TIMEOUT_MS` | 300 | 無邊緣當停轉 |
| `SPEED_FILTER_ALPHA` | 0.3 | RPM EMA |
| `MIN_EDGE_TICKS_SAFETY_RPM_MULT` | 2.5 | ISR 去彈跳 |

### 保護

| 常數 | 預設 |
|------|------|
| `RPM_RUNAWAY_MAX` | 17000 |
| `NO_PULSE_TIMEOUT_MS` | 200 |

### PID／調參

| 常數 | 預設 |
|------|------|
| `PID_AUTOTUNE_ENABLE` | 1 |
| `PID_AUTOTUNE_EVERY_BOOT` | 1 |
| `PID_TUNE_SETTLE_SEC` | 3 |
| `PID_TUNE_TEST_SEC` | 25 |
| `PID_TUNE_SAMPLES` | 250 |
| `PID_TUNE_STEP_RATIO` | 0.08 |
| `PID_TUNE_GAIN_SCALE` | 0.8 |
| `PID_TUNE_PRE_SETTLE_MS` | 500 |
| `PID_OUTPUT_SLEW_MAX` | 60 |
| `SPEED_STABLE_*` | 40 RPM / 2.5% / 20 hits |

### UI／INA

| 常數 | 預設 |
|------|------|
| `START_DEBOUNCE_MS` | 40 |
| `UI_REFRESH_MS` | 200 |
| `I2C_INA_FREQ_HZ` | 100000 |
| `INA232_RSHUNT_OHM` | 0.05 |
| `INA232_IMAX_A` | 1.6 |

---

## 13. 我要怎麼改？（實務指南）

### 13.1 光柵格數改了

改 `space_number`。  
雙沿時 `EDGES_PER_REV` 會自動變。確認機械上每圈邊緣數正確。

### 13.2 馬達太肉／太兇（開環爬不上去或太快）

- 爬不上去：加大 `MOTOR_PWM_STEP_RATIO` 或 `MOTOR_STARTUP_PWM_MAX_RATIO`  
- 太兇：減小步階、加長 `MOTOR_STEP_SETTLE_MS`  
- 1150 門檻不適合：改 `RPM_INIT_READABLE` / `RPM_INIT_MARGIN`

### 13.3 無脈衝看門狗誤觸／太慢

改 `NO_PULSE_TIMEOUT_MS`：

- 誤觸（啟動瞬間就 FAULT）→ 加大（如 300~500）  
- 希望更快斷電 → 减小（但勿小於正常最大脈衝間隔）

### 13.4 飛車太敏感／太鬆

改 `RPM_RUNAWAY_MAX`。  
調參期 eStop 是它的 75%（`PID_TUNE_ESTOP_RATIO`）。

### 13.5 定速震盪（pwm 狂切、RPM 晃）

優先檢查：

1. `PID_OUTPUT_SLEW_MAX` 是否夠小  
2. 是否又打開了 D 項（現行應為 PI，`Kd=0`）  
3. `SPEED_FILTER_ALPHA` 略降可更平滑  
4. `PID_TUNE_GAIN_SCALE` 再降（如 0.5）  
5. 設 `PID_AUTOTUNE_EVERY_BOOT 0`，用已存 NVS 參數對照  
6. 手動寫死 Kp/Ki（改 `load()` 預設或 NVS）

### 13.6 不想每次上電都自動調參

```cpp
#define PID_AUTOTUNE_EVERY_BOOT 0
```

或

```cpp
#define PID_AUTOTUNE_ENABLE 0
```

### 13.7 固定目標轉速

- 調參前把 `PID_settings.keep_rpm` 設成目標（或 NVS 寫入）  
- 若保持 0，程式會在 pre-settle 後鎖「當前濾波轉速」當目標

### 13.8 換 OLED 全黑

試：

```cpp
#define OLED_CONTROLLER_FORCE 2  // SH1106
```

### 13.9 新增「定速後才做事」的模組

```cpp
#include "settings/settings.h"

if (!speed_get_fault() && speed_get_speed_stable()) {
    // 安全：轉速已穩在 keep_rpm
}
```

不要直接讀 `settings.fault`／`settings.speed_stable` 欄位，一律用
`speed_get_fault()`／`speed_get_speed_stable()`（見第 6.0 節的執行緒安全介面）。
若你的新模組要維護自己的共享狀態，照抄 `settings.h` 的模式：宣告
`struct` + 一顆 `portMUX_TYPE`（實體放 `settings.cpp`）+ `SETTINGS_SCALAR_ACCESSOR(...)`
逐欄位產生 get/set，就能自動獲得同一等級的保護。

### 13.10 清除 NVS 舊 PID

序列埠監控無法直接清時，可暫時在 `setup` 呼叫清除，或改 Preferences namespace，或用 esptool erase。  
欄位在 `pid_cfg`。

---

## 14. 模組協作時序（簡圖）

```
ISR(光柵邊緣) ──寫 cap_buffer──►
                                 │
speed_sensor 每 15ms：快照→算 RPM→保護→notify PID
                                 │
motor_PID：等待 notify
  ├─ 未就緒：階梯開環 / hold
  ├─ 就緒且未調完：sTune
  └─ 就緒且已調完：PID + slew + 更新 speed_stable
                                 │
main loop：START / ESTOP / Serial / 1ms yield
board_ui：畫 OLED
ina232：讀電氣量
```

---

## 15. 已知設計取捨（讀 code 時別踩雷）

1. **故障後不自動復歸** — 安全取向，必須重開機。  
2. **`rpm_stable` ≠ `speed_stable`** — 前者開環內部，後者對外。  
3. **控制不用單格補償** — 精度用同位置整圈換穩定。  
4. **sTune 原始 Kp 不能直接餵 QuickPID** — 必須做單位換算（已在 `tunings` 分支做）。  
5. **START 不放在 UI 任務** — 避免軟 I2C 阻塞漏按。  
6. **PCB 絲印 SDA/SCL 與直覺對調** — 以 `pins.h` 註解為準。  
7. **上位機只吃純數字 RPM 行** — DBG 行請保留非純數字前綴。  
8. **4 顆自旋鎖、刻意不互相巢狀** — `g_speed_mux`/`g_pid_mux`/`g_ina_mux`/`g_ui_mux`
   各自獨立，任何函式都不會在持有一顆的情況下又去拿另一顆，避免交叉鎖死；
   跨模組讀寫共享欄位一律走 `settings.h` 的 get/set 介面（見第 6.0 節）。

---

## 16. 建議閱讀／修改順序

若你要動手改：

1. `settings/settings.h` — 調行為／參數；新增共享欄位也在這裡加 struct 欄位 +
   對應的 `SETTINGS_SCALAR_ACCESSOR(...)`（見第 6.0 節）  
2. `settings/settings.cpp` — 只有新增「一整組結構」才需要動（多加一顆
   `portMUX_TYPE` 實體）  
3. `pins/pins.h` — 換腳  
4. `motor_PID/motor_PID.cpp` — 開環／調參／閉環  
5. `speed_sensor/speed_sensor.cpp` — 測速／看門狗  
6. `main.cpp` — START／除錯輸出  
7. `board_ui` / `ina232` — 顯示與電氣量測  

除錯時先看 Serial 的：

`phase` → `probe_rpm` → `settle` → `ready` → `tune` → `Kp/Ki/Kd` → `pwm` 是否仍 bang-bang → `speed_stable` → `fault`

---

## 17. 一句話總結

這份韌體做的是：**按 START → 階梯把馬達拉到可測轉速 → 同位置整圈測速就緒 →（可選）sTune 算出保守 PI → 斜率限制的閉環定速 → 用 `speed_stable` 告訴其他模組「可以開始依賴定速」；過程中任何飛車、通電無脈衝、調參失敗或再按 START 都會鎖定斷電。**

參數幾乎都在 `settings.h`；行為邏輯在 `speed_sensor` 與 `motor_PID`；人機在 `main` + `board_ui`；電氣在 `ina232`。
