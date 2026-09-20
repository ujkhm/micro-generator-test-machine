#pragma once
#include <Arduino.h>
#include <Preferences.h>       // 用於將變數存入FLASH
#include "freertos/FreeRTOS.h" // portMUX_TYPE / 臨界區(自旋鎖)API，記憶體保護介面用
// 常數設定

// speed_sensor.h
#define buf_idx_bit 4                    // 環形緩衝深度位元數 → 2^4 = 16 筆時間戳
#define space_number 2                   // 用於測速之光柵盤總格數(遮光/透光格對數，依實際光柵修改)
#define EDGES_PER_REV (space_number * 2) // 雙沿觸發時，每一整圈的邊緣數

// 定速就緒門檻：控制迴路實際使用的轉速一律採用「同位置整圈」量測(rpm_from_same_slot)，
// 本身就用同一物理光柵位置抵銷每格加工誤差，不需要額外的格距誤差學習/補償；
// 這裡只保留一個簡單的樣本數門檻，確保開環固定 PWM 後至少觀察幾圈、EMA 濾波值
// 已收斂，才允許離開 LEARNING 進入 READY / 定速
#define READY_SETTLE_MIN_SAMPLES 24 // 開環固定 PWM 後，至少觀察幾圈才允許進入 READY / 定速
#define SPEED_STALL_TIMEOUT_MS 300  // 超過此時間無新邊緣 → 視為停轉/無法可靠量測

// 診斷用(不影響控制邏輯)：PID_RUN 定速期間偵測異常事件並印一次 [EVT]，方便日後排查
// 轉速忽然跳變/收斂變慢等現象時，快速判斷是感測端訊號問題還是馬達真實物理擾動
#define SPEED_JUMP_WARN_RPM 120    // 單一測速週期(read_space)轉速變化超過此值視為異常，印警告
#define SPEED_EDGE_GAP_WARN_MS 100 // 定速中連續無新邊緣超過此時間(仍未達 stall 判定)視為異常，印警告

// 感測端離群值抑制：高轉速時光柵偶發假邊緣/漏跳會讓「同位置整圈」瞬間算出離譜轉速，
// 若直接餵進 EMA→PID，會出現「轉速突然飆高再慢慢回落」的假性抽搐(馬達負載未變)。
// 下列閾值只丟棄明顯不合理的單筆樣本，不影響真實的漸進加速/減速。
#define SPEED_OUTLIER_MAX_ABS_RPM 180.0f   // 單筆 rpm_same 與目前 EMA 差超過此值(RPM)
#define SPEED_OUTLIER_MAX_RATIO 0.14f        // 或超過目前 EMA 的此比例(取兩者較寬鬆者)
#define SPEED_OUTLIER_REJECT_MAX 3           // 連續拒絕幾筆後強制接受(避免真實大階躍被永遠擋住)

// 邊緣去彈跳(debounce)：光柵訊號偶爾會有機械彈跳/雜訊造成極短間隔的假觸發，
// 對「整圈平均」影響很小(分母大)，但會讓「單格補償」算出離譜的瞬時轉速(分母只有 1/4 圈)。
// 兩次觸發間隔對應轉速若超過 RPM_RUNAWAY_MAX×此倍數，視為雜訊直接在 ISR 丟棄、不計入。
#define MIN_EDGE_TICKS_SAFETY_RPM_MULT 2.5f

// 「同位置整圈」量測法可正常讀取的最低轉速門檻(對應舊版約 1000RPM 失靈現象) + 餘量
#define RPM_INIT_READABLE 1000.0f
#define RPM_INIT_MARGIN 150.0f

// 階梯開環起動：升一次 PWM → 等待穩定 → 擷取第一筆可用轉速
#define MOTOR_PWM_STEP_RATIO 0.05f        // 每次升高的 duty 佔最大輸出比例
#define MOTOR_STEP_SETTLE_MS 250          // 每次升 PWM 後等待穩定的時間(ms)
#define MOTOR_PROBE_TIMEOUT_MS 400        // settle 後等待第一筆可用轉速的逾時(ms)，逾時視為 0 並再升階
#define MOTOR_STARTUP_PWM_MAX_RATIO 0.70f // 開環起動允許的最大 duty 比例

// 失控保護
#define RPM_RUNAWAY_MAX 17000.0f // 轉速超過此值視為飛車，立即切斷輸出

// ★馬達通電看門狗(單一閾值，涵蓋馬達通電全過程，無空窗)：
// 只要 ol_pwm_cmd>0(馬達有 PWM 輸出——從剛按下 START 的第一階開環升速，
// 到就緒觀察、自動調參、定速運行，全程都算)，且連續此時間內完全沒有
// 新脈衝進來，立即緊急停止鎖定(需重開機)。
// 從馬達一啟動就開始計時、全程持續生效，不會有「還沒等到某個階段」或
// 「已經動過一次就永久失效」之類的空窗——任何時刻卡死(卡死/斷軸/感測器
// 故障/接線脫落等)都會在此時限內被抓到。
// 測試負載通斷是階躍擾動：轉速會瞬間掉/衝，光柵脈衝間隔會暫時拉長，
// 但不代表卡死。門檻需大於「負載階躍後 PID 把轉速拉回可讀區」的最長無脈衝空窗，
// 又遠短於真的卡死。請依實測「按下 START 到第一次脈衝」與「接負載後最大脈衝間隔」
// 抓安全餘量；數值愈小保護愈即時，但也愈容易在負載階躍時誤判。
#define NO_PULSE_TIMEOUT_MS 800

// PID 自動調參(sTune)：速度感測器調教完成(const_speed_ready)後執行
// 注意：本機台 PWM→RPM 增益極大(小小的佔空比變化就能造成數千 RPM 變化)，
// 快速的『反曲點法(IP)』很容易被步階響應中的震盪/超調騙到過大的製程增益，
// 進而算出過度激進、會在 0↔滿載間來回硬切換的 PID 參數，故改用：
//   1) 完整 5T 測試(direct5T)取代反曲點法(directIP)，較不怕響應有震盪
//   2) NoOvershoot_PI 規則(只用 P+I，不含 D)取代傳統 Ziegler-Nichols(較不激進)：
//      QuickPID 內部把 Kd 換算成每輪增益是「Kd / 取樣秒數」，而定速閉環取樣週期
//      (read_space)僅 15ms，若含 D 項會被放大達數百倍，量測轉速的正常雜訊就足以
//      讓輸出在 0↔滿載間反覆硬切換、永遠無法收斂；改用純 PI 規則從根源避開此問題
//      (此製程 Tau/DeadTime 比值高，sTune 自評「easy to control」，PI 已足夠)
//   3) 較小的測試步階(貼近實際工作點附近的線性區)
//   4) 依目前轉速動態估計 inputSpan，而非直接套用飛車保護的絕對上限
//   5) 額外乘上安全係數(PID_TUNE_GAIN_SCALE)做保守化
//   6) ★把 sTune 的無因次增益換算回真實工程單位★
//      sTune 內部的製程增益是 (ΔPV/inputSpan)/(ΔCO/outputSpan)，是無因次的，
//      它推導出的 Kp 也是「正規化增益」；但 QuickPID 吃的是真實單位(RPM→PWM count)，
//      兩者差 inputSpan/outputSpan 倍。本機台 inputSpan≈5000RPM、outputSpan=1023count，
//      比值高達約 4.9，直接套用會讓 Kp 放大近 5 倍而逼近臨界增益，必定劇烈震盪。
//      (sTune 官方範例的 inputSpan/outputSpan 多半數量級相近、比值≈1，所以不會踩到)
//      實際換算與重算規則見 motor_PID.cpp 的 sTune::tunings 分支
#define PID_AUTOTUNE_ENABLE 1           // 1=啟用自動調參
#define PID_AUTOTUNE_EVERY_BOOT 1       // 1=每次上電都調；0=僅 NVS 尚未 tuned 時調一次
#define PID_TUNE_SETTLE_SEC 3           // 調參前在 outputStart 穩定秒數
#define PID_TUNE_TEST_SEC 25            // 開環步階測試時間上限(秒)；5T 法需約 5×Tau，寧可抓寬鬆
#define PID_TUNE_SAMPLES 250            // 測試取樣點數(建議 200~500)
#define PID_TUNE_STEP_RATIO 0.08f       // 相對目前開環 duty 再升的比例(佔 max PWM)；愈小愈接近目標點附近的線性區
#define PID_TUNE_INPUT_SPAN_MIN 2000.0f // sTune 輸入(RPM)全幅估計下限，實際值於執行期依目前轉速動態估算
#define PID_TUNE_ESTOP_RATIO 0.75f      // 調參期間緊急停止門檻 = RPM_RUNAWAY_MAX × 此比例(較保守)
#define PID_TUNE_PRE_SETTLE_MS 500      // 測速就緒後，先等待此時間讓濾波轉速真正收斂，再鎖定 keep_rpm / 開始測試
#define PID_TUNE_GAIN_SCALE 0.8f        // 對算出的 Kp/Ki/Kd 額外乘上的安全係數(<1 更保守)
// NoOvershoot_PI 的閉環迴路增益 = Kp×製程增益 = 0.35×(Tau/死時間)，只由可控性比值決定。
// 萬一某次量到異常小的死時間(雜訊或取樣抖動)會讓迴路增益暴衝而震盪，故對此比值設上限；
// 20 對應迴路增益 7，對 Tau/td 已屬「很好控制」的系統而言仍相當保守
#define PID_TUNE_TAU_DEAD_RATIO_MAX 20.0f
#define PID_TUNE_TOTAL_TIMEOUT_MARGIN_SEC 10 // 整段調參(settle+test)之外再加的總時長保險絲，逾時視為失敗並斷電
#define PID_TUNE_SERIAL_VERBOSE 1            // 1=調參時印進度/sTune 摘要；0=僅關鍵事件
#define PID_TUNE_SERIAL_MS 500               // 調參進度列印間隔(ms)

// 轉速量測濾波與輸出斜率限制：
// 「同位置整圈」量測本身雜訊已經很低，仍加一層輕量濾波避免殘餘尖峰污染
// keep_rpm 鎖定與 PID 輸入；定速輸出也需限制斜率，避免每個控制週期都在
// 0↔滿載間硬切換造成機械衝擊
#define SPEED_FILTER_ALPHA 0.22f // 轉速量測的 EMA 濾波係數(愈小愈平滑、愈慢反應)
#define PID_OUTPUT_SLEW_MAX 60  // 定速閉環每次控制週期(read_space)PWM 最大變化量(count，滿載為 1023)
// 負載通斷 / 換轉速檔時的短暫增益排程(只在這兩類擾動期間生效，穩態維持調參值)：
// 調參得到的 NoOvershoot_PI 偏保守，負載階躍或 +300RPM 換檔時 PWM 被斜率限制慢慢爬。
// 暫態把 Kp/Ki 與斜率上限一起加大；連續貼近目標或逾時立刻退回原值。
// 誤差輔助必須連續成立才進、連續貼近才退：單筆光柵離群／EMA 晃過門檻不得把增益×2、slew
// 放到接近滿載，否則會抽搐（實測 PWM 單步 170～234、暫態 ON/OFF 每秒來回）。
#define PID_TRANSIENT_GAIN_SCALE 2.0f // 相對目前調參 Kp/Ki 的倍率(>1 才加速；Ti 維持不變)
#define PID_TRANSIENT_SLEW_MAX 90     // 暫態期間每週期 PWM 最大變化(count)；負載接通另有 PID_LOAD_ON_PWM_BUMP
#define PID_TRANSIENT_MAX_MS 8000     // 暫態最長時間；逾時強制退回原增益，避免一直用高增益震盪
#define PID_TRANSIENT_EXIT_ERR_RPM 50.0f // |實際-目標| 低於此才開始計「可退回」(比 speed_stable 更嚴)
#define PID_TRANSIENT_EXIT_NEED_HITS 12  // 連續貼近幾次才退回原增益(約 read_space×N ms)，避免晃過一次就 OFF
#define PID_ASSIST_ERR_RPM 90.0f         // 誤差再大過此值才開始計「誤差輔助」(帶載後半段爬升過慢)
#define PID_ASSIST_NEED_HITS 10          // 連續超差幾次才進暫態：單筆離群／EMA 抖動不觸發
#define PID_LOAD_ON_PWM_BUMP 90          // 負載接通當下先加的 PWM count，抵消發電機扭矩階躍

// 轉速穩調旗標(speed_stable)判定：僅在整機正常閉環且貼近目標時才可能為 true
#define SPEED_STABLE_ABS_EPS 40.0f             // |實際轉速-目標| 低於此值的基底(RPM)
#define SPEED_STABLE_ABS_EPS_PER_1000RPM 28.0f // 隨目標轉速放寬：高轉時感測/PID 殘差較大
#define SPEED_STABLE_REL_EPS 0.035f            // 或相對誤差低於此比例視為貼近(高轉略放寬)
#define SPEED_STABLE_NEED_HITS 20              // 連續滿足幾次(約 read_space×N ms)才置 true
#define SPEED_STABLE_MIN_RPM RPM_INIT_READABLE // 低於此轉速不宣告穩調(尚非可穩定讀取區)

// 測速 / 起動階段(供除錯與模組協調)
enum speed_init_phase : uint8_t
{
    SPEED_PHASE_STEP_UP = 0,    // 階梯升 PWM，等待 settle
    SPEED_PHASE_PROBE = 1,      // settle 結束，等待第一筆可用同位置轉速
    SPEED_PHASE_LEARNING = 2,   // 已達可讀門檻，固定當前 PWM 做就緒觀察(穩定確認)
    SPEED_PHASE_READY = 3,      // 測速初始化完成
    SPEED_PHASE_FAULT = 4,      // 失控保護觸發，輸出切斷
    SPEED_PHASE_PID_TUNE = 5,   // 測速完成後，正在自動調 PID
    SPEED_PHASE_PID_RUN = 6,    // PID 定速運行中
    SPEED_PHASE_PID_PAUSED = 7, // ★量測序列請求暫停(發電機斷線)：PWM=0，非故障，重按 START 可續測
};

// 故障碼(settings.fault_code)
enum speed_fault_code : uint8_t
{
    FAULT_NONE = 0,
    FAULT_RUNAWAY_RPM = 1,      // 轉速超過 RPM_RUNAWAY_MAX
    FAULT_NO_PULSE_TIMEOUT = 2, // 馬達通電看門狗：連續 NO_PULSE_TIMEOUT_MS 無新脈衝(全程涵蓋)
    FAULT_PWM_MAX_NO_SPEED = 3, // 開環已達上限仍無法達到可讀轉速
    FAULT_AUTOTUNE_FAIL = 4,    // PID 自動調參失敗/中止
    FAULT_ESTOP = 5,            // 板載 START 鈕緊急停止(僅重開機可恢復)
    FAULT_MEASURE_SAFETY = 6,   // 量測序列(安全電流/內阻)偵測到硬安全門檻被突破
};

// ---- I2C 匯流排頻率(Hz) ----
// OLED 為模組(通常自帶上拉)；INA232 板端漏裝上拉，韌體需開 ESP32 內部上拉並可調時鐘
#define I2C_OLED_FREQ_HZ 400000 // OLED(Wire / I2C0)：模組自帶上拉，可用 Fast-mode
#define I2C_INA_FREQ_HZ 100000  // INA232(Wire1 / I2C1)：板端無外接上拉，必須用 100k；有外接 4.7k 後可改 400000

// ---- INA232 電壓/電流感測 ----
// 原理圖 R8=0.1Ω(2512)；ADCRANGE=0(±81.92mV) → 理論最大約 0.819A，故 Imax 取 0.8A 留餘量
// 晶片 AVG=16。帶載時 V 仍像被測試電阻拉住而 I≈0：標 current_plausible=false，
// 不把這筆寫進 EMA、也不准安全電流／內阻拿去算結果。不可用上一筆電流假裝還在測。
#define INA232_I2C_ADDR 0x40     // INA232A + A0→GND；若 A0 接 VS/SDA/SCL 請改 0x41/0x42/0x43
#define INA232_RSHUNT_OHM 0.1f   // 分流電阻(Ω)，對應原理圖 R8
#define INA232_IMAX_A 0.8f       // 預期最大電流(A)，用於計算 Current_LSB / Calibration
#define INA232_ADCRANGE_80MV 1   // 1=±81.92mV(ADCRANGE=0)；0=±20.48mV(ADCRANGE=1，CAL 需 /4)
#define INA232_AVG_CODE 2        // CONFIG AVG 欄位：0=1、1=4、2=16、3=64（datasheet Table 7-4）
#define INA232_FILTER_ALPHA 0.2f // ESP32 端 EMA 係數(愈小愈平滑、愈慢反應)
#define INA232_TASK_PERIOD_MS 1  // 讀取任務週期；1ms + 最短轉換時間 ≈ 盡可能快
#define INA_I_VS_V_MIN_RATIO 0.40f // 帶載時 |I| 低於 (V/Rload)*此比例 → 這一筆電流不能當測試數據
#define INA_I_INCONSISTENT_PAUSE_MS 500 // 帶載 V/I 連續對不上超過此時長：暫停本檔，不把 0A 寫進 I_cont

// ---- 板載互動介面(OLED + START) ----
#define OLED_I2C_ADDR 0x3C // 0.96" SSD1306 常見位址
#define UI_REFRESH_MS 200  // OLED 刷新週期(ms)
// ★很多副廠「0.96" SSD1306」模組實際上是 SH1106 控制器：
// 兩者初始化指令大部分共通，即使晶片其實是 SH1106，用 SSD1306 驅動送指令
// 對方仍會正常 ACK，u8g2.begin() 因此回傳成功，但畫面完全不會顯示
// (定址方式不同)。若換新模組後出現「有找到位址、begin 成功、但螢幕全黑」，
// 把下面改成 2 強制用 SH1106 測試看看；改回 1 可強制 SSD1306。
// 0 = 自動(先試 SSD1306，失敗才試 SH1106；SSD1306 誤判 ACK 時無法偵測到)
#define OLED_CONTROLLER_FORCE 0 // 0=自動 1=強制SSD1306 2=強制SH1106
#define START_DEBOUNCE_MS 40    // START 鈕去彈跳時間(ms)
#define START_ACTIVE_LOW 1      // 1=按下為 LOW(INPUT_PULLUP)；0=按下為 HIGH

// 互動介面狀態(供除錯與模組協調)
enum ui_app_state : uint8_t
{
    UI_WAIT_START = 0, // 開機等待按下 START 才開始測試
    UI_RUNNING = 1,    // 已啟動馬達控制，螢幕顯示即時資訊
    UI_ESTOP = 2,      // 緊急停止鎖定，僅重開機可恢復
    UI_LINK_LOST = 3,  // 夾子／量測線鬆脫暫停：非故障，排除後重按 START 從中斷處續測
};

// =====================================================================================
// 記憶體保護 / 執行緒安全存取介面 (Settings Access Layer)
// =====================================================================================
// 背景：下面 speed_sensor / motor_PID / ina232_sensor / board_ui 這幾個共享結構，
// 會被多個 FreeRTOS 任務(motor_PID、speed_sensor、ina232、board_ui、Arduino loop())
// 以及一個中斷服務常式(MCPWM Capture ISR)同時存取。單純宣告 volatile 只保證「編譯器
// 不會把它快取在暫存器裡、每次都乖乖重新讀寫記憶體」，並不能保證：
//   1. 「一次更新好幾個彼此相關欄位」的操作不會被其他任務/中斷插隊到一半
//      (例如 ISR 正在把新邊緣寫進環形緩衝，任務端卻同時在做快照，兩邊各拿到一半新一半舊)
//   2. 讀到的多個欄位彼此是「同一個時間點」的值(例如同時讀 now_speed 與 speed_valid，
//      有可能讀到「新 now_speed 配舊 speed_valid」這種實際上沒發生過的組合狀態)
// 因此這裡提供一套「自旋鎖(spinlock) + 存取函式」的介面，取代直接讀寫欄位：
//   - 每個共享結構各自使用一把 portMUX_TYPE 自旋鎖，模組之間互不阻塞(細粒度鎖)
//   - 一律透過 portENTER/EXIT_CRITICAL_SAFE()存取：ESP-IDF 會自動判斷目前是在一般
//     任務還是中斷內執行，並呼叫對應版本；呼叫端(包含之後新寫的模組)完全不必自己
//     分辨，也不會誤把「一般任務用的臨界區 API」用在中斷裡而觸發斷言中止
//     (經查證 ESP-IDF 的 spinlock 對同一顆核心是可重入的：同一條執行路徑上巢狀對
//     同一把鎖上鎖只會疊加計數、解鎖對應次數才會真正釋放，不會自己把自己鎖死)
//   - 對這個檔案裡「目前每一個欄位」都提供對應的 thread-safe get/set 函式
//   - 另提供 SettingsLockGuard(RAII 自旋鎖)，給需要一次讀寫多個欄位、
//     要求彼此一致的情境使用(例如測速環形緩衝的整組快照)
//   - 之後新增模組只要「照著抄」：宣告自己的 struct + 一顆 portMUX_TYPE，
//     再用 SETTINGS_SCALAR_ACCESSOR() 幫每個欄位產生 get/set，就能自動獲得
//     同一等級的保護，不必自己重新設計鎖的邏輯
//
// 使用限制(務必遵守，否則保護形同虛設)：
//   - 自旋鎖只能包住「很快就會結束」的程式碼(幾行欄位讀寫)，因為持有期間會關閉
//     本核心的中斷。嚴禁在鎖內呼叫 Preferences(NVS/Flash)、Serial、delay()等
//     耗時或會被中斷的函式 —— 需要存取 Flash 時，請先在鎖內把值複製到區域變數，
//     解鎖後再對區域變數做 Flash I/O(實際寫法見下方 motor_PID::load()/save())
// =====================================================================================

extern portMUX_TYPE g_speed_mux; // 保護 speed_sensor(settings)；實體在 settings.cpp
extern portMUX_TYPE g_pid_mux;   // 保護 motor_PID(PID_settings)；實體在 settings.cpp
extern portMUX_TYPE g_ina_mux;   // 保護 ina232_sensor(ina_settings)；實體在 settings.cpp
extern portMUX_TYPE g_ui_mux;    // 保護 board_ui(ui_settings)；實體在 settings.cpp
extern portMUX_TYPE g_meas_mux;  // 保護 measure_settings(量測序列＋安全電流＋內阻＋曲線)；實體在 settings.cpp
extern portMUX_TYPE g_bt_mux;    // 保護 bt_telemetry_state；實體在 settings.cpp

// RAII 自旋鎖：建構時上鎖、解構(離開作用域)時自動解鎖，避免忘記解鎖或中途 return
// 導致鎖住不放。任務(task)或中斷(ISR)context都能安全使用(見上方說明)。
// 用法：{ SettingsLockGuard lock(g_speed_mux); /* 在此範圍內存取共享欄位 */ }
class SettingsLockGuard
{
public:
    explicit SettingsLockGuard(portMUX_TYPE &mux) : mux_(mux)
    {
        portENTER_CRITICAL_SAFE(&mux_);
    }
    ~SettingsLockGuard()
    {
        portEXIT_CRITICAL_SAFE(&mux_);
    }
    SettingsLockGuard(const SettingsLockGuard &) = delete;
    SettingsLockGuard &operator=(const SettingsLockGuard &) = delete;

private:
    portMUX_TYPE &mux_;
};

// 產生「單一純量欄位」的 thread-safe get/set(適用 bool/uint8_t/uint16_t/uint32_t/float
// 等一次讀寫即可完成的型別；也適用 bitfield，因為這裡完全不取址、只做值的讀寫)：
//   FuncPrefix_get_Field() / FuncPrefix_set_Field(value)
#define SETTINGS_SCALAR_ACCESSOR(FuncPrefix, StructVar, Mux, Field, FieldType) \
    inline FieldType FuncPrefix##_get_##Field()                                \
    {                                                                          \
        SettingsLockGuard _lock(Mux);                                          \
        return (StructVar).Field;                                              \
    }                                                                          \
    inline void FuncPrefix##_set_##Field(FieldType _value)                     \
    {                                                                          \
        SettingsLockGuard _lock(Mux);                                          \
        (StructVar).Field = _value;                                            \
    }

// 唯讀版本：只產生 get，用於「不可由外部單獨改寫、只能透過專用函式整組更新」的欄位
// (例如測速環形緩衝的 buf_idx/count_number，實際寫入請見 speed_capture_* 系列函式)
#define SETTINGS_SCALAR_GETTER(FuncPrefix, StructVar, Mux, Field, FieldType) \
    inline FieldType FuncPrefix##_get_##Field()                              \
    {                                                                        \
        SettingsLockGuard _lock(Mux);                                        \
        return (StructVar).Field;                                            \
    }

// 變數設定
// speed sensor settings and shared variables
struct speed_sensor
{
    uint16_t read_space = 15;                     // 多久讀取一次速度感測器的值,單位為毫秒(此值亦是PID之計算間隔)
    float now_speed;                              // 現在主動力馬達的轉速(RPM)
    uint8_t buf_idx : buf_idx_bit = 0;            // 測速環形緩衝寫入指標(自動在 0~15 循環)
    uint32_t cap_buffer[(1 << buf_idx_bit)];      // 用於儲存最後的幾個數據(取決於上面的buf_idx)
    uint8_t count_number : buf_idx_bit;           // 指標快照
    uint16_t Timestamp_state[(1 << buf_idx_bit)]; // 用於儲存cap_buffer陣列中數值的狀態(0:可用 1:不可用)
    uint32_t edge_count = 0;                      // 絕對邊緣計數(ISR遞增，用於對應物理格序)
    uint16_t settle_samples = 0;                  // 開環固定 PWM 後，已觀察到的同位置整圈樣本數
    bool speed_valid = false;                     // 目前 now_speed 是否可信(未逾時)
    bool const_speed_ready = false;               // 就緒觀察完成 → 允許定速/調參
    float min_measurable_rpm = RPM_INIT_READABLE; // 可正常讀取的最低轉速門檻
    uint8_t init_phase = SPEED_PHASE_STEP_UP;     // 目前初始化階段
    bool rpm_stable = false;                      // 內部：開環已 hold 在可讀門檻以上(非對外穩調旗標)

    // ---- 對外：轉速穩調旗標(其他模組請讀這個) ----
    // true  = PID+測速皆就緒、無異常、轉速可穩定讀取且已貼近 keep_rpm
    // false = 初始化中 / 調參中 / 故障 / 換目標後尚未到位 / 轉速不可信
    bool speed_stable = false;

    // ---- 與 motor_PID 協調的開環階梯介面 ----
    uint16_t ol_pwm_cmd = 0;       // 目前開環 PWM duty 指令(由 motor_PID 寫入)
    bool ol_hold = false;          // true：停在當前 PWM，進行就緒觀察(穩定確認)
    bool ol_probe_request = false; // motor：settle 結束，請求擷取第一筆可用轉速
    bool ol_probe_ready = false;   // sensor：已擷取到 probe 轉速
    float ol_probe_rpm = 0.0f;     // 該次階梯的 probe 轉速

    // ---- 失控保護(兩模組共用) ----
    bool fault = false;
    uint8_t fault_code = FAULT_NONE;
};
extern volatile speed_sensor settings; // 實體在speed_sensor.cpp中

// ---- speed_sensor 執行緒安全存取介面 ----
// 除了測速環形緩衝那 4 個彼此耦合的欄位(見下方 speed_capture_* )，
// 其餘欄位都各自獨立產生一組 get/set。
SETTINGS_SCALAR_ACCESSOR(speed, settings, g_speed_mux, read_space, uint16_t)
SETTINGS_SCALAR_ACCESSOR(speed, settings, g_speed_mux, now_speed, float)
SETTINGS_SCALAR_GETTER(speed, settings, g_speed_mux, buf_idx, uint8_t)
SETTINGS_SCALAR_GETTER(speed, settings, g_speed_mux, count_number, uint8_t)
SETTINGS_SCALAR_GETTER(speed, settings, g_speed_mux, edge_count, uint32_t)
SETTINGS_SCALAR_ACCESSOR(speed, settings, g_speed_mux, settle_samples, uint16_t)
SETTINGS_SCALAR_ACCESSOR(speed, settings, g_speed_mux, speed_valid, bool)
SETTINGS_SCALAR_ACCESSOR(speed, settings, g_speed_mux, const_speed_ready, bool)
SETTINGS_SCALAR_ACCESSOR(speed, settings, g_speed_mux, min_measurable_rpm, float)
SETTINGS_SCALAR_ACCESSOR(speed, settings, g_speed_mux, init_phase, uint8_t)
SETTINGS_SCALAR_ACCESSOR(speed, settings, g_speed_mux, rpm_stable, bool)
SETTINGS_SCALAR_ACCESSOR(speed, settings, g_speed_mux, speed_stable, bool)
SETTINGS_SCALAR_ACCESSOR(speed, settings, g_speed_mux, ol_pwm_cmd, uint16_t)
SETTINGS_SCALAR_ACCESSOR(speed, settings, g_speed_mux, ol_hold, bool)
SETTINGS_SCALAR_ACCESSOR(speed, settings, g_speed_mux, ol_probe_request, bool)
SETTINGS_SCALAR_ACCESSOR(speed, settings, g_speed_mux, ol_probe_ready, bool)
SETTINGS_SCALAR_ACCESSOR(speed, settings, g_speed_mux, ol_probe_rpm, float)
SETTINGS_SCALAR_ACCESSOR(speed, settings, g_speed_mux, fault, bool)
SETTINGS_SCALAR_ACCESSOR(speed, settings, g_speed_mux, fault_code, uint8_t)

// 供任何新模組(例如量測序列)觸發「整機安全鎖定」用，效果等同 motor_PID/speed_sensor
// 內部各自的 trip_fault()：只負責把 fault 相關欄位設成同一組一致的事實。
// ★呼叫端仍須自行 ledcWrite(MOTOR_PWM_PIN, 0) 立即切斷★──本函式定義在 settings.h，
// 不知道 MOTOR_PWM_PIN(定義在 pins.h)，且呼叫端通常不想等 motor_PID 任務下一輪
// 被喚醒才停下來(可能還要等最多一個 read_space)。這是新增的純附加函式，
// 不修改/不影響 motor_PID.cpp、speed_sensor.cpp 既有的 3 份 trip_fault() 實作。
inline void speed_trigger_fault(uint8_t code)
{
    SettingsLockGuard lock(g_speed_mux);
    settings.fault = true;
    settings.fault_code = code;
    settings.const_speed_ready = false;
    settings.ol_hold = false;
    settings.ol_probe_request = false;
    settings.ol_probe_ready = false;
    settings.init_phase = SPEED_PHASE_FAULT;
    settings.speed_valid = false;
    settings.speed_stable = false;
    settings.ol_pwm_cmd = 0;
}

// ---- 測速環形緩衝：ISR 專用寫入 / 任務端整組快照 ----
// cap_buffer(時間戳陣列)、buf_idx(寫入指標)、Timestamp_state、edge_count 這 4 個
// 欄位必須「當成一組」一起變化才有意義：例如只更新 buf_idx 卻沒同步寫入 cap_buffer，
// 環形緩衝的索引就會對不上實際資料。因此不比照上面用單欄位巨集，改用下面幾個
// 專用函式包住整組操作，全部共用 g_speed_mux，讓 ISR 寫入與任務端讀取彼此互斥。
//
// speed_capture_push_edge()：整個程式唯一允許寫入這組欄位的地方，只給 ISR 呼叫。
// 標成 IRAM_ATTR 是防禦性寫法：此函式只有 ISR 這唯一呼叫端、又是 inline，正常會被
// 編譯器直接內聯進已經是 IRAM_ATTR 的 mcpwm_cap_cb() 裡；但萬一某次編譯沒內聯，
// 沒有 IRAM_ATTR 的話它就會留在 Flash 段——若這時剛好其他任務正在寫 NVS(Flash)、
// 快取被短暫關閉，中斷卻要去 Flash 抓這段程式碼執行，就會直接當機。
inline void IRAM_ATTR speed_capture_push_edge(uint32_t tick_value)
{
    SettingsLockGuard lock(g_speed_mux);
    settings.cap_buffer[settings.buf_idx] = tick_value;
    settings.Timestamp_state[settings.buf_idx] = 0;
    settings.buf_idx = settings.buf_idx + 1;
    settings.edge_count = settings.edge_count + 1;
}

// 任務端快照：out_buffer 至少需 (1<<buf_idx_bit) 筆容量。
// out_count_number 會拿到「快照當下」的 buf_idx(同時也順便發布進共享結構供除錯讀取，
// 與舊版行為一致)。回傳值為實際寫入 out_buffer 的筆數。
inline uint8_t speed_capture_snapshot(uint32_t *out_buffer, uint8_t out_buffer_len,
                                      uint8_t &out_count_number, uint32_t &out_edge_count)
{
    SettingsLockGuard lock(g_speed_mux);
    settings.count_number = settings.buf_idx;
    out_count_number = settings.count_number;
    out_edge_count = settings.edge_count;
    const uint8_t n = (out_buffer_len < (1 << buf_idx_bit)) ? out_buffer_len : (uint8_t)(1 << buf_idx_bit);
    for (uint8_t i = 0; i < n; i++)
    {
        out_buffer[i] = settings.cap_buffer[i];
    }
    return n;
}

// 任務端標記某個緩衝格「資料已被用過」；語意上與 push_edge 存取同一組欄位，故同樣走 g_speed_mux
inline void speed_capture_mark_used(uint8_t idx)
{
    SettingsLockGuard lock(g_speed_mux);
    settings.Timestamp_state[idx & ((1 << buf_idx_bit) - 1)] = 1;
}

struct motor_PID
{
    float keep_rpm = 0;        // 目標轉速(RPM)；0 時調參完成後會鎖存當前轉速
    uint32_t pwm_freq = 20000; // PWM頻率
    uint8_t pwm_res = 10;      // PWM解析度(位元)
    float Kp;                  // 以下參數如要修改預設值請到void load()內修改
    float Ki;
    float Kd;
    bool tuned = false;           // NVS：是否已完成過自動調參
    bool autotune_active = false; // 執行期：目前是否正在 sTune
    bool autotune_done = false;   // 執行期：本次上電是否已完成調參(或已跳過)

    // 【結構體自我載入】
    // ★注意：Preferences(NVS/Flash) I/O 可能耗時數毫秒，絕不可在自旋鎖內執行
    // (鎖內會關閉本核心中斷，長時間關閉中斷會拖慢/卡住其他中斷驅動的功能)。
    // 因此策略是：只用鎖保護「讀/寫共享欄位」這幾行極短的操作，Flash I/O 全程在鎖外，
    // 對外表現仍是「一次要嘛拿到舊值全套、要嘛拿到新值全套」的原子更新。
    void load() volatile
    {
        float default_keep;
        {
            SettingsLockGuard lock(g_pid_mux);
            default_keep = keep_rpm; // 取目前值當 NVS 沒有紀錄時的預設 fallback
        }

        Preferences prefs;
        prefs.begin("pid_cfg", true); // 只讀模式
        const float f_kp = prefs.getFloat("Kp", 0.0f);
        const float f_ki = prefs.getFloat("Ki", 0.0f);
        const float f_kd = prefs.getFloat("Kd", 0.0f);
        const bool b_tuned = prefs.getBool("tuned", false);
        const float f_keep = prefs.getFloat("keep_rpm", default_keep);
        prefs.end();

        SettingsLockGuard lock(g_pid_mux);
        Kp = f_kp;
        Ki = f_ki;
        Kd = f_kd;
        tuned = b_tuned;
        keep_rpm = f_keep;
    }

    // 【結構體自我儲存】(理由同上，Flash I/O 全程在鎖外)
    void save() volatile
    {
        float f_kp, f_ki, f_kd, f_keep;
        bool b_tuned;
        {
            SettingsLockGuard lock(g_pid_mux);
            f_kp = Kp;
            f_ki = Ki;
            f_kd = Kd;
            b_tuned = tuned;
            f_keep = keep_rpm;
        }

        Preferences prefs;
        prefs.begin("pid_cfg", false); // 讀寫模式
        prefs.putFloat("Kp", f_kp);
        prefs.putFloat("Ki", f_ki);
        prefs.putFloat("Kd", f_kd);
        prefs.putBool("tuned", b_tuned);
        prefs.putFloat("keep_rpm", f_keep);
        prefs.end();
    }
};
extern volatile motor_PID PID_settings; // 實體在motor_PID.cpp中

// ---- motor_PID 執行緒安全存取介面 ----
SETTINGS_SCALAR_ACCESSOR(pid, PID_settings, g_pid_mux, keep_rpm, float)
SETTINGS_SCALAR_ACCESSOR(pid, PID_settings, g_pid_mux, pwm_freq, uint32_t)
SETTINGS_SCALAR_ACCESSOR(pid, PID_settings, g_pid_mux, pwm_res, uint8_t)
SETTINGS_SCALAR_ACCESSOR(pid, PID_settings, g_pid_mux, Kp, float)
SETTINGS_SCALAR_ACCESSOR(pid, PID_settings, g_pid_mux, Ki, float)
SETTINGS_SCALAR_ACCESSOR(pid, PID_settings, g_pid_mux, Kd, float)
SETTINGS_SCALAR_ACCESSOR(pid, PID_settings, g_pid_mux, tuned, bool)
SETTINGS_SCALAR_ACCESSOR(pid, PID_settings, g_pid_mux, autotune_active, bool)
SETTINGS_SCALAR_ACCESSOR(pid, PID_settings, g_pid_mux, autotune_done, bool)

// INA232 感測結果(晶片每次轉換即讀；下列為 ESP32 EMA 後的對外數值)
struct ina232_sensor
{
    float bus_V = 0.0f;        // 匯流排電壓(V)
    float current_A = 0.0f;    // 電流(A)，有號
    float power_W = 0.0f;      // 功率(W) = |V×I| 或由暫存器換算後再平滑
    float shunt_mV = 0.0f;     // 分流電壓(mV)，除錯用
    bool online = false;       // 是否成功辨識到 INA232(Manufacturer ID)
    bool data_valid = false;       // 是否已有至少一筆成功讀值
    bool current_plausible = true; // 帶載時 V/I 是否符合 LOAD_TEST_RESISTOR_OHM；false 時 current_A 仍是原始讀值，禁止當測試數據
    uint32_t sample_count = 0;     // 累計成功讀取次數
};
extern volatile ina232_sensor ina_settings; // 實體在 ina232.cpp

// ---- ina232_sensor 執行緒安全存取介面 ----
SETTINGS_SCALAR_ACCESSOR(ina, ina_settings, g_ina_mux, bus_V, float)
SETTINGS_SCALAR_ACCESSOR(ina, ina_settings, g_ina_mux, current_A, float)
SETTINGS_SCALAR_ACCESSOR(ina, ina_settings, g_ina_mux, power_W, float)
SETTINGS_SCALAR_ACCESSOR(ina, ina_settings, g_ina_mux, shunt_mV, float)
SETTINGS_SCALAR_ACCESSOR(ina, ina_settings, g_ina_mux, online, bool)
SETTINGS_SCALAR_ACCESSOR(ina, ina_settings, g_ina_mux, data_valid, bool)
SETTINGS_SCALAR_ACCESSOR(ina, ina_settings, g_ina_mux, current_plausible, bool)
SETTINGS_SCALAR_ACCESSOR(ina, ina_settings, g_ina_mux, sample_count, uint32_t)

// 讀-改-寫一次鎖完成的原子遞增，避免「讀出→+1→寫回」中間被插隊造成遺失更新
// (目前只有 ina232 任務自己會呼叫，但介面上比照多寫入者情境設計比較保險)
inline uint32_t ina_increment_sample_count()
{
    SettingsLockGuard lock(g_ina_mux);
    ina_settings.sample_count = ina_settings.sample_count + 1;
    return ina_settings.sample_count;
}

// 板載互動介面共享狀態
struct board_ui
{
    uint8_t state = UI_WAIT_START; // 目前 UI 狀態機
    bool motor_started = false;    // 是否已呼叫馬達/測速初始化
    bool oled_ok = false;          // OLED 是否初始化成功
};
extern volatile board_ui ui_settings; // 實體在 board_ui.cpp

// ---- board_ui 執行緒安全存取介面 ----
SETTINGS_SCALAR_ACCESSOR(ui, ui_settings, g_ui_mux, state, uint8_t)
SETTINGS_SCALAR_ACCESSOR(ui, ui_settings, g_ui_mux, motor_started, bool)
SETTINGS_SCALAR_ACCESSOR(ui, ui_settings, g_ui_mux, oled_ok, bool)

// =====================================================================================
// 量測序列：安全電流／內阻／曲線＋極限轉速／發電機斷線偵測／負載開關／藍牙遙測
// =====================================================================================
// 整體流程(由新模組 measure_seq 主導，詳見各 *_ARCH.md)：
//   MIN_SPEED_HOLD(定速在 MEASURE_MIN_RPM) → SAFE_CURRENT → RESISTANCE → CURVE_CALC → DONE
// 這 4 個新模組(measure_seq/safe_current/gen_resistance/curve_calc)全部跑在
// 「同一個」FreeRTOS 任務(measure_seq 建立)裡，用函式呼叫依序驅動，彼此之間
// 保證不會並行寫入，因此以下 measure_settings 一個結構＋一顆鎖(g_meas_mux)
// 就足夠涵蓋全部欄位，不需要比照 speed_sensor/motor_PID 拆成多顆鎖
// (三份 ARCH.md 文件本身也明白允許「併入同一個結構、一顆鎖」這個做法)。
// 唯一的寫入者是 measure_seq 所在任務；其他任務(board_ui/bt_telemetry/main)一律只讀。
//
// ★需要依實際機台調整的常數已在下方個別註記「依實際機台調整」。
// =====================================================================================

// ★這是「開環起動判定可讀轉速」的門檻，不是拿來強制命令 PID 的目標值★
// 本機台 PWM→RPM 增益極不均勻(見 motor_PID.cpp 開頭大量註解)，開環階梯只保證停在
// 第一個「達到這個門檻以上」的 PWM 階，實際停下來的轉速通常會比這裡高一截，且每次
// 開機可能不完全一樣。sTune 自動調參正是針對「停下來那個實際轉速附近」做特性化，
// 因此量測序列全程都必須沿用那個自然停下來的轉速當起點(見 safe_current_reset()
// 讀 pid_get_keep_rpm() 的做法)，絕不可以硬把 keep_rpm 蓋成這個常數──否則等於
// 拿著在別的工作點特性化出來的增益，命令系統跳去一個沒被特性化過的轉速，在這種
// 增益極不均勻的機台上很容易造成長時間震盪不收斂(已實測踩到過這個坑)。
// 這個常數目前只用於：(1) 量測序列的說明文件/事件訊息參考值，(2) 曲線報表在
// 還沒有任何實測起點可用時的備援下限。
#define MEASURE_MIN_RPM (RPM_INIT_READABLE + RPM_INIT_MARGIN)

// ---- 負載開關：沿用 pins.h 既有的 SERVO_PIN(不改名)，高電位＝把測試負載接上發電機 ----
#define LOAD_SWITCH_ACTIVE_HIGH 1    // 1=高電位接通負載；0=低電位接通
#define LOAD_TEST_RESISTOR_OHM (10.0f / 3.0f) // 三顆 10Ω 並聯≈3.33Ω；須與實物一致，程式不會自測外接電阻
#define LOAD_SWITCH_SETTLE_MS 60     // 切換負載後，繼電器/MOSFET 接點穩定的等待(ms)

// ---- 發電機防反接串聯蕭特基二極體(SS54)電壓補償 ----
// 硬體：發電機(鱷魚夾)→ SS54(防反接，順向才讓電流流向 INA232/負載)→ R8 分流電阻/INA232。
// 二極體有順向壓降 V_F(I)，INA232 量到的匯流排電壓會比發電機端子的真實電壓低這一截，
// 且 V_F 隨電流變大而變大(不是固定值)。若不補償，量到的 Voc/V 會系統性偏低，
// 算出來的 R_th、k_e、以及最終曲線報表都會跟著偏。
// 電流量測不受影響(同一顆二極體是串聯元件，流過分流電阻的電流跟流過二極體的電流
// 是同一個，只有電壓被二極體「吃掉」一截，不需要補償電流)。
//
// ★這是「近似模型」，不是精確查表★：SS54 datasheet 保證的唯一精確點是
// V_F(5A)≤0.55V(幾乎所有廠牌 SS54 都寫這個規格)，但那是靠近額定電流、由封裝
// 串聯電阻主導的區間，本應用最多用到約 0.5A，離那個區間還遠，直接套用會失真。
// 這裡改成只針對本應用真正會用到的 0～0.6A 範圍校準：用「每十倍電流壓降變化
// 一個常數(V/decade)」這個蕭特基二極體常見量級，錨定在一個中電流參考點。
// ★想要更準，最直接的方法★：拿實驗室電源＋已知電阻，量手上這顆實際二極體在
// 約 0.05A、0.2A、0.5A 三個電流下的真實順向壓降，回頭調整下面兩個常數即可
// (SS54_VF_REF_V 對準 SS54_VF_REF_I_A 那個電流量到的實測值，SS54_VF_SLOPE_PER_DECADE
// 用兩個實測點反推：斜率＝(V2-V1)/log10(I2/I1))。
#define SS54_PROTECTION_DIODE_PRESENT 1  // 0＝沒有裝這顆防反接二極體，完全不做電壓補償
#define SS54_VF_REF_I_A 0.1f             // 校準基準電流(A)
#define SS54_VF_REF_V 0.25f              // 基準電流下的順向壓降(V)★近似值，建議實測校正
#define SS54_VF_SLOPE_PER_DECADE 0.10f   // 每十倍電流變化的壓降變化量(V/decade)★近似值，建議實測校正
#define SS54_VF_MIN_CURRENT_A 0.0005f    // 電流小於此值時視同這個值去算(避免 log(0)/負電流)

// 依目前實際電流估算 SS54 的順向壓降(V)。回傳 0 代表沒裝二極體(補償停用)。
// 只在「本次讀值有效」的呼叫端使用，不在這裡驗證電流合理性(呼叫端自己已經在做)。
inline float ss54_forward_drop_V(float forward_current_A)
{
#if SS54_PROTECTION_DIODE_PRESENT
    const float i = fmaxf(fabsf(forward_current_A), (float)SS54_VF_MIN_CURRENT_A);
    const float decades = log10f(i / (float)SS54_VF_REF_I_A);
    const float v = (float)SS54_VF_REF_V + (float)SS54_VF_SLOPE_PER_DECADE * decades;
    return fmaxf(v, 0.0f);
#else
    (void)forward_current_A;
    return 0.0f;
#endif
}

// 把 INA232 量到的匯流排電壓换算回發電機端子的真實電壓：真實值一定 ≥ 量到的值
// (二極體只會讓電壓變低，不會變高)。current_A 一律傳「這一筆電壓對應的那個電流」，
// 兩者必須是同一次讀值，不要混用不同時間點的 V 與 I。
inline float ss54_compensate_voltage_V(float measured_bus_V, float current_A)
{
    return measured_bus_V + ss54_forward_drop_V(current_A);
}

// ---- 發電機／負載鱷魚夾脫落：非故障，重按 START 從中斷的那一檔／那一點續測（保留已通過進度） ----
#define GEN_LINK_LOST_V_MAX 0.15f     // 匯流排電壓低於此值視為「無輸出」(★依實際 k_e 調整，須遠小於最低轉速下的 Voc)
#define GEN_LINK_LOST_TIMEOUT_MS 3000 // 連續無輸出超過此時間才判定斷線("超過數秒")，避免瞬間雜訊誤判

// ---- 安全電流識別(SAFE_CURRENT_ARCH.md) ----
#define SAFE_I_HARD_CEILING_A 0.6f                      // 硬電流天花板，必須低於 INA232 0.8A 滿量程(★依實際電機調整)
#define SAFE_I_MIN_VALID_A 0.02f                        // 電流低於此值才「有資格」再看是不是開路
#define SAFE_I_OPEN_V_RATIO 0.50f                       // V >= Voc*此比例才像真開路(帶載時端子會被拉低很多)
#define SAFE_I_CONTACT_CONFIRM_MS 400                   // 開路特徵需連續成立才當夾子鬆脫；INA 單筆電流雜訊不算

// 帶載且端子仍被測試電阻拉在低電壓時，電流必須約等於 V/R。I≈0 而 V 仍是 1～2V：這一筆不是真實電流，
// 禁止寫進 I_cont／熱下垂／內阻。電壓已高到不像帶載（真開路）則不算 mismatch。
inline bool ina_loaded_vi_mismatch(float bus_V, float current_A)
{
    const float i_from_v = bus_V / (float)LOAD_TEST_RESISTOR_OHM;
    if (i_from_v < (float)SAFE_I_MIN_VALID_A)
    {
        return false;
    }
    if (i_from_v > (float)SAFE_I_HARD_CEILING_A * 1.2f)
    {
        return false;
    }
    return fabsf(current_A) < i_from_v * (float)INA_I_VS_V_MIN_RATIO;
}
#define SAFE_I_PASS_DROOP_RATIO 0.03f                   // 熱穩後下垂 ≤ 此比例(3%) → 本檔通過
#define SAFE_I_LIMIT_DROOP_RATIO 0.08f                  // 熱穩後下垂 ≥ 此比例(8%) → 本檔視為上限(不算通過)
#define SAFE_RPM_STEP 300.0f                            // 每檔轉速增量(★依實際機台調整)
#define SAFE_RPM_MAX_CEILING 14000.0f                   // 僅曲線／n_lim 遠端守衛(須遠低於飛車 RPM_RUNAWAY_MAX)。安全電流階梯不再拿它當終點：PWM 頂滿仍轉不到下一檔 → 用上一檔進內阻
#define SAFE_DRIVE_STALL_HOLD_MS 3000                   // WAIT_SPEED 期間 PWM 已頂滿且仍離目標太遠、持續此時長 → 判定主動力轉不到
#define SAFE_DRIVE_STALL_PWM_SLACK 8                    // 距滿格少於此 count 視為 PWM 已頂滿(10-bit 滿格 1023)
#define SAFE_HANDOFF_COAST_MIN_MS 500                   // 進內阻前 PWM=0 最短滑行：光柵可能已讀偏低，不可立刻放行 PID
#define SAFE_HANDOFF_COAST_MAX_MS 8000                  // 滑行上限；逾時 Voc 仍高 → 硬故障(PWM 可能沒真正歸零)
#define SAFE_HANDOFF_VOC_RATIO 1.20f                    // 滑行完成：匯流排 Voc ≤ 上一通過檔 Voc × 此比例(Voc 才是真實轉速)
#define SAFE_ELECTRICAL_SETTLE_MS 300                   // 接通負載後，電氣穩定等待(ms)
#define SAFE_THERMAL_CHECK_WINDOW_MS 20000              // 判斷「打平」的觀察窗(ms)
#define SAFE_THERMAL_SOAK_MAX_MS (15UL * 60UL * 1000UL) // 熱穩逾時上限(15 分鐘)
#define SAFE_COOLDOWN_MS (15UL * 1000UL)                // 檔與檔之間最短冷卻時間(ms)(★依實際散熱調整)
#define SAFE_OC_SAMPLE_MS 500                           // 開路取樣視窗(ms)
#define SAFE_OC_MAX_CURRENT_A 0.02f                     // 開路電流高於此值視為負載黏住(接線異常)
#define SAFE_SPEED_WAIT_TIMEOUT_MS 25000                // 等待穩調的基底逾時(ms)
#define SPEED_WAIT_TIMEOUT_PER_RPM_MS 7                 // 隨目標轉速追加的逾時(ms/RPM)

// ---- 發電機跳刷偵測(安全電流階梯期間，只用 INA) ----
// 對象是待測發電機換向器，不是主動力 775、也不是光柵。帶載時電流瞬間掉向 0、
// 開路時電壓閃爍 → 判跳刷。偵測到且至少已有一檔通過 → 用上一檔 I_cont，
// 報表 n_lim 鎖在那一檔，不算硬故障。第一檔就跳刷才鎖定。
#define BRUSH_JUMP_MIN_RPM 2800.0f           // 低於此轉速不判跳刷(低轉 INA 雜訊較多)
#define BRUSH_JUMP_I_DROP_RATIO 0.45f        // 帶載電流掉到電氣穩態的此比例以下算一次閃斷
#define BRUSH_JUMP_I_HITS 5                  // 觀察窗內閃斷次數
#define BRUSH_JUMP_I_WINDOW_MS 800           // 帶載電流閃斷觀察窗(ms)
#define BRUSH_JUMP_V_OC_RATIO 0.22f          // 開路電壓相對 EMA 掉超過此比例算一次
#define BRUSH_JUMP_V_HITS 5                  // 開路電壓閃爍次數
#define BRUSH_JUMP_V_WINDOW_MS 800           // 開路電壓觀察窗(ms)
#define BRUSH_JUMP_IGNORE_AFTER_CONNECT_MS 500 // 剛接通負載的電流階躍不計入跳刷

// ---- 內阻識別(INTERNAL_RESISTANCE_ARCH.md) ----
#define RES_MAX_POINTS 3                // 高/中/低三個轉速點(量測順序為高→中→低，見 gen_resistance.cpp)
#define RES_OC_SAMPLE_MS 400            // 開路取樣視窗(ms)
#define RES_LOAD_SAMPLE_MS 400          // 帶載取樣視窗(ms)
#define RES_MIN_VALID_A 0.02f           // 帶載電流低於此值視為未接上
#define RES_MAX_OHM 200.0f              // 算出的 R_th 超過此值視為失敗(接線異常)
#define RES_MIN_OHM 0.01f               // 算出的 R_th 低於此值(含負值)視為失敗(短路/接線異常)
#define RES_SPEED_WAIT_TIMEOUT_MS SAFE_SPEED_WAIT_TIMEOUT_MS // 與安全電流共用基底；實際逾時見 speed_wait_timeout_ms()

// 依目標轉速計算「等待 speed_stable」的逾時：高轉階躍較慢收斂，需較長等待
inline uint32_t speed_wait_timeout_ms(float target_rpm)
{
    const float rpm = (target_rpm > 0.0f) ? target_rpm : 0.0f;
    const uint32_t extra = (uint32_t)(rpm * (float)SPEED_WAIT_TIMEOUT_PER_RPM_MS);
    return (uint32_t)SAFE_SPEED_WAIT_TIMEOUT_MS + extra;
}

// ---- 曲線＋極限轉速計算(LIMIT_RPM_ARCH.md) ----
#define CURVE_RPM_STEP 100.0f // 輸出曲線的轉速步進(RPM)
#define CURVE_V_ALLOW 40.0f   // 端電壓硬上限(V)，需低於 INA232/後級耐壓(★依實際後級調整)
#define MAX_CURVE_POINTS 180  // 曲線陣列容量上限(涵蓋到 RPM_RUNAWAY_MAX 仍綽綽有餘)

// 頂層量測序列階段
enum measure_phase : uint8_t
{
    MEAS_IDLE = 0,           // 尚未啟動(等 START 觸發 motor/speed 任務)
    MEAS_MIN_SPEED_HOLD = 1, // 定速至 MEASURE_MIN_RPM，等待 speed_stable
    MEAS_SAFE_CURRENT = 2,   // 安全電流識別中
    MEAS_RESISTANCE = 3,     // 內阻識別中
    MEAS_CURVE_CALC = 4,     // 曲線＋極限轉速計算中(馬達已關閉)
    MEAS_DONE = 5,           // 全部完成，馬達已關閉；UI 回到待測，可再按 START 重測
    MEAS_LINK_LOST = 6,      // 量測暫停中（夾子鬆脫或 INA V/I 不一致），等待重按 START 續測
};

// 暫停原因：同一套 START 續測，畫面／上位機文案不同，避免把 INA 讀錯當成夾子鬆脫
enum meas_pause_cause : uint8_t
{
    PAUSE_CAUSE_NONE = 0,
    PAUSE_CAUSE_CONTACT = 1,      // 負載沒接上／電壓回到 Voc
    PAUSE_CAUSE_INA_MISMATCH = 2, // 帶載電壓還在，電流讀值與 V/R 對不上（本檔數據作廢，重做這一檔）
};

// 安全電流模組內部子階段(對應 SAFE_CURRENT_ARCH.md 狀態機 B~G)
enum safe_current_phase : uint8_t
{
    SAFE_PH_PREP = 0,         // 準備下一檔：負載斷開、設定目標轉速
    SAFE_PH_WAIT_SPEED = 1,   // 等待穩調
    SAFE_PH_OC_SAMPLE = 2,    // 開路取樣(確認負載未黏住、記錄 Voc)
    SAFE_PH_CONNECT = 3,      // 接通負載、等電氣穩定
    SAFE_PH_THERMAL_SOAK = 4, // 熱穩觀察
    SAFE_PH_JUDGE = 5,        // 本檔判定(先斷負載)
    SAFE_PH_COOLDOWN = 6,     // 檔與檔之間冷卻
    SAFE_PH_HANDOFF = 7,      // 進內阻前：PWM=0 滑行到上一檔 Voc，再拉回上一通過檔
};

// 內阻模組內部子階段(對應 INTERNAL_RESISTANCE_ARCH.md 狀態機 P1~P6)
enum resistance_phase : uint8_t
{
    RES_PH_PREP = 0,        // 設定本點目標轉速
    RES_PH_WAIT_SPEED = 1,  // 等待穩調
    RES_PH_OC_SAMPLE = 2,   // 開路取樣
    RES_PH_CONNECT = 3,     // 接通負載、等定速恢復
    RES_PH_LOAD_SAMPLE = 4, // 帶載取樣(取完立刻斷開)
    RES_PH_COMPUTE = 5,     // 計算本點 R_th、判斷是否可信
};

// 曲線模組對外 n_lim 的限制原因
enum curve_limit_reason : uint8_t
{
    LIMIT_REASON_NONE = 0,
    LIMIT_REASON_RL_CURRENT = 1,   // 固定電阻電流封頂(n_rl)最嚴
    LIMIT_REASON_MPP_KNEE = 2,     // 最大功率點合法上限(n_knee)最嚴
    LIMIT_REASON_OPEN_VOLTAGE = 3, // 開路耐壓上限(n_voc)最嚴
    LIMIT_REASON_CEILING = 4,      // 人為天花板最嚴，或資料不足只能用天花板
    LIMIT_REASON_BRUSH_JUMP = 5,   // 跳刷：尚未熱封頂就接觸不穩，n_lim 鎖在上一通過檔
    LIMIT_REASON_DRIVE_LIMIT = 6,  // 主動力轉不到下一檔：I_cont／n_lim 鎖在上一通過檔，不是硬故障
};

// 量測序列共享狀態：measure_seq 所在任務是唯一寫入者，其餘任務一律唯讀。
struct measure_settings
{
    // ---- 頂層流程 ----
    uint8_t phase = MEAS_IDLE;        // 見 measure_phase
    uint8_t resume_phase = MEAS_IDLE; // 夾子暫停時，記住要恢復到哪個階段(MIN_SPEED_HOLD/SAFE_CURRENT/RESISTANCE)
    bool session_active = false;      // 本次 START 之後量測序列是否已啟動(供顯示用)

    // ---- 鱷魚夾／量測線鬆脫偵測與暫停/續測（START 從中斷檔／點繼續，不整段重跑） ----
    bool armed = false;            // 本次啟動後是否已出現過一次 speed_stable(之前不信任電壓判斷)
    bool link_lost = false;        // 對外：目前是否處於夾子鬆脫暫停
    bool pause_request = false;    // 寫給 motor_PID：true=把 PWM 歸零並停在 PID_PAUSED(motor_PID 只讀)
    bool resume_requested = false; // 由 main.cpp(偵測到 START 按下)寫入 true；measure_seq 消化後清回 false
    bool restart_requested = false; // 測完回到待測後再按 START：整段重跑(不重開機、不重調參)
    uint8_t pause_cause = PAUSE_CAUSE_NONE;

    // ---- 負載開關(對外顯示用；實際腳位操作在 measure_seq.cpp) ----
    bool load_connected = false; // 目前負載開關電位是否等於「已接通」

    // ==================== 安全電流模組 ====================
    uint8_t safe_phase = SAFE_PH_PREP; // 見 safe_current_phase
    float safe_start_rpm = 0.0f;       // 識別開始當下，PID 自動調參後自然停下來的轉速(第 0 檔基準)
    float safe_target_rpm = 0.0f;      // 本檔目標轉速
    float safe_oc_voltage_V = 0.0f;    // 本檔開路電壓(Voc)
    float safe_electrical_A = 0.0f;    // 本檔剛接通、電氣穩定後的電流
    float safe_hot_A = 0.0f;           // 本檔熱穩後的電流
    float safe_droop_ratio = 0.0f;     // 本檔下垂比例((electrical-hot)/electrical)
    uint32_t safe_phase_start_ms = 0;  // 本子階段開始時間(供逾時判斷與畫面顯示經過時間)
    bool safe_done = false;            // 識別是否已完成且可信(i_cont 有效)
    bool safe_pass_any = false;        // 是否至少有一檔通過(第一檔就失敗要能分辨)
    float safe_i_cont_A = 0.0f;        // 結果：連續安全電流 I_cont
    float safe_i_cont_rpm = 0.0f;      // 結果：量到 I_cont 當時的轉速(通過的最高檔)
    float safe_last_pass_rpm = 0.0f;   // 內部：目前為止通過的最高檔轉速(逐檔更新)
    float safe_last_pass_A = 0.0f;     // 內部：目前為止通過的最高檔熱穩電流
    float safe_last_pass_oc_V = 0.0f;  // 內部：上一通過檔的開路電壓(交接滑行用；光柵不可信時 Voc 才是真實轉速)
    float brush_jump_rpm = 0.0f;       // >0：本輪因跳刷提前結束，報表 n_lim 不得超過此轉速
    float drive_limit_rpm = 0.0f;      // >0：本輪因主動力轉不到下一檔結束，報表 n_lim 不得超過此轉速

    // ==================== 內阻模組 ====================
    uint8_t res_phase = RES_PH_PREP; // 見 resistance_phase
    uint8_t res_point_index = 0;     // 目前是第幾個轉速點(0..RES_MAX_POINTS-1)
    float res_target_rpm = 0.0f;
    float res_oc_voltage_V = 0.0f;
    float res_load_V = 0.0f;
    float res_load_A = 0.0f;
    uint32_t res_phase_start_ms = 0;
    bool res_done = false;
    uint8_t res_valid_points = 0; // 成功量到、通過檢查的點數
    // res_point_* 三個陣列一組一組對應，透過 meas_res_get_point()/meas_res_set_point() 存取
    float res_point_rpm[RES_MAX_POINTS];
    float res_point_rth[RES_MAX_POINTS];
    float res_point_ke[RES_MAX_POINTS]; // 該點 Voc/rpm
    float res_rth_ohm = 0.0f;           // 結果：R_th(有效點平均)
    float res_ke_v_per_rpm = 0.0f;      // 結果：k_e(有效點平均，V/RPM)

    // ==================== 曲線＋極限轉速模組 ====================
    bool curve_done = false;
    float curve_n_rl = 0.0f;   // 固定測試電阻上的功率封頂轉速
    float curve_n_knee = 0.0f; // 最大功率點合法上限轉速(可變負載才有意義)
    float curve_n_voc = 0.0f;  // 開路耐壓上限轉速
    float curve_n_lim = 0.0f;  // 對外：最終採用的轉速上限
    uint8_t curve_limit_reason = LIMIT_REASON_NONE;
    uint16_t curve_point_count = 0; // 實際填入 curve_* 陣列的點數
    // curve_* 四個陣列一組一組對應，透過 meas_curve_get_point()/meas_curve_set_point() 存取
    float curve_rpm[MAX_CURVE_POINTS];
    float curve_v_at_maxp[MAX_CURVE_POINTS];
    float curve_p_max[MAX_CURVE_POINTS];
    float curve_r_opt[MAX_CURVE_POINTS];
};
extern volatile measure_settings meas_settings; // 實體在 measure_seq.cpp 中

// ---- measure_settings 執行緒安全存取介面(純量欄位) ----
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, phase, uint8_t)
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, resume_phase, uint8_t)
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, session_active, bool)
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, armed, bool)
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, link_lost, bool)
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, pause_request, bool)
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, resume_requested, bool)
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, restart_requested, bool)
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, pause_cause, uint8_t)
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, load_connected, bool)

SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, safe_phase, uint8_t)
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, safe_start_rpm, float)
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, safe_target_rpm, float)
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, safe_oc_voltage_V, float)
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, safe_electrical_A, float)
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, safe_hot_A, float)
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, safe_droop_ratio, float)
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, safe_phase_start_ms, uint32_t)
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, safe_done, bool)
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, safe_pass_any, bool)
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, safe_i_cont_A, float)
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, safe_i_cont_rpm, float)
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, safe_last_pass_rpm, float)
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, safe_last_pass_A, float)
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, safe_last_pass_oc_V, float)
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, brush_jump_rpm, float)
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, drive_limit_rpm, float)

SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, res_phase, uint8_t)
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, res_point_index, uint8_t)
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, res_target_rpm, float)
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, res_oc_voltage_V, float)
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, res_load_V, float)
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, res_load_A, float)
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, res_phase_start_ms, uint32_t)
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, res_done, bool)
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, res_valid_points, uint8_t)
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, res_rth_ohm, float)
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, res_ke_v_per_rpm, float)

SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, curve_done, bool)
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, curve_n_rl, float)
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, curve_n_knee, float)
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, curve_n_voc, float)
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, curve_n_lim, float)
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, curve_limit_reason, uint8_t)
SETTINGS_SCALAR_ACCESSOR(meas, meas_settings, g_meas_mux, curve_point_count, uint16_t)

// ---- 內阻三點陣列：整組讀寫，避免 rpm/rth/ke 三個陣列各自上鎖造成瞬間不一致 ----
inline bool meas_res_set_point(uint8_t idx, float rpm, float rth, float ke)
{
    if (idx >= RES_MAX_POINTS)
    {
        return false;
    }
    SettingsLockGuard lock(g_meas_mux);
    meas_settings.res_point_rpm[idx] = rpm;
    meas_settings.res_point_rth[idx] = rth;
    meas_settings.res_point_ke[idx] = ke;
    return true;
}

inline bool meas_res_get_point(uint8_t idx, float &rpm, float &rth, float &ke)
{
    if (idx >= RES_MAX_POINTS)
    {
        return false;
    }
    SettingsLockGuard lock(g_meas_mux);
    rpm = meas_settings.res_point_rpm[idx];
    rth = meas_settings.res_point_rth[idx];
    ke = meas_settings.res_point_ke[idx];
    return true;
}

// ---- 曲線表：整組讀寫(rpm/V/P/R_opt 四個陣列對應同一個索引) ----
inline bool meas_curve_set_point(uint16_t idx, float rpm, float v, float p, float r_opt)
{
    if (idx >= MAX_CURVE_POINTS)
    {
        return false;
    }
    SettingsLockGuard lock(g_meas_mux);
    meas_settings.curve_rpm[idx] = rpm;
    meas_settings.curve_v_at_maxp[idx] = v;
    meas_settings.curve_p_max[idx] = p;
    meas_settings.curve_r_opt[idx] = r_opt;
    return true;
}

inline bool meas_curve_get_point(uint16_t idx, float &rpm, float &v, float &p, float &r_opt)
{
    if (idx >= MAX_CURVE_POINTS)
    {
        return false;
    }
    SettingsLockGuard lock(g_meas_mux);
    rpm = meas_settings.curve_rpm[idx];
    v = meas_settings.curve_v_at_maxp[idx];
    p = meas_settings.curve_p_max[idx];
    r_opt = meas_settings.curve_r_opt[idx];
    return true;
}

// ---- 藍牙序列遙測：只有極少量自身狀態，其餘資料現讀現查其他模組的 get 函式 ----
struct bt_telemetry_state
{
    bool client_connected = false; // SerialBT.hasClient()
    uint32_t publish_count = 0;    // 累計推播次數(除錯用)
};
extern volatile bt_telemetry_state bt_settings; // 實體在 bt_telemetry.cpp

SETTINGS_SCALAR_ACCESSOR(bt, bt_settings, g_bt_mux, client_connected, bool)
SETTINGS_SCALAR_ACCESSOR(bt, bt_settings, g_bt_mux, publish_count, uint32_t)
