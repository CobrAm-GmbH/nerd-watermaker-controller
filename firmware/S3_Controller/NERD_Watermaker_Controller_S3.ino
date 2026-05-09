#include <Arduino.h>
#include <esp_display_panel.hpp>
#include <lvgl.h>
#include "lvgl_v8_port.h"
#include <math.h>
#include <Preferences.h>
#include <HardwareSerial.h>
#include <driver/i2c.h>
#include <esp_err.h>
#include <stdarg.h>

using namespace esp_panel::drivers;
using namespace esp_panel::board;

// --------------------------------------------------
// IMAGE DECLARATIONS
// --------------------------------------------------
LV_IMG_DECLARE(img_prepump);
LV_IMG_DECLARE(img_hppump);
LV_IMG_DECLARE(img_flush);
LV_IMG_DECLARE(img_workflow);

// --------------------------------------------------
// NAV ARROW POSITIONS (absolute)
// keep your tuned values here
// --------------------------------------------------
static const int ARROW_LEFT_X  = 20;
static const int ARROW_LEFT_Y  = 20;
static const int ARROW_RIGHT_X = 765;
static const int ARROW_RIGHT_Y = 20;

// --------------------------------------------------
// GLOBAL STATE / CONFIG
// --------------------------------------------------

static const char *SW_VERSION = "N.E.R.D. v1.1.0";

// --------------------------------------------------
// v1.1.0 Stability Update
// --------------------------------------------------
//
// Main fixes:
// - RS485 UART initialized once only
// - RX buffer flushed before Modbus reads
// - Reduced blocking serial polling
// - Improved main loop timing
// - GUI update throttling added
// - Debug screen updates limited
// - I2C bus speed reduced from 400kHz to 100kHz
//   for improved GT911 touch stability
//
// Result:
// - Prevents random ESP32-S3 resets / INT_WDT
// - Greatly improves RS485 stability
// - System remains operational during GT911 I2C errors
//
// Notes:
// - GT911 touch I2C errors may still occur sporadically
// - Controller logic and outputs continue running normally
// --------------------------------------------------

static bool prepumpOn = false;
static bool hppumpOn = false;
static bool flushOn = false;
static bool tankTestOn = false;

static bool cmdFeedToggle = false;
static bool cmdHppumpToggle = false;
static bool cmdFlushToggle = false;
static bool cmdTankToggle = false;
static bool cmdFaultReset = false;

static bool autoProductionEnabled = false;
static bool autoProductionModeTime = true;   // true = TIME, false = LITERS

static bool autoProductionActive = false;

static uint32_t autoProductionStartRuntimeSec = 0;
static uint32_t autoProductionTargetRuntimeSec = 0;

static uint32_t autoProductionStartLiters_x10 = 0;
static uint32_t autoProductionTargetLiters_x10 = 0;

static int autoProductionTimeHours = 1;      // 1..5
static int autoProductionTargetLiters = 50; // 50..500 step 50

static bool flushCycleActive = false;
static unsigned long flushStartMs = 0;
static int flushTimeSec = 180;   // default 3 minutes
static int flowTrimPct = 100;    // 100% = no trim
static char flowSensorMode = 'A';   // UI selector only: A = F=11*Q, B = F=73*Q

// Runtime counter for H.P. PUMP (persistent)
// Definitive FRAM version: runtime is stored in external FRAM on I2C_NUM_0, addr 0x50.
static Preferences prefs;   // kept only to minimize changes; not used for runtime persistence
static uint32_t hppumpRuntimeSec = 0;
static bool framReady = false;
static bool framInitTried = false;
static unsigned long framInitEarliestMs = 0;

// --- DEBUG PERSISTENT ---
static uint32_t debugBootCount = 0;
static uint32_t debugLastResetReason = 0;
static uint32_t debugHeartbeatCount = 0;
static uint32_t debugLastUptimeSec = 0;

static unsigned long lastPersistentHeartbeatSaveMs = 0;

// --- DEBUG RING BUFFER (RAM ONLY) ---
static const int DEBUG_LOG_LINES = 10;
static const int DEBUG_LOG_LINE_LEN = 96;

static char debugLogBuffer[DEBUG_LOG_LINES][DEBUG_LOG_LINE_LEN];
static int debugLogWriteIndex = 0;
static bool debugLogWrapped = false;

// --- BOOT LOG BUFFER (RAM ONLY) ---
static const int BOOT_LOG_LINES = 30;
static const int BOOT_LOG_LINE_LEN = 96;

static char bootLogBuffer[BOOT_LOG_LINES][BOOT_LOG_LINE_LEN];
static int bootLogWriteIndex = 0;
static bool bootLogWrapped = false;

static unsigned long lastHppumpSecondTick = 0;
static bool runtimeSavePending = false;
static unsigned long runtimeSaveDueMs = 0;
static uint32_t lastDisplayedHoursSec = 0xFFFFFFFFUL;


static uint32_t totalProducedLiters_x10 = 0;
static unsigned long lastLitersSecondTick = 0;
static bool litersSavePending = false;
static unsigned long litersSaveDueMs = 0;
static uint32_t lastDisplayedLitersInt = 0xFFFFFFFFUL;
static float litersFractionAcc = 0.0f;

// --------------------
// USER PIN MAPPING / BOARD BUS MAP
// Waveshare ESP32-S3-Touch-LCD-4.3B RS485 pins from vendor docs:
// GPIO43 = RS485_RXD, GPIO44 = RS485_TXD.
// Flow stays direct because it is a pulse input.
// NOTE: GPIO4 is also used as TP_IRQ on this board, so if the touch and flow
// interfere in testing, move the flow pulse to another free-capable input path.
// --------------------
static const int PIN_FLOW_SENSOR = 4;

// FRAM on external I2C bus, discovered stable as:
// controller = I2C_NUM_0
// address    = 0x50
static const i2c_port_t FRAM_I2C_PORT = I2C_NUM_0;
static const uint8_t FRAM_I2C_ADDR = 0x50;
static const uint16_t FRAM_ADDR_HPPUMP_RUNTIME = 0x0000;
static const uint16_t FRAM_ADDR_TOTAL_LITERS_X10 = 0x0004;
static const uint16_t FRAM_ADDR_DEBUG_BOOT_COUNT       = 0x0008;
static const uint16_t FRAM_ADDR_DEBUG_LAST_RESET       = 0x000C;
static const uint16_t FRAM_ADDR_DEBUG_HEARTBEAT_COUNT  = 0x0010;
static const uint16_t FRAM_ADDR_DEBUG_LAST_UPTIME_SEC  = 0x0014;

// --------------------
// SHARED RS485 MODBUS BUS
// All remote modules sit on the same A/B pair.
// Suggested addresses for this project:
//   Relay board   = 10
//   Analog module = 1
// The relay PDF confirms default 9600 baud and default address 255.
// Eletechsup FAQ confirms the analog module default is address 1 at 9600 8N1.
// --------------------
static const int RS485_UART_RX_PIN = 43;
static const int RS485_UART_TX_PIN = 44; 
static const uint32_t RS485_BAUD = 9600;
static HardwareSerial ModbusSerial(1);

static const uint8_t RELAY_MODBUS_ADDR = 10;   // set relay board to 10 before final install
static const uint8_t ANALOG_MODBUS_ADDR = 1;   // R4IVB02 / R4CVA02 default

static const uint8_t RELAY_CH_FEED  = 0;
static const uint8_t RELAY_CH_HP    = 1;
static const uint8_t RELAY_CH_FLUSH = 2;
static const uint8_t RELAY_CH_TANK  = 3;

// Analog module assumptions isolated here for easy correction after first bus test.
// Public docs confirm function code 03/04 reads but do not expose the register map.
static const bool ANALOG_USE_INPUT_REGS = true;
static const uint16_t ANALOG_REG_PRESSURE = 0x0000; // IN1
static const uint16_t ANALOG_REG_TDS      = 0x0001; // IN2
static const float ANALOG_COUNTS_TO_VOLT  = 0.01f;

// --------------------
// SENSOR / CALIBRATION
// --------------------
static const float PRESSURE_SENSOR_FULL_SCALE_BAR = 100.0f; // 0-5V = 0-100 bar
static const float PRESSURE_SENSOR_FULL_SCALE_V   = 5.0f;
static const float PRESSURE_DIVIDER_RATIO         = 1.5f;   // 5V -> 3.33V, e.g. 10k top / 20k bottom
static const float PRESSURE_TRIP_BAR              = 68.0f;
static const float PRESSURE_RESET_MAX_BAR         = 9.0f;   // allow reset only when pressure is near zero

static const float TDS_OUTPUT_MAX_V               = 2.3f;   // module output max
static const float TDS_MAX_PPM                    = 1000.0f;
static const float TDS_CALIBRATION_FACTOR         = 1.00f;

static const float FLOW_SENSOR_K_HZ_PER_LMIN      = 11.0f;  // F = 11 * Q(L/min)
static const float FLOW_NOFLOW_THRESHOLD_LH       = 10.0f;  // below this = no flow
static const unsigned long FLOW_NOFLOW_GRACE_MS   = 302010UL; // edit this value as desired

// --------------------
// REAL VALUES
// --------------------
static float pressureBar = 0.0f;
static float pressurePsi = 0.0f;

static float flowLhRaw = 0.0f;
static float flowLhFiltered = 0.0f;
static float flowLhTrimmed = 0.0f;
static float flowGph = 0.0f;

static float tdsPpm = 0.0f;

static unsigned long lastPressureReadMs = 0;
static unsigned long lastTdsReadMs = 0;
static unsigned long lastFlowCalcMs = 0;
static unsigned long lastFlowDotUpdate = 0;
static int flowDotAngle = 0;

static volatile uint32_t flowPulseCount = 0;
static unsigned long hpStartedMs = 0;

// --------------------
// ALARMS / FAULTS
// --------------------
enum FaultCode {
    FAULT_NONE = 0,
    FAULT_PRESSURE_HIGH = 1,
    FAULT_NO_FLOW = 2
};

static bool alarmPressureHigh = false;
static bool alarmTdsHigh = false;
static bool alarmNoFlow = false;
static bool faultLatched = false;
static FaultCode activeFault = FAULT_NONE;
static char faultTitle[64] = "SYSTEM ALARM";
static char faultMessage[256] = "No active fault.";


// --------------------------------------------------
// COLORS
// --------------------------------------------------
static const lv_color_t COL_BG        = lv_color_hex(0x161A1D); // Grafana-like gray
static const lv_color_t COL_PANEL     = lv_color_hex(0x1E252B);
static const lv_color_t COL_PANEL_BRD = lv_color_hex(0x2B333B);

static const lv_color_t COL_TOPBAR    = lv_color_hex(0x1565C0);
static const lv_color_t COL_BTN_OFF   = lv_color_hex(0x2E90E5);
static const lv_color_t COL_BTN_ON    = lv_color_hex(0x18B000);
static const lv_color_t COL_BTN_TEST  = lv_color_hex(0xFF8C00);
static const lv_color_t COL_WHITE     = lv_color_hex(0xFFFFFF);
static const lv_color_t COL_SOFT      = lv_color_hex(0xD9E8FF);
static const lv_color_t COL_ARC_BG    = lv_color_hex(0x25313B);
static const lv_color_t COL_ARC_BLUE  = lv_color_hex(0x2196F3);
static const lv_color_t COL_ORANGE    = lv_color_hex(0xFF8C00);
static const lv_color_t COL_RED       = lv_color_hex(0xE53935);
static const lv_color_t COL_GREEN     = lv_color_hex(0x18B000);

// --------------------------------------------------
// SCREEN HANDLES
// --------------------------------------------------
static lv_obj_t *screenMain = nullptr;
static lv_obj_t *screenSettings = nullptr;
static lv_obj_t *screenAlarm = nullptr;
static lv_obj_t *screenProduction = nullptr;
static lv_obj_t *screenDebug = nullptr;

// Bottom buttons per screen [screen][button]
static lv_obj_t *btns[4][4] = {{nullptr}};
static lv_obj_t *btnIcons[4][4] = {{nullptr}};
static lv_obj_t *btnLabels[4][4] = {{nullptr}};

// Gentle backgrounds behind gauges
static lv_obj_t *gaugeBgPressure = nullptr;
static lv_obj_t *gaugeBgFlow = nullptr;
static lv_obj_t *gaugeBgTds = nullptr;

// Main screen gauges
static lv_obj_t *arcFlowOuter = nullptr;
static lv_obj_t *arcFlowInner = nullptr;
static lv_obj_t *flowDot = nullptr;
static lv_obj_t *lblFlowMain = nullptr;
static lv_obj_t *lblFlowSub  = nullptr;

static lv_obj_t *arcPressure = nullptr;
static lv_obj_t *arcPressureOuterGreen  = nullptr;
static lv_obj_t *arcPressureOuterOrange = nullptr;
static lv_obj_t *arcPressureOuterRed    = nullptr;
static lv_obj_t *pressureMark60 = nullptr;
static lv_obj_t *pressureMark70 = nullptr;
static lv_point_t pressureMark60Pts[2];
static lv_point_t pressureMark70Pts[2];
static lv_obj_t *lblPressureMain = nullptr;
static lv_obj_t *lblPressureSub  = nullptr;

static lv_obj_t *arcTds = nullptr;
static lv_obj_t *arcTdsOuterGreen  = nullptr;
static lv_obj_t *arcTdsOuterOrange = nullptr;
static lv_obj_t *arcTdsOuterRed    = nullptr;
static lv_obj_t *tdsMark500 = nullptr;
static lv_obj_t *tdsMark750 = nullptr;
static lv_point_t tdsMark500Pts[2];
static lv_point_t tdsMark750Pts[2];
static lv_obj_t *lblTdsMain = nullptr;
static lv_obj_t *lblTdsSub  = nullptr;

// Settings screen center panels
static lv_obj_t *panelHours = nullptr;
static lv_obj_t *panelFlush = nullptr;
static lv_obj_t *panelTrim  = nullptr;

static lv_obj_t *lblHoursValue = nullptr;
static lv_obj_t *spinnerHours = nullptr;
static lv_obj_t *lblLitersValue = nullptr;

static lv_obj_t *rollerFlushTime = nullptr;

static lv_obj_t *lblTrimValue = nullptr;
static lv_obj_t *lblSensorModeValue = nullptr;
static lv_obj_t *btnSensorA = nullptr;
static lv_obj_t *btnSensorB = nullptr;

static lv_obj_t *alarmLed = nullptr;
static lv_obj_t *lblAlarmStatus = nullptr;
static lv_obj_t *lblFaultTitle = nullptr;
static lv_obj_t *lblFaultMessage = nullptr;

// Production screen center panels
static lv_obj_t *panelProdEnable = nullptr;
static lv_obj_t *panelProdMode   = nullptr;
static lv_obj_t *panelProdTarget = nullptr;

static lv_obj_t *btnProdEnable = nullptr;
static lv_obj_t *lblProdEnableValue = nullptr;
static lv_obj_t *btnProdStart = nullptr;

static lv_obj_t *btnModeTime = nullptr;
static lv_obj_t *btnModeLiters = nullptr;
static lv_obj_t *lblProdModeValue = nullptr;

static lv_obj_t *btnTargetMinus = nullptr;
static lv_obj_t *btnTargetPlus = nullptr;
static lv_obj_t *lblProdTargetValue = nullptr;
static lv_obj_t *lblProdCountdown = nullptr;

// Debug screen panels
static lv_obj_t *panelDebugSystem = nullptr;
static lv_obj_t *panelDebugMemory = nullptr;
static lv_obj_t *panelDebugEvents = nullptr;

static lv_obj_t *lblDebugSystemValue = nullptr;
static lv_obj_t *lblDebugMemoryValue = nullptr;
static lv_obj_t *lblDebugEventsValue = nullptr;

static lv_obj_t *panelBootLog = nullptr;
static lv_obj_t *lblBootLogValue = nullptr;

// --------------------------------------------------
// FORWARD DECLARATIONS
// --------------------------------------------------
static void update_all_button_visuals();
static void update_alarm_indicators();
static void show_main_screen();
static void show_settings_screen();
static void show_alarm_screen();
static void schedule_runtime_save(uint32_t delayMs);
static void process_pending_runtime_save();
static void apply_outputs();
static void safe_shutdown_outputs();
static void update_flow_sensor_selector_panel();
static void schedule_liters_save(uint32_t delayMs);
static void process_pending_liters_save();
static void update_total_liters();
static void update_liters_label();
static void show_production_screen();
static void create_production_screen();

static void show_debug_screen();
static void create_debug_screen();
static void update_debug_screen();

static void boot_log(const char *msg);
static void boot_logf(const char *fmt, ...);

static void update_production_screen_visuals();
static void prod_start_cb(lv_event_t *e);
static void prod_enable_cb(lv_event_t *e);
static void prod_mode_time_cb(lv_event_t *e);
static void prod_mode_liters_cb(lv_event_t *e);
static void prod_target_minus_cb(lv_event_t *e);
static void prod_target_plus_cb(lv_event_t *e);
static void process_auto_production_logic();

static void debug_load_persistent_info();
static void debug_save_boot_info();
static void debug_save_heartbeat_persistent();
static const char* reset_reason_to_string(esp_reset_reason_t reason);
static void debug_print_boot_report();

static void debug_log(const char *msg);
static void debug_logf(const char *fmt, ...);

// --------------------------------------------------
// HELPERS
// --------------------------------------------------
static void style_button(lv_obj_t *btn, lv_color_t color)
{
    lv_obj_set_style_bg_color(btn, color, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(btn, COL_WHITE, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(btn, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_radius(btn, 12, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(btn, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
}

static lv_obj_t *make_button_bg(lv_obj_t *parent, int x, int y, int w, int h, lv_event_cb_t cb, void *user_data)
{
    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_set_pos(btn, x, y);
    lv_obj_set_size(btn, w, h);
    style_button(btn, COL_BTN_OFF);
    lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, user_data);
    return btn;
}

static lv_obj_t *make_button_icon(lv_obj_t *parent, const void *src, int x, int y)
{
    lv_obj_t *img = lv_img_create(parent);
    lv_img_set_src(img, src);
    lv_obj_set_pos(img, x, y);
    lv_obj_clear_flag(img, LV_OBJ_FLAG_CLICKABLE);
    return img;
}

static lv_obj_t *make_button_text(lv_obj_t *parent, const char *txt, int x, int y, int w)
{
    lv_obj_t *lbl = lv_label_create(parent);
    lv_label_set_text(lbl, txt);
    lv_obj_set_width(lbl, w);
    lv_obj_set_pos(lbl, x, y);
    lv_obj_set_style_text_color(lbl, COL_WHITE, 0);
    lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_16, 0);
    lv_label_set_long_mode(lbl, LV_LABEL_LONG_WRAP);
    lv_obj_clear_flag(lbl, LV_OBJ_FLAG_CLICKABLE);
    return lbl;
}

static lv_obj_t *create_gauge_background(lv_obj_t *parent, int x, int y, int size)
{
    lv_obj_t *bg = lv_obj_create(parent);
    lv_obj_set_size(bg, size, size);
    lv_obj_set_pos(bg, x, y);

    lv_obj_set_style_radius(bg, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(bg, COL_PANEL, 0);
    lv_obj_set_style_bg_opa(bg, 180, 0);
    lv_obj_set_style_border_color(bg, COL_PANEL_BRD, 0);
    lv_obj_set_style_border_width(bg, 1, 0);
    lv_obj_set_style_shadow_width(bg, 0, 0);
    lv_obj_set_style_pad_all(bg, 0, 0);
    lv_obj_clear_flag(bg, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(bg, LV_OBJ_FLAG_CLICKABLE);
    return bg;
}

static lv_obj_t *create_panel(lv_obj_t *parent, int x, int y, int w, int h)
{
    lv_obj_t *panel = lv_obj_create(parent);
    lv_obj_set_pos(panel, x, y);
    lv_obj_set_size(panel, w, h);

    lv_obj_set_style_bg_color(panel, COL_PANEL, 0);
    lv_obj_set_style_bg_opa(panel, 200, 0);
    lv_obj_set_style_border_color(panel, COL_PANEL_BRD, 0);
    lv_obj_set_style_border_width(panel, 1, 0);
    lv_obj_set_style_radius(panel, 12, 0);
    lv_obj_set_style_shadow_width(panel, 0, 0);
    lv_obj_set_style_pad_all(panel, 0, 0);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);

    return panel;
}

static lv_obj_t *create_gauge_base(lv_obj_t *parent, int x, int y, int size, int minVal, int maxVal, int value)
{
    lv_obj_t *arc = lv_arc_create(parent);
    lv_obj_set_size(arc, size, size);
    lv_obj_set_pos(arc, x, y);

    lv_arc_set_rotation(arc, 135);
    lv_arc_set_bg_angles(arc, 0, 270);
    lv_arc_set_range(arc, minVal, maxVal);
    lv_arc_set_value(arc, value);

    lv_obj_remove_style(arc, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_set_style_arc_width(arc, 10, LV_PART_MAIN);
    lv_obj_set_style_arc_color(arc, COL_ARC_BG, LV_PART_MAIN);

    lv_obj_set_style_arc_width(arc, 10, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(arc, COL_ARC_BLUE, LV_PART_INDICATOR);

    return arc;
}

static lv_obj_t *create_arc_segment(lv_obj_t *parent, int x, int y, int size, int startAngle, int endAngle, lv_color_t color, int width)
{
    lv_obj_t *arc = lv_arc_create(parent);
    lv_obj_set_size(arc, size, size);
    lv_obj_set_pos(arc, x, y);

    lv_arc_set_rotation(arc, 135);
    lv_arc_set_bg_angles(arc, startAngle, endAngle);
    lv_arc_set_range(arc, 0, 100);
    lv_arc_set_value(arc, 100);

    lv_obj_remove_style(arc, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_set_style_arc_opa(arc, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_arc_width(arc, width, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(arc, color, LV_PART_INDICATOR);

    return arc;
}

static lv_obj_t *create_threshold_mark(lv_obj_t *parent, lv_point_t pts[2], int cx, int cy, int rOuter, int rInner, int valuePercent)
{
    float angleDeg = 135.0f + (valuePercent * 270.0f / 100.0f);
    float rad = angleDeg * 3.14159265359f / 180.0f;

    pts[0].x = (lv_coord_t)roundf(cx + cosf(rad) * rOuter);
    pts[0].y = (lv_coord_t)roundf(cy + sinf(rad) * rOuter);
    pts[1].x = (lv_coord_t)roundf(cx + cosf(rad) * rInner);
    pts[1].y = (lv_coord_t)roundf(cy + sinf(rad) * rInner);

    lv_obj_t *line = lv_line_create(parent);
    lv_line_set_points(line, pts, 2);
    lv_obj_set_style_line_color(line, COL_WHITE, 0);
    lv_obj_set_style_line_width(line, 3, 0);
    lv_obj_clear_flag(line, LV_OBJ_FLAG_CLICKABLE);
    return line;
}

// FLOW helpers
static lv_obj_t *create_full_ring(lv_obj_t *parent, int x, int y, int size, int width, lv_color_t color, lv_opa_t opa)
{
    lv_obj_t *arc = lv_arc_create(parent);
    lv_obj_set_size(arc, size, size);
    lv_obj_set_pos(arc, x, y);

    lv_arc_set_rotation(arc, 0);
    lv_arc_set_bg_angles(arc, 0, 359);
    lv_arc_set_range(arc, 0, 100);
    lv_arc_set_value(arc, 100);

    lv_obj_remove_style(arc, NULL, LV_PART_KNOB);
    lv_obj_clear_flag(arc, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_set_style_arc_width(arc, width, LV_PART_MAIN);
    lv_obj_set_style_arc_color(arc, color, LV_PART_MAIN);
    lv_obj_set_style_arc_opa(arc, opa, LV_PART_MAIN);

    lv_obj_set_style_arc_opa(arc, LV_OPA_TRANSP, LV_PART_INDICATOR);

    return arc;
}

static lv_obj_t *create_flow_dot(lv_obj_t *parent, int size)
{
    lv_obj_t *dot = lv_obj_create(parent);
    lv_obj_set_size(dot, size, size);
    lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(dot, COL_WHITE, 0);
    lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(dot, 0, 0);
    lv_obj_set_style_shadow_width(dot, 0, 0);
    lv_obj_clear_flag(dot, LV_OBJ_FLAG_CLICKABLE);
    return dot;
}

static void set_flow_dot_position(int cx, int cy, int radius, int angleDeg)
{
    float rad = angleDeg * 3.14159265359f / 180.0f;
    int dotSize = lv_obj_get_width(flowDot);

    int x = (int)roundf(cx + cosf(rad) * radius) - dotSize / 2;
    int y = (int)roundf(cy + sinf(rad) * radius) - dotSize / 2;

    lv_obj_set_pos(flowDot, x, y);
}

static lv_obj_t *create_alarm_led(lv_obj_t *parent, int x, int y, int size)
{
    lv_obj_t *led = lv_obj_create(parent);
    lv_obj_set_size(led, size, size);
    lv_obj_set_pos(led, x, y);
    lv_obj_set_style_radius(led, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(led, lv_color_hex(0x4A5560), 0);
    lv_obj_set_style_bg_opa(led, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(led, COL_WHITE, 0);
    lv_obj_set_style_border_width(led, 2, 0);
    lv_obj_set_style_shadow_width(led, 0, 0);
    lv_obj_clear_flag(led, LV_OBJ_FLAG_CLICKABLE);
    return led;
}

static lv_obj_t *create_alarm_status_label(lv_obj_t *parent, int x, int y)
{
    lv_obj_t *lbl = lv_label_create(parent);
    lv_label_set_text(lbl, "SYSTEM OK");
    lv_obj_set_pos(lbl, x, y);
    lv_obj_set_style_text_color(lbl, COL_WHITE, 0);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_14, 0);
    return lbl;
}

static bool fram_write_u32(uint16_t memAddr, uint32_t value)
{
    uint8_t buf[6];
    buf[0] = (uint8_t)(memAddr >> 8);
    buf[1] = (uint8_t)(memAddr & 0xFF);
    buf[2] = (uint8_t)(value & 0xFF);
    buf[3] = (uint8_t)((value >> 8) & 0xFF);
    buf[4] = (uint8_t)((value >> 16) & 0xFF);
    buf[5] = (uint8_t)((value >> 24) & 0xFF);

    esp_err_t err = i2c_master_write_to_device(
        FRAM_I2C_PORT,
        FRAM_I2C_ADDR,
        buf,
        sizeof(buf),
        pdMS_TO_TICKS(50)
    );

    return err == ESP_OK;
}

static bool fram_read_u32(uint16_t memAddr, uint32_t *valueOut)
{
    if (!valueOut) return false;

    uint8_t addrBuf[2];
    addrBuf[0] = (uint8_t)(memAddr >> 8);
    addrBuf[1] = (uint8_t)(memAddr & 0xFF);

    uint8_t data[4] = {0, 0, 0, 0};

    esp_err_t err = i2c_master_write_read_device(
        FRAM_I2C_PORT,
        FRAM_I2C_ADDR,
        addrBuf,
        2,
        data,
        4,
        pdMS_TO_TICKS(50)
    );
    if (err != ESP_OK) return false;

    *valueOut =
        (uint32_t)data[0] |
        ((uint32_t)data[1] << 8) |
        ((uint32_t)data[2] << 16) |
        ((uint32_t)data[3] << 24);

    return true;
}

static bool fram_probe()
{
    uint8_t addrBuf[2] = {
        (uint8_t)(FRAM_ADDR_HPPUMP_RUNTIME >> 8),
        (uint8_t)(FRAM_ADDR_HPPUMP_RUNTIME & 0xFF)
    };
    uint8_t data[1] = {0};

    esp_err_t err = i2c_master_write_read_device(
        FRAM_I2C_PORT,
        FRAM_I2C_ADDR,
        addrBuf,
        2,
        data,
        1,
        pdMS_TO_TICKS(30)
    );

    return err == ESP_OK;
}

static void fram_try_init_deferred()
{
    if (framInitTried) return;
    if ((long)(millis() - framInitEarliestMs) < 0) return;

    framInitTried = true;

    framReady = fram_probe();

    if (framReady) {
    debug_log("FRAM READY");
    boot_log("FRAM READY");
} else {
    debug_log("FRAM FAIL");
    boot_log("FRAM FAIL");
}

    if (framReady) {
        uint32_t storedRuntime = 0;
        if (fram_read_u32(FRAM_ADDR_HPPUMP_RUNTIME, &storedRuntime)) {
            hppumpRuntimeSec = storedRuntime;
        } else {
            framReady = false;
        }

        if (framReady) {
            uint32_t storedLiters_x10 = 0;
            if (fram_read_u32(FRAM_ADDR_TOTAL_LITERS_X10, &storedLiters_x10)) {
                totalProducedLiters_x10 = storedLiters_x10;
            } else {
                framReady = false;
            }
        }

        if (framReady) {
            debug_load_persistent_info();
            debug_save_boot_info();
            debug_print_boot_report();
        }
    }
}

static const char* reset_reason_to_string(esp_reset_reason_t reason)
{
    switch (reason) {
        case ESP_RST_UNKNOWN:   return "UNKNOWN";
        case ESP_RST_POWERON:   return "POWERON";
        case ESP_RST_EXT:       return "EXTERNAL";
        case ESP_RST_SW:        return "SOFTWARE";
        case ESP_RST_PANIC:     return "PANIC";
        case ESP_RST_INT_WDT:   return "INT_WDT";
        case ESP_RST_TASK_WDT:  return "TASK_WDT";
        case ESP_RST_WDT:       return "WDT";
        case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
        case ESP_RST_BROWNOUT:  return "BROWNOUT";
        case ESP_RST_SDIO:      return "SDIO";
        default:                return "OTHER";
    }
}

static void debug_load_persistent_info()
{
    if (!framReady) return;

    fram_read_u32(FRAM_ADDR_DEBUG_BOOT_COUNT,      &debugBootCount);
    fram_read_u32(FRAM_ADDR_DEBUG_LAST_RESET,      &debugLastResetReason);
    fram_read_u32(FRAM_ADDR_DEBUG_HEARTBEAT_COUNT, &debugHeartbeatCount);
    fram_read_u32(FRAM_ADDR_DEBUG_LAST_UPTIME_SEC, &debugLastUptimeSec);

    // Se i dati sono chiaramente sporchi / non inizializzati, li azzeriamo
    if (debugBootCount > 1000000UL ||
        debugHeartbeatCount > 1000000UL ||
        debugLastUptimeSec > 31536000UL)   // > 1 anno di uptime non ha senso
    {
        debugBootCount = 0;
        debugLastResetReason = 0;
        debugHeartbeatCount = 0;
        debugLastUptimeSec = 0;

        fram_write_u32(FRAM_ADDR_DEBUG_BOOT_COUNT,      debugBootCount);
        fram_write_u32(FRAM_ADDR_DEBUG_LAST_RESET,      debugLastResetReason);
        fram_write_u32(FRAM_ADDR_DEBUG_HEARTBEAT_COUNT, debugHeartbeatCount);
        fram_write_u32(FRAM_ADDR_DEBUG_LAST_UPTIME_SEC, debugLastUptimeSec);
    }
}

static void debug_save_boot_info()
{
    if (!framReady) return;

    esp_reset_reason_t reason = esp_reset_reason();

    debugBootCount++;
    debugLastResetReason = (uint32_t)reason;

    fram_write_u32(FRAM_ADDR_DEBUG_BOOT_COUNT, debugBootCount);
    fram_write_u32(FRAM_ADDR_DEBUG_LAST_RESET, debugLastResetReason);
}

static void debug_save_heartbeat_persistent()
{
    if (!framReady) return;

    unsigned long now = millis();
    if ((now - lastPersistentHeartbeatSaveMs) < 60000UL) return;
    lastPersistentHeartbeatSaveMs = now;

    debugHeartbeatCount++;
    debugLastUptimeSec = now / 1000UL;

    fram_write_u32(FRAM_ADDR_DEBUG_HEARTBEAT_COUNT, debugHeartbeatCount);
    fram_write_u32(FRAM_ADDR_DEBUG_LAST_UPTIME_SEC, debugLastUptimeSec);
}

static void debug_print_boot_report()
{
    esp_reset_reason_t reason = esp_reset_reason();

    Serial.println();
    Serial.println("========================================");
    Serial.println("N.E.R.D. PERSISTENT DEBUG REPORT");
    Serial.printf("Current reset reason: %s (%d)\n",
              reset_reason_to_string(reason),
              (int)reason);
    Serial.printf("Persistent boot count: %lu\n", (unsigned long)debugBootCount);
    Serial.printf("Persistent last reset reason: %s (%lu)\n",
              reset_reason_to_string((esp_reset_reason_t)debugLastResetReason),
              (unsigned long)debugLastResetReason);
    Serial.printf("Persistent heartbeat count: %lu\n", (unsigned long)debugHeartbeatCount);
    Serial.printf("Persistent last uptime: %lu s\n", (unsigned long)debugLastUptimeSec);
    Serial.printf("Free heap: %lu bytes\n", (unsigned long)ESP.getFreeHeap());
    Serial.printf("Free PSRAM: %lu bytes\n", (unsigned long)ESP.getFreePsram());
    Serial.println("========================================");
    Serial.println();
    debug_log("BOOT READY");
    boot_logf("RESET: %s (%d)", reset_reason_to_string(reason), (int)reason);
boot_log("BOOT READY");
}

static void debug_log(const char *msg)
{
    if (!msg) return;

    snprintf(debugLogBuffer[debugLogWriteIndex],
             DEBUG_LOG_LINE_LEN,
             "%s",
             msg);

    debugLogWriteIndex++;
    if (debugLogWriteIndex >= DEBUG_LOG_LINES) {
        debugLogWriteIndex = 0;
        debugLogWrapped = true;
    }

    Serial.printf("[DBG] %s\n", msg);
}

static void debug_logf(const char *fmt, ...)
{
    if (!fmt) return;

    char temp[DEBUG_LOG_LINE_LEN];

    va_list args;
    va_start(args, fmt);
    vsnprintf(temp, sizeof(temp), fmt, args);
    va_end(args);

    debug_log(temp);
}

static void boot_log(const char *msg)
{
    if (!msg) return;

    snprintf(bootLogBuffer[bootLogWriteIndex],
             BOOT_LOG_LINE_LEN,
             "%s",
             msg);

    bootLogWriteIndex++;
    if (bootLogWriteIndex >= BOOT_LOG_LINES) {
        bootLogWriteIndex = 0;
        bootLogWrapped = true;
    }

    Serial.printf("[BOOT] %s\n", msg);
}

static void boot_logf(const char *fmt, ...)
{
    if (!fmt) return;

    char temp[BOOT_LOG_LINE_LEN];

    va_list args;
    va_start(args, fmt);
    vsnprintf(temp, sizeof(temp), fmt, args);
    va_end(args);

    boot_log(temp);
}

static uint16_t modbus_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)data[pos];
        for (int i = 0; i < 8; i++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

static bool modbus_send_request(const uint8_t *frame, size_t len)
{
    ModbusSerial.flush();
    while (ModbusSerial.available()) ModbusSerial.read();
    ModbusSerial.write(frame, len);
    ModbusSerial.flush();
    return true;
}

static int modbus_read_response(uint8_t *buf, size_t maxLen, uint32_t timeoutMs)
{
    size_t idx = 0;
    unsigned long startMs = millis();
    while ((millis() - startMs) < timeoutMs) {
        while (ModbusSerial.available() && idx < maxLen) {
            buf[idx++] = (uint8_t)ModbusSerial.read();
        }
        if (idx >= 5) {
            delay(4);
            while (ModbusSerial.available() && idx < maxLen) {
                buf[idx++] = (uint8_t)ModbusSerial.read();
            }
            return (int)idx;
        }
        delay(1);
    }
    return (int)idx;
}

static bool modbus_check_crc(const uint8_t *buf, size_t len)
{
    if (len < 4) return false;
    uint16_t rx = (uint16_t)buf[len - 2] | ((uint16_t)buf[len - 1] << 8);
    uint16_t calc = modbus_crc16(buf, len - 2);
    return rx == calc;
}

static bool modbus_read_register_u16(uint8_t devAddr, uint8_t fn, uint16_t reg, uint16_t *valueOut)
{
    if (!valueOut) return false;

    uint8_t frame[8];
    frame[0] = devAddr;
    frame[1] = fn;
    frame[2] = (uint8_t)(reg >> 8);
    frame[3] = (uint8_t)(reg & 0xFF);
    frame[4] = 0x00;
    frame[5] = 0x01;

    uint16_t crc = modbus_crc16(frame, 6);
    frame[6] = (uint8_t)(crc & 0xFF);
    frame[7] = (uint8_t)(crc >> 8);

    modbus_send_request(frame, sizeof(frame));

    uint8_t resp[16];
    int n = modbus_read_response(resp, sizeof(resp), 80);

    if (n < 7) return false;
    if (!modbus_check_crc(resp, (size_t)n)) return false;
    if (resp[0] != devAddr) return false;
    if (resp[1] != fn) return false;
    if (resp[2] != 0x02) return false;

    *valueOut = ((uint16_t)resp[3] << 8) | resp[4];
    return true;
}

static bool modbus_read_register_u16_retry(uint8_t devAddr, uint8_t fn, uint16_t reg, uint16_t *valueOut)
{
    // First attempt
    if (modbus_read_register_u16(devAddr, fn, reg, valueOut)) {
        return true;
    }

    // Small delay before retry (bus settling)
    delay(15);

    // Second attempt
    return modbus_read_register_u16(devAddr, fn, reg, valueOut);
}

static bool modbus_write_single_coil(uint8_t devAddr, uint16_t coil, bool on)
{
    uint8_t frame[8];
    frame[0] = devAddr;
    frame[1] = 0x05;
    frame[2] = (uint8_t)(coil >> 8);
    frame[3] = (uint8_t)(coil & 0xFF);
    frame[4] = on ? 0xFF : 0x00;
    frame[5] = 0x00;
    uint16_t crc = modbus_crc16(frame, 6);
    frame[6] = (uint8_t)(crc & 0xFF);
    frame[7] = (uint8_t)(crc >> 8);

    modbus_send_request(frame, sizeof(frame));

    uint8_t resp[16];
    int n = modbus_read_response(resp, sizeof(resp), 80);
    if (n != 8) return false;
    if (!modbus_check_crc(resp, (size_t)n)) return false;
    for (int i = 0; i < 6; ++i) {
        if (resp[i] != frame[i]) return false;
    }
    return true;
}

static bool modbus_force_all_relays_off(uint8_t devAddr)
{
    uint8_t frame[10];
    frame[0] = devAddr;
    frame[1] = 0x0F;
    frame[2] = 0x00;
    frame[3] = 0x00;
    frame[4] = 0x00;
    frame[5] = 0x08;
    frame[6] = 0x01;
    frame[7] = 0x00;
    uint16_t crc = modbus_crc16(frame, 8);
    frame[8] = (uint8_t)(crc & 0xFF);
    frame[9] = (uint8_t)(crc >> 8);

    modbus_send_request(frame, sizeof(frame));

    uint8_t resp[16];
    int n = modbus_read_response(resp, sizeof(resp), 80);
    if (n != 8) return false;
    if (!modbus_check_crc(resp, (size_t)n)) return false;
    return resp[0] == devAddr && resp[1] == 0x0F;
}

// --------------------------------------------------
// RS485 FRESH INIT (fix stabile Waveshare)
// --------------------------------------------------
static void rs485_begin_fresh()
{
    static bool rs485Started = false;

    if (!rs485Started) {
        ModbusSerial.end();
        delay(20);

        ModbusSerial.begin(RS485_BAUD, SERIAL_8N1, RS485_UART_RX_PIN, RS485_UART_TX_PIN);
        ModbusSerial.setRxBufferSize(256);

        delay(50);
        rs485Started = true;
    }

    while (ModbusSerial.available()) {
        ModbusSerial.read();
    }
}

// --------------------------------------------------
// ANALOG READ STABILE (FIX DEFINITIVO)
// --------------------------------------------------
static bool analog_read_voltage_rs485(uint16_t reg, float *voltageOut)
{
    if (!voltageOut) return false;

    // 👉 il segreto: reset bus prima di ogni lettura
    rs485_begin_fresh();

    uint8_t fn = ANALOG_USE_INPUT_REGS ? 0x04 : 0x03;

    // Frame Modbus
    uint8_t frame[8];
    frame[0] = ANALOG_MODBUS_ADDR;
    frame[1] = fn;
    frame[2] = (uint8_t)(reg >> 8);
    frame[3] = (uint8_t)(reg & 0xFF);
    frame[4] = 0x00;
    frame[5] = 0x01;

    uint16_t crc = modbus_crc16(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = crc >> 8;

    // TX
    while (ModbusSerial.available()) ModbusSerial.read();
    delay(5);

    ModbusSerial.write(frame, 8);
    ModbusSerial.flush();

    // RX
    uint8_t resp[16];
    size_t received = 0;
    uint32_t start = millis();
    uint32_t lastByte = 0;

    while ((millis() - start) < 120) {
        while (ModbusSerial.available() && received < sizeof(resp)) {
            resp[received++] = ModbusSerial.read();
            lastByte = millis();
        }

        if (received >= 7) break;

        if (received > 0 && (millis() - lastByte) > 20) break;

        delay(1);
    }

    // VALIDAZIONE
    if (received < 7) return false;
    if (resp[0] != ANALOG_MODBUS_ADDR) return false;
    if (resp[1] != fn) return false;
    if (resp[2] != 0x02) return false;

    uint16_t crcCalc = modbus_crc16(resp, received - 2);
    uint16_t crcRecv = resp[received - 2] | (resp[received - 1] << 8);

    if (crcCalc != crcRecv) return false;

    uint16_t raw = ((uint16_t)resp[3] << 8) | resp[4];

    *voltageOut = raw * ANALOG_COUNTS_TO_VOLT;

    return true;
}

static void relay_write_channel(uint8_t channel, bool on)
{
    if (modbus_write_single_coil(RELAY_MODBUS_ADDR, channel, on)) return;

    delay(20);
    modbus_write_single_coil(RELAY_MODBUS_ADDR, channel, on);
}

static void relay_all_off()
{
    modbus_force_all_relays_off(RELAY_MODBUS_ADDR);
}

static void IRAM_ATTR flow_pulse_isr()
{
    flowPulseCount++;
}

static void init_hardware()
{
    pinMode(PIN_FLOW_SENSOR, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_FLOW_SENSOR), flow_pulse_isr, RISING);

    ModbusSerial.begin(RS485_BAUD, SERIAL_8N1, RS485_UART_RX_PIN, RS485_UART_TX_PIN);
    delay(80);
    relay_all_off();
}

static void apply_outputs()
{
    static bool lastFeed  = false;
    static bool lastHp    = false;
    static bool lastFlush = false;
    static bool lastTank  = false;
    static bool firstRun  = true;

    if (firstRun || prepumpOn != lastFeed) {
        relay_write_channel(RELAY_CH_FEED, prepumpOn);
        lastFeed = prepumpOn;
    }

    if (firstRun || hppumpOn != lastHp) {
        relay_write_channel(RELAY_CH_HP, hppumpOn);
        lastHp = hppumpOn;
    }

    if (firstRun || flushOn != lastFlush) {
        relay_write_channel(RELAY_CH_FLUSH, flushOn);
        lastFlush = flushOn;
    }

    if (firstRun || tankTestOn != lastTank) {
        relay_write_channel(RELAY_CH_TANK, tankTestOn);
        lastTank = tankTestOn;
    }

    firstRun = false;
}

static void safe_shutdown_outputs()
{
    debug_log("SAFE SHUTDOWN");
    prepumpOn = false;
    hppumpOn = false;
    flushOn = false;
    flushCycleActive = false;
    autoProductionActive = false;
    relay_all_off();
    schedule_runtime_save(300UL);
    schedule_liters_save(300UL);
}


// --------------------------------------------------
// NAVIGATION
// --------------------------------------------------
static void nav_to_settings_cb(lv_event_t *e)
{
    (void)e;
    show_settings_screen();
}

static void nav_to_production_cb(lv_event_t *e)
{
    (void)e;
    show_production_screen();
}

static void nav_to_main_cb(lv_event_t *e)
{
    (void)e;
    show_main_screen();
}

static void nav_to_settings_from_production_cb(lv_event_t *e)
{
    (void)e;
    show_settings_screen();
}

static void nav_to_debug_cb(lv_event_t *e)
{
    (void)e;
    show_debug_screen();
}

static void nav_to_production_from_debug_cb(lv_event_t *e)
{
    (void)e;
    show_production_screen();
}

static lv_obj_t *create_header(
    lv_obj_t *parent,
    bool leftArrow,
    bool rightArrow,
    lv_event_cb_t leftCb,
    lv_event_cb_t rightCb
)
{
    lv_obj_t *top = lv_obj_create(parent);
    lv_obj_set_size(top, 800, 60);
    lv_obj_set_pos(top, 0, 0);

    lv_obj_set_style_bg_color(top, COL_TOPBAR, 0);
    lv_obj_set_style_bg_opa(top, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(top, 0, 0);
    lv_obj_set_style_radius(top, 0, 0);
    lv_obj_set_style_pad_all(top, 0, 0);
    lv_obj_clear_flag(top, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(top);
    lv_label_set_text(title, "N.E.R.D. - Watermaker Controller");
    lv_obj_set_style_text_color(title, COL_WHITE, 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_align(title, LV_ALIGN_CENTER, 0, 0);

    if (leftArrow) {
        lv_obj_t *btn = lv_obj_create(top);
        lv_obj_set_pos(btn, 0, 0);
        lv_obj_set_size(btn, 60, 60);
        lv_obj_set_style_bg_opa(btn, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(btn, 0, 0);
        lv_obj_add_flag(btn, LV_OBJ_FLAG_CLICKABLE);

        if (leftCb) {
            lv_obj_add_event_cb(btn, leftCb, LV_EVENT_CLICKED, NULL);
        }

        lv_obj_t *lbl = lv_label_create(top);
        lv_label_set_text(lbl, LV_SYMBOL_LEFT);
        lv_obj_set_style_text_color(lbl, COL_WHITE, 0);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_20, 0);
        lv_obj_set_pos(lbl, ARROW_LEFT_X, ARROW_LEFT_Y);
        lv_obj_clear_flag(lbl, LV_OBJ_FLAG_CLICKABLE);
    }

    if (rightArrow) {
        lv_obj_t *btn = lv_obj_create(top);
        lv_obj_set_pos(btn, 740, 0);
        lv_obj_set_size(btn, 60, 60);
        lv_obj_set_style_bg_opa(btn, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(btn, 0, 0);
        lv_obj_add_flag(btn, LV_OBJ_FLAG_CLICKABLE);

        if (rightCb) {
            lv_obj_add_event_cb(btn, rightCb, LV_EVENT_CLICKED, NULL);
        }

        lv_obj_t *lbl = lv_label_create(top);
        lv_label_set_text(lbl, LV_SYMBOL_RIGHT);
        lv_obj_set_style_text_color(lbl, COL_WHITE, 0);
        lv_obj_set_style_text_font(lbl, &lv_font_montserrat_20, 0);
        lv_obj_set_pos(lbl, ARROW_RIGHT_X, ARROW_RIGHT_Y);
        lv_obj_clear_flag(lbl, LV_OBJ_FLAG_CLICKABLE);
    }

    return top;
}

// --------------------------------------------------
// BUTTON STATE SYNC
// --------------------------------------------------
enum ButtonIndex {
    BTN_FEED = 0,
    BTN_HPPUMP = 1,
    BTN_FLUSH = 2,
    BTN_TANK = 3
};

static void update_spinner_state()
{
    if (spinnerHours) {
        if (hppumpOn) lv_obj_clear_flag(spinnerHours, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(spinnerHours, LV_OBJ_FLAG_HIDDEN);
    }
}

static void update_flow_trim_panel()
{
    if (lblTrimValue) {
        lv_label_set_text_fmt(lblTrimValue, "%d%%", flowTrimPct);
    }
}

static void update_flow_sensor_selector_panel()
{
    if (lblSensorModeValue) {
        if (flowSensorMode == 'A') {
            lv_label_set_text(lblSensorModeValue, "SENSOR A  (F = 11 x Q)");
        } else {
            lv_label_set_text(lblSensorModeValue, "SENSOR B  (F = 73 x Q)");
        }
    }

    if (btnSensorA) {
        style_button(btnSensorA, (flowSensorMode == 'A') ? COL_BTN_ON : COL_BTN_OFF);
    }
    if (btnSensorB) {
        style_button(btnSensorB, (flowSensorMode == 'B') ? COL_BTN_ON : COL_BTN_OFF);
    }
}

static void update_production_screen_visuals()
{
    if (btnProdEnable) {
        style_button(btnProdEnable, autoProductionEnabled ? COL_BTN_ON : COL_BTN_OFF);
    }

    if (btnProdStart) {
        // --- Stato visuale bottone ---
        if (!autoProductionEnabled || !hppumpOn) {
            style_button(btnProdStart, COL_BTN_OFF);
            lv_obj_set_style_bg_opa(btnProdStart, LV_OPA_50, 0);
        } else if (autoProductionActive) {
            style_button(btnProdStart, COL_BTN_ON);
            lv_obj_set_style_bg_opa(btnProdStart, LV_OPA_COVER, 0);
        } else {
            style_button(btnProdStart, COL_BTN_TEST);
            lv_obj_set_style_bg_opa(btnProdStart, LV_OPA_COVER, 0);
        }

        // --- Testo START / STOP ---
        lv_obj_t *child = lv_obj_get_child(btnProdStart, 0);
        if (child) {
            if (autoProductionActive) {
                lv_label_set_text(child, "STOP AUTO");
            } else {
                lv_label_set_text(child, "START AUTO");
            }
        }
    }

    if (lblProdEnableValue) {
        if (!autoProductionEnabled) {
            lv_label_set_text(lblProdEnableValue, "DISABLED");
        } else if (autoProductionActive) {
            lv_label_set_text(lblProdEnableValue, "RUNNING");
        } else {
            lv_label_set_text(lblProdEnableValue, "READY");
        }
    }

    if (btnModeTime) {
        style_button(btnModeTime, autoProductionModeTime ? COL_BTN_ON : COL_BTN_OFF);
    }

    if (btnModeLiters) {
        style_button(btnModeLiters, autoProductionModeTime ? COL_BTN_OFF : COL_BTN_ON);
    }

    if (lblProdModeValue) {
        lv_label_set_text(lblProdModeValue, autoProductionModeTime ? "TIME MODE" : "LITERS MODE");
    }

    if (lblProdTargetValue) {
        if (autoProductionModeTime) {
            lv_label_set_text_fmt(lblProdTargetValue, "%d HOUR%s",
                                  autoProductionTimeHours,
                                  (autoProductionTimeHours == 1) ? "" : "S");
        } else {
            lv_label_set_text_fmt(lblProdTargetValue, "%d L", autoProductionTargetLiters);
        }
    }

    if (lblProdCountdown) {
        if (!autoProductionActive) {
            if (autoProductionModeTime) {
                lv_label_set_text(lblProdCountdown, "TTG --:--:--");
            } else {
                lv_label_set_text(lblProdCountdown, "LTG --- L");
            }
        } else {
            if (autoProductionModeTime) {
                uint32_t remainingSec = 0;
                if (autoProductionTargetRuntimeSec > hppumpRuntimeSec) {
                    remainingSec = autoProductionTargetRuntimeSec - hppumpRuntimeSec;
                }

                uint32_t hh = remainingSec / 3600UL;
                uint32_t mm = (remainingSec % 3600UL) / 60UL;
                uint32_t ss = remainingSec % 60UL;

                lv_label_set_text_fmt(lblProdCountdown, "TTG %02lu:%02lu:%02lu",
                                      (unsigned long)hh,
                                      (unsigned long)mm,
                                      (unsigned long)ss);
            } else {
                uint32_t remaining_x10 = 0;
                if (autoProductionTargetLiters_x10 > totalProducedLiters_x10) {
                    remaining_x10 = autoProductionTargetLiters_x10 - totalProducedLiters_x10;
                }

                uint32_t remainingLiters = remaining_x10 / 10UL;
                lv_label_set_text_fmt(lblProdCountdown, "LTG %lu L",
                                      (unsigned long)remainingLiters);
            }
        }
    }
}

static void update_all_button_visuals()
{
    bool outputsLocked = faultLatched;

    for (int s = 0; s < 4; s++) {
        if (btns[s][BTN_FEED]) {
            style_button(btns[s][BTN_FEED], prepumpOn ? COL_BTN_ON : COL_BTN_OFF);
            lv_obj_set_style_bg_opa(btns[s][BTN_FEED], outputsLocked ? LV_OPA_50 : LV_OPA_COVER, 0);
            lv_label_set_text(btnLabels[s][BTN_FEED], prepumpOn ? "FEED PUMP ON" : "FEED PUMP OFF");
        }
        if (btns[s][BTN_HPPUMP]) {
            style_button(btns[s][BTN_HPPUMP], hppumpOn ? COL_BTN_ON : COL_BTN_OFF);
            lv_obj_set_style_bg_opa(btns[s][BTN_HPPUMP], outputsLocked ? LV_OPA_50 : LV_OPA_COVER, 0);
            lv_label_set_text(btnLabels[s][BTN_HPPUMP], hppumpOn ? "H.P. PUMP ON" : "H.P. PUMP OFF");
        }
        if (btns[s][BTN_FLUSH]) {
            style_button(btns[s][BTN_FLUSH], flushOn ? COL_BTN_ON : COL_BTN_OFF);
            lv_obj_set_style_bg_opa(btns[s][BTN_FLUSH], outputsLocked ? LV_OPA_50 : LV_OPA_COVER, 0);
            lv_label_set_text(btnLabels[s][BTN_FLUSH], flushOn ? "FLUSH ON" : "FLUSH OFF");
        }
        if (btns[s][BTN_TANK]) {
            style_button(btns[s][BTN_TANK], tankTestOn ? COL_BTN_TEST : COL_BTN_OFF);
            lv_obj_set_style_bg_opa(btns[s][BTN_TANK], outputsLocked ? LV_OPA_50 : LV_OPA_COVER, 0);
            lv_label_set_text(btnLabels[s][BTN_TANK], tankTestOn ? "WATER TO TEST" : "WATER TO TANK");
        }
    }
    update_spinner_state();
    update_alarm_indicators();
    update_flow_sensor_selector_panel();
    update_production_screen_visuals();
}


// --------------------------------------------------
// EVENTS FOR BOTTOM BUTTONS
// --------------------------------------------------
static void prepump_event(lv_event_t *e)
{
    (void)e;
    cmdFeedToggle = true;
}

static void hppump_event(lv_event_t *e)
{
    (void)e;
    cmdHppumpToggle = true;
}

static void flush_event(lv_event_t *e)
{
    (void)e;
    cmdFlushToggle = true;
}

static void tank_event(lv_event_t *e)
{
    (void)e;
    cmdTankToggle = true;
}

// --------------------------------------------------
// SETTINGS SCREEN EVENTS
// --------------------------------------------------
static void flush_time_changed_cb(lv_event_t *e)
{
    lv_obj_t *roller = lv_event_get_target(e);
    char buf[16];
    lv_roller_get_selected_str(roller, buf, sizeof(buf));

    int minVal = atoi(buf);
    if (minVal >= 1 && minVal <= 5) {
        flushTimeSec = minVal * 60;
    }
}

static void trim_minus_cb(lv_event_t *e)
{
    (void)e;
    if (flowTrimPct > 50) flowTrimPct--;
    update_flow_trim_panel();
    update_alarm_indicators();
}

static void trim_plus_cb(lv_event_t *e)
{
    (void)e;
    if (flowTrimPct < 150) flowTrimPct++;
    update_flow_trim_panel();
    update_alarm_indicators();
}

static void sensor_a_cb(lv_event_t *e)
{
    (void)e;
    flowSensorMode = 'A';
    update_flow_sensor_selector_panel();
}

static void sensor_b_cb(lv_event_t *e)
{
    (void)e;
    flowSensorMode = 'B';
    update_flow_sensor_selector_panel();
}

static void fault_reset_cb(lv_event_t *e)
{
    (void)e;
    cmdFaultReset = true;
}

// --------------------------------------------------
// PRODUCTION SCREEN EVENTS
// --------------------------------------------------
static void prod_enable_cb(lv_event_t *e)
{
    (void)e;
    autoProductionEnabled = !autoProductionEnabled;
    update_production_screen_visuals();
}

static void prod_start_cb(lv_event_t *e)
{
    (void)e;

    // Se già attivo, il bottone diventa STOP AUTO:
    // annulla solo l'automatismo, senza spegnere la macchina
    if (autoProductionActive) {
        debug_log("AUTO STOP BY USER");
        autoProductionActive = false;
        update_production_screen_visuals();
        return;
    }

    // Per partire davvero servono:
    // 1) funzione abilitata
    // 2) HP pump accesa
    if (!autoProductionEnabled) {
        return;
    }

    if (!hppumpOn) {
        return;
    }

    if (autoProductionModeTime) {
        autoProductionStartRuntimeSec = hppumpRuntimeSec;
        autoProductionTargetRuntimeSec = hppumpRuntimeSec + ((uint32_t)autoProductionTimeHours * 3600UL);
        debug_logf("AUTO START TIME %d H", autoProductionTimeHours);
    } else {
        autoProductionStartLiters_x10 = totalProducedLiters_x10;
        autoProductionTargetLiters_x10 = totalProducedLiters_x10 + ((uint32_t)autoProductionTargetLiters * 10UL);
        debug_logf("AUTO START LITERS %d L", autoProductionTargetLiters);
    }

    autoProductionActive = true;
    update_production_screen_visuals();
}

static void prod_mode_time_cb(lv_event_t *e)
{
    (void)e;
    autoProductionModeTime = true;
    update_production_screen_visuals();
}

static void prod_mode_liters_cb(lv_event_t *e)
{
    (void)e;
    autoProductionModeTime = false;
    update_production_screen_visuals();
}

static void prod_target_minus_cb(lv_event_t *e)
{
    (void)e;

    if (autoProductionModeTime) {
        if (autoProductionTimeHours > 1) autoProductionTimeHours--;
    } else {
        if (autoProductionTargetLiters > 50) autoProductionTargetLiters -= 50;
    }

    update_production_screen_visuals();
}

static void prod_target_plus_cb(lv_event_t *e)
{
    (void)e;

    if (autoProductionModeTime) {
        if (autoProductionTimeHours < 5) autoProductionTimeHours++;
    } else {
        if (autoProductionTargetLiters < 500) autoProductionTargetLiters += 50;
    }

    update_production_screen_visuals();
}

// --------------------------------------------------
// RUNTIME / MEMORY
// --------------------------------------------------
static void schedule_runtime_save(uint32_t delayMs)
{
    runtimeSavePending = true;
    runtimeSaveDueMs = millis() + delayMs;
}

static void process_pending_runtime_save()
{
    if (!runtimeSavePending) return;

    unsigned long now = millis();
    if ((long)(now - runtimeSaveDueMs) < 0) return;

    if (framReady) {
        fram_write_u32(FRAM_ADDR_HPPUMP_RUNTIME, hppumpRuntimeSec);
    }

    runtimeSavePending = false;
    runtimeSaveDueMs = 0;
}

static void schedule_liters_save(uint32_t delayMs)
{
    litersSavePending = true;
    litersSaveDueMs = millis() + delayMs;
}

static void process_pending_liters_save()
{
    if (!litersSavePending) return;

    unsigned long now = millis();
    if ((long)(now - litersSaveDueMs) < 0) return;

    if (framReady) {
        fram_write_u32(FRAM_ADDR_TOTAL_LITERS_X10, totalProducedLiters_x10);
    }

    litersSavePending = false;
    litersSaveDueMs = 0;
}

static void update_hours_label()
{
    uint32_t totalSec = hppumpRuntimeSec;

    if (totalSec == lastDisplayedHoursSec) return;
    lastDisplayedHoursSec = totalSec;

    uint32_t hours = totalSec / 3600UL;
    uint32_t minutes = (totalSec % 3600UL) / 60UL;

    if (lblHoursValue) {
        lv_label_set_text_fmt(lblHoursValue, "%lu h  %02lu m",
                              (unsigned long)hours,
                              (unsigned long)minutes);
    }
}

static void update_liters_label()
{
    uint32_t litersInt = totalProducedLiters_x10 / 10UL;

    if (litersInt == lastDisplayedLitersInt) return;
    lastDisplayedLitersInt = litersInt;

    if (lblLitersValue) {
        lv_label_set_text_fmt(lblLitersValue, "%lu L", (unsigned long)litersInt);
    }
}

static void update_hppump_runtime()
{
    unsigned long now = millis();

    if (lastHppumpSecondTick == 0) {
        lastHppumpSecondTick = now;
        return;
    }

    if (hppumpOn) {
        while ((now - lastHppumpSecondTick) >= 1000UL) {
            hppumpRuntimeSec++;
            lastHppumpSecondTick += 1000UL;
        }
    } else {
        lastHppumpSecondTick = now;
    }
}

static void update_total_liters()
{
    unsigned long now = millis();

    if (lastLitersSecondTick == 0) {
        lastLitersSecondTick = now;
        return;
    }

    if (!hppumpOn) {
        lastLitersSecondTick = now;
        return;
    }

    while ((now - lastLitersSecondTick) >= 1000UL) {
        // Litri prodotti in 1 secondo
        float deltaLiters = flowLhTrimmed / 3600.0f;

        // Accumulo frazionario semplice
        litersFractionAcc += deltaLiters;

        // Ogni volta che raggiungiamo 0.1 L, incrementiamo il contatore persistente
        while (litersFractionAcc >= 0.1f) {
            totalProducedLiters_x10++;
            litersFractionAcc -= 0.1f;
            schedule_liters_save(60000UL);
        }

        lastLitersSecondTick += 1000UL;
    }
}

static void process_auto_production_logic()
{
    if (!autoProductionActive) return;
    if (!hppumpOn) return;

    if (autoProductionModeTime) {
        if (hppumpRuntimeSec >= autoProductionTargetRuntimeSec) {
            debug_log("AUTO TARGET REACHED TIME");
            safe_shutdown_outputs();
            autoProductionActive = false;
        }
    } else {
        if (totalProducedLiters_x10 >= autoProductionTargetLiters_x10) {
            debug_log("AUTO TARGET REACHED LITERS");
            safe_shutdown_outputs();
            autoProductionActive = false;
        }
    }
}

static void latch_fault(FaultCode code, const char *title, const char *message)
{
    if (faultLatched) return;

    debug_logf("FAULT %d: %s", (int)code, title ? title : "UNKNOWN");

    faultLatched = true;
    activeFault = code;

    if (title) {
        snprintf(faultTitle, sizeof(faultTitle), "%s", title);
    } else {
        snprintf(faultTitle, sizeof(faultTitle), "SYSTEM ALARM");
    }

    if (message) {
        snprintf(faultMessage, sizeof(faultMessage), "%s", message);
    } else {
        snprintf(faultMessage, sizeof(faultMessage), "Unknown fault.");
    }

    // Force all logical outputs OFF
    prepumpOn = false;
    hppumpOn = false;
    flushOn = false;
    tankTestOn = false;
    flushCycleActive = false;
    autoProductionActive = false;

    // stop runtime accumulation
    schedule_runtime_save(200UL);
    schedule_liters_save(200UL);

    // push visual updates
    update_all_button_visuals();
    update_alarm_indicators();

    // switch to alarm screen
    show_alarm_screen();
}

static void read_pressure_sensor()
{
    unsigned long now = millis();
    if ((now - lastPressureReadMs) < 250UL) return;
    lastPressureReadMs = now;

    float volts = 0.0f;
    if (!analog_read_voltage_rs485(ANALOG_REG_PRESSURE, &volts)) {
        return;
    }

    if (volts < 0.0f) volts = 0.0f;
    if (volts > PRESSURE_SENSOR_FULL_SCALE_V) volts = PRESSURE_SENSOR_FULL_SCALE_V;

    float bar = (volts / PRESSURE_SENSOR_FULL_SCALE_V) * PRESSURE_SENSOR_FULL_SCALE_BAR;

    pressureBar = (pressureBar * 0.75f) + (bar * 0.25f);
    pressurePsi = pressureBar * 14.5038f;

    alarmPressureHigh = (pressureBar >= PRESSURE_TRIP_BAR);

    if (pressureBar >= PRESSURE_TRIP_BAR) {
        latch_fault(FAULT_PRESSURE_HIGH,
                    "HIGH PRESSURE ALARM",
                    "Pressure reached 68 bar or higher. Fully open the needle valve, wait until pressure drops near zero, then press RESET.");
    }
}

static void read_tds_sensor()
{
    unsigned long now = millis();
    if ((now - lastTdsReadMs) < 800UL) return;
    lastTdsReadMs = now;

    float voltage = 0.0f;
    if (!analog_read_voltage_rs485(ANALOG_REG_TDS, &voltage)) {
        return;
    }

    float ppm = (voltage / TDS_OUTPUT_MAX_V) * TDS_MAX_PPM * TDS_CALIBRATION_FACTOR;

    if (ppm < 0.0f) ppm = 0.0f;
    if (ppm > 2000.0f) ppm = 2000.0f;

    tdsPpm = (tdsPpm * 0.80f) + (ppm * 0.20f);
    alarmTdsHigh = (tdsPpm > 500.0f);
}

static void read_flow_sensor()
{
    unsigned long now = millis();
    if ((now - lastFlowCalcMs) < 1000UL) return;   // lascia pure 1000 ms se ti va bene così
    lastFlowCalcMs = now;

    // Manteniamo il fresh init, visto che sul tuo bus ha dato stabilità
    rs485_begin_fresh();
    delay(10);

    uint16_t hz_x100 = 0;

    // Piccolo settling time dopo re-init bus
    delay(20);

    bool ok0 = modbus_read_register_u16_retry(20, 0x04, 0, &hz_x100);

    if (!ok0) {
        alarmNoFlow = true;
        return;
    }

    float hz = hz_x100 / 100.0f;

    float flow_l_min;

    if (flowSensorMode == 'A') {
        // HIGH FLOW sensor
        flow_l_min = (hz <= 1.0f) ? 0.0f : (hz + 3.0f) / 10.0f;
    } else {
        // LOW FLOW sensor
        flow_l_min = hz / 118.0f;
    }

    float flow_l_h = flow_l_min * 60.0f;

    // Smooth
    flowLhRaw = (flowLhRaw * 0.7f) + (flow_l_h * 0.3f);

    // Trim
    flowLhTrimmed = flowLhRaw * (flowTrimPct / 100.0f);

    // Convert to GPH
    flowGph = flowLhTrimmed * 0.264172f;

    alarmNoFlow = (flowLhTrimmed < FLOW_NOFLOW_THRESHOLD_LH);
}

static void process_fault_reset()
{
    if (!cmdFaultReset) return;
    cmdFaultReset = false;

    if (!faultLatched) return;

    if (activeFault == FAULT_PRESSURE_HIGH && pressureBar > PRESSURE_RESET_MAX_BAR) {
        return;
    }

    if (activeFault == FAULT_NO_FLOW && flowLhTrimmed < FLOW_NOFLOW_THRESHOLD_LH) {
        return;
    }

    debug_log("FAULT RESET");
    faultLatched = false;
    activeFault = FAULT_NONE;
    snprintf(faultTitle, sizeof(faultTitle), "SYSTEM OK");
    snprintf(faultMessage, sizeof(faultMessage), "Fault reset completed. Outputs remain OFF until you start them again.");
    show_main_screen();
}

static void controller_logic()
{
    if (cmdTankToggle) {
        cmdTankToggle = false;
        if (!faultLatched) {
            tankTestOn = !tankTestOn;
            debug_log(tankTestOn ? "WATER TO TEST" : "WATER TO TANK");
        }
    }

    if (cmdFeedToggle) {
        cmdFeedToggle = false;
        if (!faultLatched) {
            prepumpOn = !prepumpOn;
            debug_log(prepumpOn ? "FEED ON" : "FEED OFF");
        }
    }

    if (cmdHppumpToggle) {
        cmdHppumpToggle = false;

        if (!faultLatched) {
            if (!hppumpOn) {
                hppumpOn = true;
                debug_log("HP ON");
                hpStartedMs = millis();
                lastHppumpSecondTick = millis();
                runtimeSavePending = false;
                runtimeSaveDueMs = 0;
            } else {
                hppumpOn = false;
                debug_log("HP OFF");
                schedule_runtime_save(500UL);
                schedule_liters_save(500UL);
                autoProductionActive = false;
            }
        }
    }

    if (cmdFlushToggle) {
        cmdFlushToggle = false;

        if (!faultLatched) {
            if (!flushCycleActive) {
                flushCycleActive = true;
                flushStartMs = millis();
                flushOn = true;
                debug_log("FLUSH ON");
            } else {
                flushCycleActive = false;
                flushOn = false;
                debug_log("FLUSH OFF");
            }
        }
    }

    if (flushCycleActive) {
        unsigned long now = millis();
        if ((now - flushStartMs) >= (unsigned long)flushTimeSec * 1000UL) {
            flushCycleActive = false;
            flushOn = false;
            debug_log("FLUSH AUTO OFF");
        }
    }

    process_fault_reset();
}

// --------------------------------------------------
// SCREEN BUILDERS
// --------------------------------------------------
static void create_bottom_buttons_for_screen(lv_obj_t *parent, int screenIndex)
{
    const int gap = 10;
    const int margin = 5;
    const int btnW = 190;
    const int btnH = 110;
    const int y = 480 - btnH - margin;

    int x1 = margin;
    int x2 = x1 + btnW + gap;
    int x3 = x2 + btnW + gap;
    int x4 = x3 + btnW + gap;

    btns[screenIndex][BTN_FEED]   = make_button_bg(parent, x1, y, btnW, btnH, prepump_event, NULL);
    btns[screenIndex][BTN_HPPUMP] = make_button_bg(parent, x2, y, btnW, btnH, hppump_event, NULL);
    btns[screenIndex][BTN_FLUSH]  = make_button_bg(parent, x3, y, btnW, btnH, flush_event, NULL);
    btns[screenIndex][BTN_TANK]   = make_button_bg(parent, x4, y, btnW, btnH, tank_event, NULL);

    btnIcons[screenIndex][BTN_FEED]   = make_button_icon(parent, &img_prepump,  x1 + 65, y + 12);
    btnIcons[screenIndex][BTN_HPPUMP] = make_button_icon(parent, &img_hppump,   x2 + 65, y + 12);
    btnIcons[screenIndex][BTN_FLUSH]  = make_button_icon(parent, &img_flush,    x3 + 65, y + 12);
    btnIcons[screenIndex][BTN_TANK]   = make_button_icon(parent, &img_workflow, x4 + 65, y + 12);

    btnLabels[screenIndex][BTN_FEED]   = make_button_text(parent, "FEED PUMP OFF",      x1 + 4, y + 82, 182);
    btnLabels[screenIndex][BTN_HPPUMP] = make_button_text(parent, "H.P. PUMP OFF", x2 + 4, y + 82, 182);
    btnLabels[screenIndex][BTN_FLUSH]  = make_button_text(parent, "FLUSH OFF",     x3 + 4, y + 82, 182);
    btnLabels[screenIndex][BTN_TANK]   = make_button_text(parent, "WATER TO TANK", x4 + 4, y + 82, 182);
}

static void create_main_screen()
{
    screenMain = lv_obj_create(NULL);
    lv_obj_set_size(screenMain, 800, 480);
    lv_obj_set_style_bg_color(screenMain, COL_BG, 0);
    lv_obj_set_style_bg_opa(screenMain, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(screenMain, 0, 0);
    lv_obj_set_style_radius(screenMain, 0, 0);
    lv_obj_clear_flag(screenMain, LV_OBJ_FLAG_SCROLLABLE);

    create_header(screenMain, false, true, NULL, nav_to_settings_cb);

    alarmLed = create_alarm_led(screenMain, 20, 72, 22);
    lblAlarmStatus = create_alarm_status_label(screenMain, 52, 74);

    // instruments moved 15 px upward
    const int gaugeSize = 213;
    const int y = 110;

    // gentle equal backgrounds
    // side gauges moved 8 px toward center
    gaugeBgPressure = create_gauge_background(screenMain, 11,  y - 17, 253);
    gaugeBgFlow     = create_gauge_background(screenMain, 276, y - 17, 253);
    gaugeBgTds      = create_gauge_background(screenMain, 541, y - 17, 253);

    // LEFT = PRESSURE
    arcPressureOuterGreen  = create_arc_segment(screenMain, 21, y - 7, 233, 0,   162, COL_GREEN,  5);
    arcPressureOuterOrange = create_arc_segment(screenMain, 21, y - 7, 233, 162, 189, COL_ORANGE, 5);
    arcPressureOuterRed    = create_arc_segment(screenMain, 21, y - 7, 233, 189, 270, COL_RED,    5);

    arcPressure = create_gauge_base(screenMain, 30, y + 2, gaugeSize, 0, 100, 0);

    lv_obj_t *lblPressureTitle = lv_label_create(screenMain);
    lv_label_set_text(lblPressureTitle, "PRESSURE");
    lv_obj_set_style_text_color(lblPressureTitle, COL_WHITE, 0);
    lv_obj_set_style_text_font(lblPressureTitle, &lv_font_montserrat_16, 0);
    lv_obj_align_to(lblPressureTitle, arcPressure, LV_ALIGN_CENTER, 0, -40);

    lblPressureMain = lv_label_create(screenMain);
    lv_label_set_text(lblPressureMain, "0 BAR");
    lv_obj_set_style_text_color(lblPressureMain, COL_ARC_BLUE, 0);
    lv_obj_set_style_text_font(lblPressureMain, &lv_font_montserrat_18, 0);
    lv_obj_align_to(lblPressureMain, arcPressure, LV_ALIGN_CENTER, 0, 8);

    lblPressureSub = lv_label_create(screenMain);
    lv_label_set_text(lblPressureSub, "0 PSI");
    lv_obj_set_style_text_color(lblPressureSub, COL_ARC_BLUE, 0);
    lv_obj_set_style_text_font(lblPressureSub, &lv_font_montserrat_14, 0);
    lv_obj_align_to(lblPressureSub, lblPressureMain, LV_ALIGN_OUT_BOTTOM_MID, 0, 6);

    {
        const int cx = 21 + 233 / 2;
        const int cy = (y - 7) + 233 / 2;
        pressureMark60 = create_threshold_mark(screenMain, pressureMark60Pts, cx, cy, 116, 104, 60);
        pressureMark70 = create_threshold_mark(screenMain, pressureMark70Pts, cx, cy, 116, 104, 70);
    }

    // CENTER = FLOW
    arcFlowOuter = create_full_ring(screenMain, 286, y - 7, 233, 5, COL_ARC_BLUE, LV_OPA_90);
    arcFlowInner = create_full_ring(screenMain, 295, y + 2, gaugeSize, 10, COL_ARC_BLUE, LV_OPA_70);

    lv_obj_t *lblFlowTitle = lv_label_create(screenMain);
    lv_label_set_text(lblFlowTitle, "FLOW");
    lv_obj_set_style_text_color(lblFlowTitle, COL_WHITE, 0);
    lv_obj_set_style_text_font(lblFlowTitle, &lv_font_montserrat_16, 0);
    lv_obj_align_to(lblFlowTitle, arcFlowInner, LV_ALIGN_CENTER, 0, -40);

    lblFlowMain = lv_label_create(screenMain);
    lv_label_set_text(lblFlowMain, "0 L/H");
    lv_obj_set_style_text_color(lblFlowMain, COL_WHITE, 0);
    lv_obj_set_style_text_font(lblFlowMain, &lv_font_montserrat_18, 0);
    lv_obj_align_to(lblFlowMain, arcFlowInner, LV_ALIGN_CENTER, 0, 8);

    lblFlowSub = lv_label_create(screenMain);
    lv_label_set_text(lblFlowSub, "0.0 GPH");
    lv_obj_set_style_text_color(lblFlowSub, COL_SOFT, 0);
    lv_obj_set_style_text_font(lblFlowSub, &lv_font_montserrat_14, 0);
    lv_obj_align_to(lblFlowSub, lblFlowMain, LV_ALIGN_OUT_BOTTOM_MID, 0, 6);

    flowDot = create_flow_dot(screenMain, 10);

    // RIGHT = TDS
    arcTdsOuterGreen  = create_arc_segment(screenMain, 551, y - 7, 233, 0,   135, COL_GREEN,  5);
    arcTdsOuterOrange = create_arc_segment(screenMain, 551, y - 7, 233, 135, 203, COL_ORANGE, 5);
    arcTdsOuterRed    = create_arc_segment(screenMain, 551, y - 7, 233, 203, 270, COL_RED,    5);

    arcTds = create_gauge_base(screenMain, 560, y, gaugeSize, 0, 1000, 0);

    lv_obj_t *lblTdsTitle = lv_label_create(screenMain);
    lv_label_set_text(lblTdsTitle, "TDS");
    lv_obj_set_style_text_color(lblTdsTitle, COL_WHITE, 0);
    lv_obj_set_style_text_font(lblTdsTitle, &lv_font_montserrat_16, 0);
    lv_obj_align_to(lblTdsTitle, arcTds, LV_ALIGN_CENTER, 0, -40);

    lblTdsMain = lv_label_create(screenMain);
    lv_label_set_text(lblTdsMain, "0 PPM");
    lv_obj_set_style_text_color(lblTdsMain, COL_GREEN, 0);
    lv_obj_set_style_text_font(lblTdsMain, &lv_font_montserrat_18, 0);
    lv_obj_align_to(lblTdsMain, arcTds, LV_ALIGN_CENTER, 0, 8);

    lblTdsSub = lv_label_create(screenMain);
    lv_label_set_text(lblTdsSub, "QUALITY");
    lv_obj_set_style_text_color(lblTdsSub, COL_GREEN, 0);
    lv_obj_set_style_text_font(lblTdsSub, &lv_font_montserrat_14, 0);
    lv_obj_align_to(lblTdsSub, lblTdsMain, LV_ALIGN_OUT_BOTTOM_MID, 0, 6);

    {
        const int cx = 551 + 233 / 2;
        const int cy = (y - 7) + 233 / 2;
        tdsMark500 = create_threshold_mark(screenMain, tdsMark500Pts, cx, cy, 116, 104, 50);
        tdsMark750 = create_threshold_mark(screenMain, tdsMark750Pts, cx, cy, 116, 104, 75);
    }

    create_bottom_buttons_for_screen(screenMain, 0);
}

static void create_settings_screen()
{
    screenSettings = lv_obj_create(NULL);
    lv_obj_set_size(screenSettings, 800, 480);
    lv_obj_set_style_bg_color(screenSettings, COL_BG, 0);
    lv_obj_set_style_bg_opa(screenSettings, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(screenSettings, 0, 0);
    lv_obj_set_style_radius(screenSettings, 0, 0);
    lv_obj_clear_flag(screenSettings, LV_OBJ_FLAG_SCROLLABLE);

    create_header(screenSettings, true, true, nav_to_main_cb, nav_to_production_cb);

    // Central area
    // Header bottom = 60, gap = 15 => y = 75
    // Buttons top = 365, gap = 15 => bottom area end = 350
    // Height = 275
    const int panelY = 75;
    const int panelH = 275;
    const int gap = 10;
    const int margin = 10;
    const int panelW = 253; // leaves 11 px on far right, acceptable visually

    const int x1 = margin;              // 10
    const int x2 = x1 + panelW + gap;   // 273
    const int x3 = x2 + panelW + gap;   // 536

    panelHours = create_panel(screenSettings, x1, panelY, panelW, panelH);
    panelFlush = create_panel(screenSettings, x2, panelY, panelW, panelH);
    panelTrim  = create_panel(screenSettings, x3, panelY, panelW, panelH);

// ---------------- PANEL 1: SYSTEM TOTALS ----------------
lv_obj_t *lblHoursTitle = lv_label_create(panelHours);
lv_label_set_text(lblHoursTitle, "SYSTEM TOTALS");
lv_obj_set_style_text_color(lblHoursTitle, COL_WHITE, 0);
lv_obj_set_style_text_font(lblHoursTitle, &lv_font_montserrat_18, 0);
lv_obj_align(lblHoursTitle, LV_ALIGN_TOP_MID, 0, 14);

lv_obj_t *lblHpCaption = lv_label_create(panelHours);
lv_label_set_text(lblHpCaption, "H.P. PUMP");
lv_obj_set_style_text_color(lblHpCaption, COL_WHITE, 0);
lv_obj_set_style_text_font(lblHpCaption, &lv_font_montserrat_14, 0);
lv_obj_align(lblHpCaption, LV_ALIGN_TOP_MID, 0, 52);

lblHoursValue = lv_label_create(panelHours);
lv_label_set_text(lblHoursValue, "0 h  00 m");
lv_obj_set_style_text_color(lblHoursValue, COL_ARC_BLUE, 0);
lv_obj_set_style_text_font(lblHoursValue, &lv_font_montserrat_24, 0);
lv_obj_align(lblHoursValue, LV_ALIGN_TOP_MID, 0, 78);

lv_obj_t *lblLitersCaption = lv_label_create(panelHours);
lv_label_set_text(lblLitersCaption, "WATER PRODUCED");
lv_obj_set_style_text_color(lblLitersCaption, COL_WHITE, 0);
lv_obj_set_style_text_font(lblLitersCaption, &lv_font_montserrat_14, 0);
lv_obj_align(lblLitersCaption, LV_ALIGN_TOP_MID, 0, 145);

lblLitersValue = lv_label_create(panelHours);
lv_label_set_text(lblLitersValue, "0 L");
lv_obj_set_style_text_color(lblLitersValue, COL_ARC_BLUE, 0);
lv_obj_set_style_text_font(lblLitersValue, &lv_font_montserrat_24, 0);
lv_obj_align(lblLitersValue, LV_ALIGN_TOP_MID, 0, 171);

spinnerHours = lv_spinner_create(panelHours, 1000, 60);
lv_obj_set_size(spinnerHours, 42, 42);
lv_obj_align(spinnerHours, LV_ALIGN_BOTTOM_MID, 0, -18);
lv_obj_set_style_arc_width(spinnerHours, 4, LV_PART_MAIN);
lv_obj_set_style_arc_width(spinnerHours, 4, LV_PART_INDICATOR);
lv_obj_set_style_arc_color(spinnerHours, COL_ARC_BG, LV_PART_MAIN);
lv_obj_set_style_arc_color(spinnerHours, COL_ARC_BLUE, LV_PART_INDICATOR);

    // ---------------- PANEL 2: FLUSH PRESET ----------------
    lv_obj_t *lblFlushTitle = lv_label_create(panelFlush);
    lv_label_set_text(lblFlushTitle, "FLUSHING TIME PRESET");
    lv_obj_set_style_text_color(lblFlushTitle, COL_WHITE, 0);
    lv_obj_set_style_text_font(lblFlushTitle, &lv_font_montserrat_16, 0);
    lv_obj_align(lblFlushTitle, LV_ALIGN_TOP_MID, 0, 18);

    rollerFlushTime = lv_roller_create(panelFlush);
    lv_roller_set_options(rollerFlushTime,
                          "1 min\n"
                          "2 min\n"
                          "3 min\n"
                          "4 min\n"
                          "5 min",
                          LV_ROLLER_MODE_NORMAL);
    lv_roller_set_visible_row_count(rollerFlushTime, 3);
    lv_obj_set_size(rollerFlushTime, 170, 120);
    lv_obj_align(rollerFlushTime, LV_ALIGN_CENTER, 0, 10);
    lv_obj_add_event_cb(rollerFlushTime, flush_time_changed_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_set_style_text_font(rollerFlushTime, &lv_font_montserrat_20, 0);
    lv_roller_set_selected(rollerFlushTime, 2, LV_ANIM_OFF); // 3 min default

    // ---------------- PANEL 3: FLOW TRIM + SENSOR SELECTOR ----------------
    lv_obj_t *lblTrimTitle = lv_label_create(panelTrim);
    lv_label_set_text(lblTrimTitle, "FLOW FINE TUNING");
    lv_obj_set_style_text_color(lblTrimTitle, COL_WHITE, 0);
    lv_obj_set_style_text_font(lblTrimTitle, &lv_font_montserrat_18, 0);
    lv_obj_align(lblTrimTitle, LV_ALIGN_TOP_MID, 0, 18);

    lv_obj_t *btnMinus = lv_btn_create(panelTrim);
    lv_obj_set_size(btnMinus, 56, 56);
    lv_obj_align(btnMinus, LV_ALIGN_CENTER, -72, -39);
    style_button(btnMinus, COL_BTN_OFF);
    lv_obj_add_event_cb(btnMinus, trim_minus_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *lblMinus = lv_label_create(btnMinus);
    lv_label_set_text(lblMinus, "-");
    lv_obj_set_style_text_font(lblMinus, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(lblMinus, COL_WHITE, 0);
    lv_obj_center(lblMinus);

    lv_obj_t *btnPlus = lv_btn_create(panelTrim);
    lv_obj_set_size(btnPlus, 56, 56);
    lv_obj_align(btnPlus, LV_ALIGN_CENTER, 72, -39);
    style_button(btnPlus, COL_BTN_OFF);
    lv_obj_add_event_cb(btnPlus, trim_plus_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *lblPlus = lv_label_create(btnPlus);
    lv_label_set_text(lblPlus, "+");
    lv_obj_set_style_text_font(lblPlus, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(lblPlus, COL_WHITE, 0);
    lv_obj_center(lblPlus);

    lblTrimValue = lv_label_create(panelTrim);
    lv_label_set_text(lblTrimValue, "100%");
    lv_obj_set_style_text_color(lblTrimValue, COL_ARC_BLUE, 0);
    lv_obj_set_style_text_font(lblTrimValue, &lv_font_montserrat_28, 0);
    lv_obj_align(lblTrimValue, LV_ALIGN_CENTER, 0, -39);

    lv_obj_t *lblSensorTitle = lv_label_create(panelTrim);
    lv_label_set_text(lblSensorTitle, "FLOW SENSOR");
    lv_obj_set_style_text_color(lblSensorTitle, COL_WHITE, 0);
    lv_obj_set_style_text_font(lblSensorTitle, &lv_font_montserrat_16, 0);
    lv_obj_align(lblSensorTitle, LV_ALIGN_CENTER, 0, 26);

    btnSensorA = lv_btn_create(panelTrim);
    lv_obj_set_size(btnSensorA, 64, 42);
    lv_obj_align(btnSensorA, LV_ALIGN_CENTER, -44, 68);
    style_button(btnSensorA, COL_BTN_ON);
    lv_obj_add_event_cb(btnSensorA, sensor_a_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *lblA = lv_label_create(btnSensorA);
    lv_label_set_text(lblA, "A");
    lv_obj_set_style_text_font(lblA, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(lblA, COL_WHITE, 0);
    lv_obj_center(lblA);

    btnSensorB = lv_btn_create(panelTrim);
    lv_obj_set_size(btnSensorB, 64, 42);
    lv_obj_align(btnSensorB, LV_ALIGN_CENTER, 44, 68);
    style_button(btnSensorB, COL_BTN_OFF);
    lv_obj_add_event_cb(btnSensorB, sensor_b_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *lblB = lv_label_create(btnSensorB);
    lv_label_set_text(lblB, "B");
    lv_obj_set_style_text_font(lblB, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(lblB, COL_WHITE, 0);
    lv_obj_center(lblB);

    lblSensorModeValue = lv_label_create(panelTrim);
    lv_label_set_text(lblSensorModeValue, "SENSOR A  (F = 11 x Q)");
    lv_obj_set_style_text_color(lblSensorModeValue, COL_SOFT, 0);
    lv_obj_set_style_text_font(lblSensorModeValue, &lv_font_montserrat_12, 0);
    lv_obj_set_width(lblSensorModeValue, 220);
    lv_obj_set_style_text_align(lblSensorModeValue, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(lblSensorModeValue, LV_ALIGN_BOTTOM_MID, 0, -18);

    create_bottom_buttons_for_screen(screenSettings, 1);
}


static void create_production_screen()
{
    screenProduction = lv_obj_create(NULL);
    lv_obj_set_size(screenProduction, 800, 480);
    lv_obj_set_style_bg_color(screenProduction, COL_BG, 0);
    lv_obj_set_style_bg_opa(screenProduction, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(screenProduction, 0, 0);
    lv_obj_set_style_radius(screenProduction, 0, 0);
    lv_obj_clear_flag(screenProduction, LV_OBJ_FLAG_SCROLLABLE);

    create_header(screenProduction, true, true, nav_to_settings_from_production_cb, nav_to_debug_cb);

    // Central area
    const int panelY = 75;
    const int panelH = 275;
    const int gap = 10;
    const int margin = 10;
    const int panelW = 253;

    const int x1 = margin;
    const int x2 = x1 + panelW + gap;
    const int x3 = x2 + panelW + gap;

    // Nuovo ordine sinistra -> destra:
    // 1) MODE
    // 2) TARGET
    // 3) ENABLE
    panelProdMode   = create_panel(screenProduction, x1, panelY, panelW, panelH);
    panelProdTarget = create_panel(screenProduction, x2, panelY, panelW, panelH);
    panelProdEnable = create_panel(screenProduction, x3, panelY, panelW, panelH);

    // ---------------- PANEL 1: TARGET MODE ----------------
    lv_obj_t *t1 = lv_label_create(panelProdMode);
    lv_label_set_text(t1, "TARGET MODE");
    lv_obj_set_style_text_color(t1, COL_WHITE, 0);
    lv_obj_set_style_text_font(t1, &lv_font_montserrat_18, 0);
    lv_obj_align(t1, LV_ALIGN_TOP_MID, 0, 18);

    btnModeTime = lv_btn_create(panelProdMode);
    lv_obj_set_size(btnModeTime, 92, 52);
    lv_obj_align(btnModeTime, LV_ALIGN_CENTER, -55, -26);
    style_button(btnModeTime, COL_BTN_ON);
    lv_obj_add_event_cb(btnModeTime, prod_mode_time_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *lblModeTime = lv_label_create(btnModeTime);
    lv_label_set_text(lblModeTime, "TIME");
    lv_obj_set_style_text_color(lblModeTime, COL_WHITE, 0);
    lv_obj_set_style_text_font(lblModeTime, &lv_font_montserrat_18, 0);
    lv_obj_center(lblModeTime);

    btnModeLiters = lv_btn_create(panelProdMode);
    lv_obj_set_size(btnModeLiters, 92, 52);
    lv_obj_align(btnModeLiters, LV_ALIGN_CENTER, 55, -26);
    style_button(btnModeLiters, COL_BTN_OFF);
    lv_obj_add_event_cb(btnModeLiters, prod_mode_liters_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *lblModeLiters = lv_label_create(btnModeLiters);
    lv_label_set_text(lblModeLiters, "LITERS");
    lv_obj_set_style_text_color(lblModeLiters, COL_WHITE, 0);
    lv_obj_set_style_text_font(lblModeLiters, &lv_font_montserrat_16, 0);
    lv_obj_center(lblModeLiters);

    lblProdModeValue = lv_label_create(panelProdMode);
    lv_label_set_text(lblProdModeValue, "TIME MODE");
    lv_obj_set_style_text_color(lblProdModeValue, COL_ARC_BLUE, 0);
    lv_obj_set_style_text_font(lblProdModeValue, &lv_font_montserrat_22, 0);
    lv_obj_align(lblProdModeValue, LV_ALIGN_BOTTOM_MID, 0, -34);

    // ---------------- PANEL 2: TARGET / COUNTDOWN ----------------
    lv_obj_t *t2 = lv_label_create(panelProdTarget);
    lv_label_set_text(t2, "TARGET / COUNTDOWN");
    lv_obj_set_style_text_color(t2, COL_WHITE, 0);
    lv_obj_set_style_text_font(t2, &lv_font_montserrat_16, 0);
    lv_obj_align(t2, LV_ALIGN_TOP_MID, 0, 18);

    // Pulsanti +/-
    btnTargetMinus = lv_btn_create(panelProdTarget);
    lv_obj_set_size(btnTargetMinus, 92, 52);
    lv_obj_align(btnTargetMinus, LV_ALIGN_CENTER, -60, -26);
    style_button(btnTargetMinus, COL_BTN_OFF);
    lv_obj_add_event_cb(btnTargetMinus, prod_target_minus_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *lblMinus = lv_label_create(btnTargetMinus);
    lv_label_set_text(lblMinus, "-");
    lv_obj_set_style_text_font(lblMinus, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(lblMinus, COL_WHITE, 0);
    lv_obj_center(lblMinus);

    btnTargetPlus = lv_btn_create(panelProdTarget);
    lv_obj_set_size(btnTargetPlus, 92, 52);
    lv_obj_align(btnTargetPlus, LV_ALIGN_CENTER, 60, -26);
    style_button(btnTargetPlus, COL_BTN_OFF);
    lv_obj_add_event_cb(btnTargetPlus, prod_target_plus_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *lblPlus = lv_label_create(btnTargetPlus);
    lv_label_set_text(lblPlus, "+");
    lv_obj_set_style_text_font(lblPlus, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(lblPlus, COL_WHITE, 0);
    lv_obj_center(lblPlus);

    lblProdTargetValue = lv_label_create(panelProdTarget);
    lv_label_set_text(lblProdTargetValue, "1 HOUR");
    lv_obj_set_style_text_color(lblProdTargetValue, COL_ARC_BLUE, 0);
    lv_obj_set_style_text_font(lblProdTargetValue, &lv_font_montserrat_24, 0);
    lv_obj_align(lblProdTargetValue, LV_ALIGN_CENTER, 0, 50);

    lblProdCountdown = lv_label_create(panelProdTarget);
    lv_label_set_text(lblProdCountdown, "TTG --:--:--");
    lv_obj_set_style_text_color(lblProdCountdown, COL_WHITE, 0);
    lv_obj_set_style_text_font(lblProdCountdown, &lv_font_montserrat_18, 0);
    lv_obj_align(lblProdCountdown, LV_ALIGN_CENTER, 0, 95);

    // ---------------- PANEL 3: AUTO PRODUCTION ----------------
    lv_obj_t *t3 = lv_label_create(panelProdEnable);
    lv_label_set_text(t3, "AUTO PRODUCTION");
    lv_obj_set_style_text_color(t3, COL_WHITE, 0);
    lv_obj_set_style_text_font(t3, &lv_font_montserrat_18, 0);
    lv_obj_align(t3, LV_ALIGN_TOP_MID, 0, 18);

    btnProdEnable = lv_btn_create(panelProdEnable);
    lv_obj_set_size(btnProdEnable, 140, 58);
    lv_obj_align(btnProdEnable, LV_ALIGN_CENTER, 0, -18);
    style_button(btnProdEnable, COL_BTN_OFF);
    lv_obj_add_event_cb(btnProdEnable, prod_enable_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *lblEnableBtn = lv_label_create(btnProdEnable);
    lv_label_set_text(lblEnableBtn, "ON / OFF");
    lv_obj_set_style_text_color(lblEnableBtn, COL_WHITE, 0);
    lv_obj_set_style_text_font(lblEnableBtn, &lv_font_montserrat_20, 0);
    lv_obj_center(lblEnableBtn);

lblProdEnableValue = lv_label_create(panelProdEnable);
lv_label_set_text(lblProdEnableValue, "DISABLED");
lv_obj_set_style_text_color(lblProdEnableValue, COL_ARC_BLUE, 0);
lv_obj_set_style_text_font(lblProdEnableValue, &lv_font_montserrat_24, 0);
lv_obj_align(lblProdEnableValue, LV_ALIGN_CENTER, 0, 34);

btnProdStart = lv_btn_create(panelProdEnable);
lv_obj_set_size(btnProdStart, 140, 52);
lv_obj_align(btnProdStart, LV_ALIGN_BOTTOM_MID, 0, -26);
style_button(btnProdStart, COL_BTN_OFF);
lv_obj_add_event_cb(btnProdStart, prod_start_cb, LV_EVENT_CLICKED, NULL);

lv_obj_t *lblStartBtn = lv_label_create(btnProdStart);
lv_label_set_text(lblStartBtn, "START AUTO");
lv_obj_set_style_text_color(lblStartBtn, COL_WHITE, 0);
lv_obj_set_style_text_font(lblStartBtn, &lv_font_montserrat_18, 0);
lv_obj_center(lblStartBtn);

    update_production_screen_visuals();

    create_bottom_buttons_for_screen(screenProduction, 2);
}

static void create_debug_screen()
{
    screenDebug = lv_obj_create(NULL);
    lv_obj_set_size(screenDebug, 800, 700);
    lv_obj_set_style_bg_color(screenDebug, COL_BG, 0);
    lv_obj_set_style_bg_opa(screenDebug, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(screenDebug, 0, 0);
    lv_obj_set_style_radius(screenDebug, 0, 0);


    create_header(screenDebug, true, false, nav_to_production_from_debug_cb, NULL);

    const int panelY = 75;
    const int panelH = 275;
    const int gap = 10;
    const int margin = 10;
    const int panelW = 253;

    const int x1 = margin;
    const int x2 = x1 + panelW + gap;
    const int x3 = x2 + panelW + gap;

    panelDebugSystem = create_panel(screenDebug, x1, panelY, panelW, panelH);
    panelDebugMemory = create_panel(screenDebug, x2, panelY, panelW, panelH);
    panelDebugEvents = create_panel(screenDebug, x3, panelY, panelW, panelH);

    const int bootPanelY = panelY + panelH + 15;
const int bootPanelH = 280;

panelBootLog = create_panel(screenDebug, 10, bootPanelY, 780, bootPanelH);

    // PANEL 1
    lv_obj_t *t1 = lv_label_create(panelDebugSystem);
    lv_label_set_text(t1, "SYSTEM");
    lv_obj_set_style_text_color(t1, COL_WHITE, 0);
    lv_obj_set_style_text_font(t1, &lv_font_montserrat_18, 0);
    lv_obj_align(t1, LV_ALIGN_TOP_MID, 0, 18);

    lblDebugSystemValue = lv_label_create(panelDebugSystem);
    lv_obj_set_width(lblDebugSystemValue, 225);
    lv_obj_set_style_text_color(lblDebugSystemValue, COL_SOFT, 0);
    lv_obj_set_style_text_font(lblDebugSystemValue, &lv_font_montserrat_16, 0);
    lv_obj_align(lblDebugSystemValue, LV_ALIGN_TOP_LEFT, 14, 56);

    // PANEL 2
    lv_obj_t *t2 = lv_label_create(panelDebugMemory);
    lv_label_set_text(t2, "MEMORY / STATUS");
    lv_obj_set_style_text_color(t2, COL_WHITE, 0);
    lv_obj_set_style_text_font(t2, &lv_font_montserrat_18, 0);
    lv_obj_align(t2, LV_ALIGN_TOP_MID, 0, 18);

    lblDebugMemoryValue = lv_label_create(panelDebugMemory);
    lv_obj_set_width(lblDebugMemoryValue, 225);
    lv_obj_set_style_text_color(lblDebugMemoryValue, COL_SOFT, 0);
    lv_obj_set_style_text_font(lblDebugMemoryValue, &lv_font_montserrat_16, 0);
    lv_obj_align(lblDebugMemoryValue, LV_ALIGN_TOP_LEFT, 14, 56);

    // PANEL 3
    lv_obj_t *t3 = lv_label_create(panelDebugEvents);
    lv_label_set_text(t3, "RECENT EVENTS");
    lv_obj_set_style_text_color(t3, COL_WHITE, 0);
    lv_obj_set_style_text_font(t3, &lv_font_montserrat_18, 0);
    lv_obj_align(t3, LV_ALIGN_TOP_MID, 0, 18);

    lblDebugEventsValue = lv_label_create(panelDebugEvents);
    lv_obj_set_width(lblDebugEventsValue, 225);
    lv_obj_set_style_text_color(lblDebugEventsValue, COL_SOFT, 0);
    lv_obj_set_style_text_font(lblDebugEventsValue, &lv_font_montserrat_14, 0);
    lv_label_set_long_mode(lblDebugEventsValue, LV_LABEL_LONG_WRAP);
    lv_obj_align(lblDebugEventsValue, LV_ALIGN_TOP_LEFT, 14, 56);

    // BOOT LOG PANEL
lv_obj_t *t4 = lv_label_create(panelBootLog);
lv_label_set_text(t4, "BOOT LOG");
lv_obj_set_style_text_color(t4, COL_WHITE, 0);
lv_obj_set_style_text_font(t4, &lv_font_montserrat_18, 0);
lv_obj_align(t4, LV_ALIGN_TOP_MID, 0, 18);

lblBootLogValue = lv_label_create(panelBootLog);
lv_obj_set_width(lblBootLogValue, 750);
lv_obj_set_style_text_color(lblBootLogValue, COL_SOFT, 0);
lv_obj_set_style_text_font(lblBootLogValue, &lv_font_montserrat_14, 0);
lv_label_set_long_mode(lblBootLogValue, LV_LABEL_LONG_WRAP);
lv_obj_align(lblBootLogValue, LV_ALIGN_TOP_LEFT, 14, 56);

    
    update_debug_screen();
}

static void create_alarm_screen()
{
    screenAlarm = lv_obj_create(NULL);
    lv_obj_set_size(screenAlarm, 800, 480);
    lv_obj_set_style_bg_color(screenAlarm, COL_BG, 0);
    lv_obj_set_style_bg_opa(screenAlarm, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(screenAlarm, 0, 0);
    lv_obj_set_style_radius(screenAlarm, 0, 0);
    lv_obj_clear_flag(screenAlarm, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *top = lv_obj_create(screenAlarm);
    lv_obj_set_size(top, 800, 60);
    lv_obj_set_pos(top, 0, 0);
    lv_obj_set_style_bg_color(top, COL_RED, 0);
    lv_obj_set_style_bg_opa(top, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(top, 0, 0);
    lv_obj_set_style_radius(top, 0, 0);
    lv_obj_clear_flag(top, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title = lv_label_create(top);
    lv_label_set_text(title, "N.E.R.D. - Watermaker Controller");
    lv_obj_set_style_text_color(title, COL_WHITE, 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_20, 0);
    lv_obj_align(title, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t *panel = create_panel(screenAlarm, 40, 88, 720, 290);

    lblFaultTitle = lv_label_create(panel);
    lv_label_set_text(lblFaultTitle, "SYSTEM ALARM");
    lv_obj_set_style_text_color(lblFaultTitle, COL_RED, 0);
    lv_obj_set_style_text_font(lblFaultTitle, &lv_font_montserrat_24, 0);
    lv_obj_align(lblFaultTitle, LV_ALIGN_TOP_MID, 0, 22);

    lblFaultMessage = lv_label_create(panel);
    lv_obj_set_width(lblFaultMessage, 640);
    lv_label_set_long_mode(lblFaultMessage, LV_LABEL_LONG_WRAP);
    lv_label_set_text(lblFaultMessage, "Fault details will appear here.");
    lv_obj_set_style_text_color(lblFaultMessage, COL_WHITE, 0);
    lv_obj_set_style_text_font(lblFaultMessage, &lv_font_montserrat_18, 0);
    lv_obj_align(lblFaultMessage, LV_ALIGN_TOP_MID, 0, 78);

    lv_obj_t *btnReset = lv_btn_create(panel);
    lv_obj_set_size(btnReset, 220, 64);
    lv_obj_align(btnReset, LV_ALIGN_BOTTOM_MID, 0, -20);
    style_button(btnReset, COL_BTN_ON);
    lv_obj_add_event_cb(btnReset, fault_reset_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *lblReset = lv_label_create(btnReset);
    lv_label_set_text(lblReset, "RESET ALARM");
    lv_obj_set_style_text_color(lblReset, COL_WHITE, 0);
    lv_obj_set_style_text_font(lblReset, &lv_font_montserrat_20, 0);
    lv_obj_center(lblReset);
}

static void show_main_screen()
{
    lv_scr_load(screenMain);
    update_all_button_visuals();
    update_hours_label();
    update_flow_trim_panel();
    update_alarm_indicators();
}

static void show_settings_screen()
{
    lv_scr_load(screenSettings);
    update_all_button_visuals();
    update_hours_label();
    update_flow_trim_panel();
    update_alarm_indicators();
}

static void show_production_screen()
{
    if (!screenProduction) return;
    lv_scr_load(screenProduction);
}

static void show_debug_screen()
{
    if (!screenDebug) return;
    lv_scr_load(screenDebug);
}

static void show_alarm_screen()
{
    if (lblFaultTitle) lv_label_set_text(lblFaultTitle, faultTitle);
    if (lblFaultMessage) lv_label_set_text(lblFaultMessage, faultMessage);
    lv_scr_load(screenAlarm);
}

static void update_debug_screen()
{
    if (lblDebugSystemValue) {
        lv_label_set_text_fmt(
            lblDebugSystemValue,
            "SW VERSION: %s\n"
            "RESET NOW: %s (%d)\n"
            "RESET PREV: %s (%lu)\n"
            "BOOT COUNT: %lu\n"
            "LAST UPTIME: %lu s",
            SW_VERSION,
            reset_reason_to_string(esp_reset_reason()),
            (int)esp_reset_reason(),
            reset_reason_to_string((esp_reset_reason_t)debugLastResetReason),
            (unsigned long)debugLastResetReason,
            (unsigned long)debugBootCount,
            (unsigned long)debugLastUptimeSec
        );
    }

    if (lblDebugMemoryValue) {
        lv_label_set_text_fmt(
            lblDebugMemoryValue,
            "HEAP: %lu\n"
            "PSRAM: %lu\n"
            "HP RUNTIME: %lu s\n"
            "TOTAL LITERS: %lu L",
            (unsigned long)ESP.getFreeHeap(),
            (unsigned long)ESP.getFreePsram(),
            (unsigned long)hppumpRuntimeSec,
            (unsigned long)(totalProducedLiters_x10 / 10UL)
        );
    }

    if (lblDebugEventsValue) {
        char buf[1200];
        buf[0] = '\0';

        int count = debugLogWrapped ? DEBUG_LOG_LINES : debugLogWriteIndex;
        int start = debugLogWrapped ? debugLogWriteIndex : 0;

        for (int i = 0; i < count; i++) {
            int idx = (start + i) % DEBUG_LOG_LINES;
            if (debugLogBuffer[idx][0] == '\0') continue;

            strncat(buf, debugLogBuffer[idx], sizeof(buf) - strlen(buf) - 1);
            strncat(buf, "\n", sizeof(buf) - strlen(buf) - 1);
        }

        if (buf[0] == '\0') {
            snprintf(buf, sizeof(buf), "NO EVENTS YET");
        }

        lv_label_set_text(lblDebugEventsValue, buf);
    }

    if (lblBootLogValue) {
    char buf[3000];
    buf[0] = '\0';

    int count = bootLogWrapped ? BOOT_LOG_LINES : bootLogWriteIndex;
    int start = bootLogWrapped ? bootLogWriteIndex : 0;

    for (int i = 0; i < count; i++) {
        int idx = (start + i) % BOOT_LOG_LINES;
        if (bootLogBuffer[idx][0] == '\0') continue;

        strncat(buf, bootLogBuffer[idx], sizeof(buf) - strlen(buf) - 1);
        strncat(buf, "\n", sizeof(buf) - strlen(buf) - 1);
    }

    if (buf[0] == '\0') {
        snprintf(buf, sizeof(buf), "NO BOOT LOG YET");
    }

    lv_label_set_text(lblBootLogValue, buf);
}
}

// --------------------------------------------------
// UPDATERS
// --------------------------------------------------
static void update_alarm_indicators()
{
    bool anyAlarm = faultLatched || alarmPressureHigh || alarmTdsHigh || alarmNoFlow;

    if (alarmLed) {
        lv_color_t ledColor = COL_GREEN;
        if (faultLatched) ledColor = COL_RED;
        else if (anyAlarm) ledColor = COL_ORANGE;
        lv_obj_set_style_bg_color(alarmLed, ledColor, 0);
    }

    if (lblAlarmStatus) {
        if (faultLatched) lv_label_set_text(lblAlarmStatus, "FAULT LATCHED");
        else if (alarmTdsHigh) lv_label_set_text(lblAlarmStatus, "TDS HIGH");
        else if (alarmNoFlow) lv_label_set_text(lblAlarmStatus, "FLOW LOW");
        else lv_label_set_text(lblAlarmStatus, "SYSTEM OK");
    }

    if (lblFaultTitle) lv_label_set_text(lblFaultTitle, faultTitle);
    if (lblFaultMessage) lv_label_set_text(lblFaultMessage, faultMessage);
}

static void update_pressure_gauge()
{
    int pressureDisplay = (int)(pressureBar + 0.5f);
    if (pressureDisplay < 0) pressureDisplay = 0;
    if (pressureDisplay > 100) pressureDisplay = 100;

    lv_arc_set_value(arcPressure, pressureDisplay);

    int psiDisplay = (int)(pressurePsi + 0.5f);

    lv_label_set_text_fmt(lblPressureMain, "%d BAR", pressureDisplay);
    lv_label_set_text_fmt(lblPressureSub, "%d PSI", psiDisplay);

    lv_color_t pressureColor;
    if (pressureBar < 60.0f) pressureColor = COL_ARC_BLUE;
    else if (pressureBar < PRESSURE_TRIP_BAR) pressureColor = COL_ORANGE;
    else pressureColor = COL_RED;

    lv_obj_set_style_arc_color(arcPressure, pressureColor, LV_PART_INDICATOR);
    lv_obj_set_style_text_color(lblPressureMain, pressureColor, 0);
    lv_obj_set_style_text_color(lblPressureSub, pressureColor, 0);
}

static void update_tds_gauge()
{
    int tdsDisplay = (int)(tdsPpm + 0.5f);
    if (tdsDisplay < 0) tdsDisplay = 0;
    if (tdsDisplay > 1000) tdsDisplay = 1000;

    lv_arc_set_value(arcTds, tdsDisplay);
    lv_label_set_text_fmt(lblTdsMain, "%d PPM", tdsDisplay);

    lv_color_t tdsColor;
    if (tdsPpm <= 500.0f) tdsColor = COL_GREEN;
    else if (tdsPpm <= 750.0f) tdsColor = COL_ORANGE;
    else tdsColor = COL_RED;

    lv_obj_set_style_arc_color(arcTds, tdsColor, LV_PART_INDICATOR);
    lv_obj_set_style_text_color(lblTdsMain, tdsColor, 0);
    lv_obj_set_style_text_color(lblTdsSub, tdsColor, 0);
}

static void update_flow_gauge()
{
    int flowDisplay = (int)(flowLhTrimmed + 0.5f);
    if (flowDisplay < 0) flowDisplay = 0;

    int gph10 = (int)(flowGph * 10.0f + 0.5f);
    int gphInt = gph10 / 10;
    int gphDec = gph10 % 10;

    lv_label_set_text_fmt(lblFlowMain, "%d L/H", flowDisplay);
    lv_label_set_text_fmt(lblFlowSub, "%d.%d GPH", gphInt, gphDec);

    bool flowActive = (flowDisplay > 0);

    if (flowActive) {
        if (millis() - lastFlowDotUpdate >= 35) {
            lastFlowDotUpdate = millis();
            flowDotAngle += 6;
            if (flowDotAngle >= 360) flowDotAngle -= 360;
        }

        lv_obj_clear_flag(flowDot, LV_OBJ_FLAG_HIDDEN);

        const int flowCx = 293 + 213 / 2;
        const int flowCy = 110 + 213 / 2;
        set_flow_dot_position(flowCx, flowCy, 101, flowDotAngle);
    } else {
        lv_obj_add_flag(flowDot, LV_OBJ_FLAG_HIDDEN);
    }
}

static void create_splash_screen()
{
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_set_size(scr, 800, 480);

    // sfondo blu header
    lv_obj_set_style_bg_color(scr, COL_TOPBAR, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(scr, 0, 0);

    // titolo principale
    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "N.E.R.D.");
    lv_obj_set_style_text_color(title, COL_WHITE, 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_28, 0);
    lv_obj_align(title, LV_ALIGN_CENTER, 0, -10);

    // sottotitolo
    lv_obj_t *subtitle = lv_label_create(scr);
    lv_label_set_text(subtitle, "Watermaker Controller");
    lv_obj_set_style_text_color(subtitle, COL_WHITE, 0);
    lv_obj_set_style_text_font(subtitle, &lv_font_montserrat_16, 0);
    lv_obj_align_to(subtitle, title, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);

    lv_scr_load(scr);
}

// --------------------------------------------------
// UI CREATE
// --------------------------------------------------
static void create_ui()
{
    create_main_screen();
    create_settings_screen();
    create_production_screen();
    create_alarm_screen();
    create_debug_screen();
    update_all_button_visuals();
    update_hours_label();
    update_flow_trim_panel();
    update_flow_sensor_selector_panel();
    show_main_screen();
}

// -------------------------------------------------- 
// SETUP / LOOP
// --------------------------------------------------

static void poll_sensors_round_robin()
{
    static uint8_t pollPhase = 0;

    switch (pollPhase) {
        case 0:
            read_flow_sensor();
            break;

        case 1:
            read_tds_sensor();
            break;

        case 2:
            read_pressure_sensor();
            break;
    }

    pollPhase++;
    if (pollPhase > 2) pollPhase = 0;
}

void setup()
{
    Serial.begin(115200);
    delay(200);

    boot_log("BOOT START");

    // --- V5 SAFE START (C3 connected) ---
    
    delay(1000);

    prefs.begin("bgwm", false);
    hppumpRuntimeSec = 0;

    init_hardware();
    lastFlowCalcMs = millis();

    Board *board = new Board();
    board->init();

#if LVGL_PORT_AVOID_TEARING_MODE
    auto lcd = board->getLCD();
    lcd->configFrameBufferNumber(LVGL_PORT_DISP_BUFFER_NUM);
#if ESP_PANEL_DRIVERS_BUS_ENABLE_RGB && CONFIG_IDF_TARGET_ESP32S3
    auto lcd_bus = lcd->getBus();
    if (lcd_bus->getBasicAttributes().type == ESP_PANEL_BUS_TYPE_RGB) {
        static_cast<BusRGB *>(lcd_bus)->configRGB_BounceBufferSize(lcd->getFrameWidth() * 10);
    }
#endif
#endif

    board->begin();
    boot_log("BOARD BEGIN OK");
    lvgl_port_init(board->getLCD(), board->getTouch());
    boot_log("LVGL INIT OK");

// SPLASH SCREEN
lvgl_port_lock(-1);
create_splash_screen();
lvgl_port_unlock();

// pausa (non troppo lunga → ok 2s)
delay(2000);

// UI vera
lvgl_port_lock(-1);
create_ui();
boot_log("UI CREATED");
lvgl_port_unlock();

    framInitEarliestMs = millis() + 2000UL;
}

void loop()
{
    controller_logic();
    update_hppump_runtime();
    fram_try_init_deferred();

    //poll_sensors_round_robin();

static unsigned long lastSensorPollMs = 0;
unsigned long nowSensor = millis();

if ((nowSensor - lastSensorPollMs) >= 250UL) {
    lastSensorPollMs = nowSensor;
    poll_sensors_round_robin();
}

    update_total_liters();
    process_auto_production_logic();

    apply_outputs();
    process_pending_runtime_save();
    process_pending_liters_save();
    // debug_save_heartbeat_persistent();   // TEMP disabled during I2C/touch stability tests

    // --------------------------------------------------
    // GUI update throttling
    // Update LVGL objects only 5 times per second
    // --------------------------------------------------
    static unsigned long lastGuiUpdateMs = 0;
    unsigned long now = millis();

    if ((now - lastGuiUpdateMs) >= 200UL) {
        lastGuiUpdateMs = now;

        lvgl_port_lock(-1);

        update_flow_gauge();
        update_pressure_gauge();
        update_tds_gauge();
        update_hours_label();
        update_liters_label();
        update_all_button_visuals();

        if (lv_scr_act() == screenDebug) {
            update_debug_screen();
        }

        if (faultLatched && lv_scr_act() != screenAlarm) {
            show_alarm_screen();
        }

        lvgl_port_unlock();
    }

    delay(10);
}