// =============================================================================
//  WACOM/ISDV4 → USB HID Digitizer для ESP32-S3 SuperMini
//  Панель: WACOM DIGITIZER UNIT SU5E-12W13AS-X07 (ThinkPad X200/X201 Tablet)
//
//  ВАЖНО: это протокол ISDV4 (embedded Tablet PC digitizer).
//  Пакет пера — 9 байт (byte7/8 — X/Y tilt, у этой
//  панели наклон не поддерживается и всегда 0). Младшие биты X и Y хранятся не
//  в byte2/byte4 целиком, а размазаны: byte2/byte4 дают только биты [6:2],
//  а младшие 2 бита X и Y лежат в byte6 (см. parse_wacom4).
//  Источник формата: https://github.com/linuxwacom/input-wacom/wiki/For-Developers:-ISDV4-Protocol
//
//  СОН/ПРОБУЖДЕНИЕ:
//  - При USB suspend/disconnect: панели отправляется '0' и поднимается SLEEP,
//    данные игнорируются, HID-отчёты не отправляются.
//  - При resume/mount: снимается SLEEP, импульс RESET, панели отправляется '1',
//    состояние пера сбрасывается.
//  - После resume шина "передёргивается" (tud_disconnect/tud_connect), что
//    заставляет хост заново энумерировать устройство (аналог usbreset).
// =============================================================================
//
//  ПОДКЛЮЧЕНИЕ:
//  ┌─────────────────────┬────────────────────────────┐
//  │  SU5E-12W13AS-X07   │ ESP32-S3 SuperMini         │
//  ├─────────────────────┼────────────────────────────┤
//  │ TX                  │ GPIO4  (RX1)               │
//  │ RX                  │ GPIO5  (TX1)               │
//  │ GND                 │ GND                        │
//  │ VCC (3.3В)          │ 3V3                        │
//  │ RESET               │ GPIO6 (в покое — на GND    │
//  │                     │  через 4.7–10 кОм, если    │
//  │                     │  GPIO6 не подключён)       │
//  │ SLEEP               │ GPIO7                      │
//  │ PROXIMITY           │ GPIO8 (необязательно)      │
//  └─────────────────────┴────────────────────────────┘
//
//  НАСТРОЙКА Arduino IDE:
//  - USB Mode:         "USB-OTG (TinyUSB)"
//  - USB CDC On Boot:  "Disabled"
//  - CPU Frequency:    40 MHz
//  - Плата:            "ESP32S3 Dev Module", в Board Manager "esp32" by Espressif Systems, v2.0.17
//  - Adafruit TinyUSB Library НЕ нужна
// =============================================================================

#include "USB.h"
#include "USBHID.h"
#include "WiFi.h"   // только чтобы явно выключить радио, WiFi тут не используется
#include "tusb.h"   // tud_hid_n_ready / tud_hid_n_report / tud_mounted / tud_suspended / tud_connect

// --- Пины UART ---
#define UART_RX_PIN     4
#define UART_TX_PIN     5

// --- Управляющие линии панели ---
#define PANEL_RES_PIN         6      // RESET
#define PANEL_SLP_PIN         7      // SLEEP
#define PANEL_PDCT_PIN        8      // PROXIMITY (выход панели), -1 = не использовать

// Полярности — по аналогии с SU-040-X01, ПРОВЕРИТЬ на железе
#define PANEL_SLP_SLEEP_LVL   HIGH   // уровень на SLEEP, при котором панель спит
#define PANEL_RES_ACTIVE_LVL  LOW    // уровень на RESET, при котором панель в сбросе
#define PANEL_WAKE_MS         100    // пауза после выхода из сна/сброса

// --- Энергосбережение ESP32 во сне ---
#define WORK_CPU_MHZ    40   // рабочая частота (как в настройках IDE)
#define SLEEP_CPU_MHZ   0    // частота во сне; 0 = не менять
#define SLEEP_LOOP_MS   50   // период опроса флага во сне

// --- Протокол ISDV4 ---
#define WACOM4_BAUD       19200
#define WACOM4_PACKET_LEN 9   // 9 байт (byte7/8 — X/Y tilt, не используются)
#define WACOM4_CMD_START  '1'
#define WACOM4_CMD_STOP   '0'
#define PANEL_STOP_MS     50     // пауза между '0' и SLEEP

// --- Если отчёт не уходит дольше этого времени при живом USB — перезагрузка ---
#define STUCK_TIMEOUT_MS  5000

// --- Передёргивание шины после сна ---
#define RESUME_SETTLE_MS  500   // ждём, пока хост проснётся
#define RECONNECT_MS      300   // длительность "отключения" шины

// --- Лимиты координат (единственный источник правды: дескриптор берёт их отсюда) ---
#define MAX_X        6576
#define MAX_Y        4128
#define MAX_PRESSURE 255

// Разбиение числа на байты для HID-дескриптора (little-endian)
#define LO8(v) ((v) & 0xFF)
#define HI8(v) (((v) >> 8) & 0xFF)

// =============================================================================
//  HID REPORT DESCRIPTOR — Digitizer / Stylus
//
//  Byte 0:
//    bit0 = Tip
//    bit1 = Side Button 1
//    bit2 = Side Button 2 / Eraser
//    bit3 = Invert           (перо перевёрнуто — ластик в зоне)
//    bit4 = In Range         (proximity)
//    bit5 = Tablet Pick (резерв)
//    bit6-7 = padding
//  Byte 1-2: X (uint16 LE)
//  Byte 3-4: Y (uint16 LE)
//  Byte 5:   Pressure (uint8, 0–255)
// =============================================================================
static const uint8_t hid_descriptor[] = {
    0x05, 0x0D,
    0x09, 0x01,
    0xA1, 0x01,
    0x09, 0x20,
    0xA1, 0x00,
    // Кнопки: 6 бит + 2 padding
    0x09, 0x42,  // Tip Switch
    0x09, 0x44,  // Barrel Switch
    0x09, 0x45,  // Eraser
    0x09, 0x3C,  // Invert
    0x09, 0x32,  // In Range
    0x09, 0x46,  // Tablet Pick (Barrel Switch 2, резерв)
    0x15, 0x00,
    0x25, 0x01,
    0x75, 0x01,
    0x95, 0x06,
    0x81, 0x02,
    0x95, 0x02,  // padding до байта
    0x81, 0x03,
    // X
    0x05, 0x01,
    0x09, 0x30,
    0x15, 0x00,
    0x26, LO8(MAX_X), HI8(MAX_X),   // Logical Max
    0x46, LO8(MAX_X), HI8(MAX_X),   // Physical Max
    0x55, 0x0D,
    0x65, 0x11,
    0x75, 0x10,
    0x95, 0x01,
    0x81, 0x02,
    // Y
    0x09, 0x31,
    0x15, 0x00,
    0x26, LO8(MAX_Y), HI8(MAX_Y),   // Logical Max
    0x46, LO8(MAX_Y), HI8(MAX_Y),   // Physical Max
    0x75, 0x10,
    0x95, 0x01,
    0x81, 0x02,
    // Pressure (0–255, 8 бит)
    0x05, 0x0D,
    0x09, 0x30,
    0x15, 0x00,
    0x26, LO8(MAX_PRESSURE), HI8(MAX_PRESSURE),
    0x46, LO8(MAX_PRESSURE), HI8(MAX_PRESSURE),
    0x75, 0x08,
    0x95, 0x01,
    0x81, 0x02,
    0xC0,
    0xC0
};

// Структура репорта — совпадает с дескриптором
struct __attribute__((packed)) PenReport {
    uint8_t  buttons;
    uint16_t x;
    uint16_t y;
    uint8_t  pressure;
};

// Результат парсинга пакета ISDV4
struct PenState {
    bool     proximity;
    bool     is_eraser;
    bool     tip;
    bool     side_button;
    uint16_t x, y;
    uint16_t pressure;
};

// =============================================================================
//  HID устройство
//  Отправка идёт напрямую через TinyUSB (tud_hid_n_report), а не через
//  USBHID::SendReport(): тот берёт внутренний семафор, который навсегда
//  остаётся занятым, если USB уснул во время передачи.
// =============================================================================
class WacomHID : public USBHIDDevice {
public:
    WacomHID() {}

    void begin() {
        hid.addDevice(this, sizeof(hid_descriptor));
    }

    uint16_t _onGetDescriptor(uint8_t* dst) {
        memcpy(dst, hid_descriptor, sizeof(hid_descriptor));
        return sizeof(hid_descriptor);
    }

    // true — отчёт принят стеком USB
    bool sendReport(const PenReport* report) {
        if (!tud_hid_n_ready(0)) return false;   // не подключён / suspend / эндпоинт занят
        return tud_hid_n_report(0, 0, report, sizeof(PenReport));
    }

    USBHID hid;
};

WacomHID wacom;
PenReport pen_report = {};
bool     report_pending   = false;   // есть неотправленное состояние
uint32_t pending_since_ms = 0;

// active_tool: фиксируется при входе в proximity
#define TOOL_NONE   0
#define TOOL_PEN    1
#define TOOL_ERASER 2
uint8_t active_tool = TOOL_NONE;

uint8_t buf[WACOM4_PACKET_LEN * 2];
uint8_t buf_len = 0;

// =============================================================================
//  Сон / пробуждение
// =============================================================================
volatile bool usb_want_awake = true;    // выставляется из USB-событий
volatile bool usb_resumed    = false;   // только по RESUME, не по STARTED
bool panel_awake = false;               // фактическое состояние панели

static void usbEventCallback(void* arg, esp_event_base_t base, int32_t id, void* data) {
    if (base != ARDUINO_USB_EVENTS) return;
    switch (id) {
        case ARDUINO_USB_SUSPEND_EVENT:
        case ARDUINO_USB_STOPPED_EVENT:    // шина сброшена / кабель отключён
            usb_want_awake = false;
            break;
        case ARDUINO_USB_RESUME_EVENT:
            usb_want_awake = true;
            usb_resumed    = true;         // запросить переэнумерацию
            break;
        case ARDUINO_USB_STARTED_EVENT:    // (пере)энумерация
            usb_want_awake = true;
            break;
        default:
            break;
    }
}

static void reset_pen_state() {
    active_tool         = TOOL_NONE;
    pen_report.buttons  = 0;
    pen_report.pressure = 0;
}

static void drain_uart() {
    while (Serial1.available()) Serial1.read();
}

static void panel_sleep() {
    Serial1.write(WACOM4_CMD_STOP);
    Serial1.flush();                 // ждём, пока байт реально уйдёт в UART
    delay(PANEL_STOP_MS);            // даём панели обработать команду
    digitalWrite(PANEL_SLP_PIN, PANEL_SLP_SLEEP_LVL);
    drain_uart();
    buf_len = 0;
    reset_pen_state();
    report_pending = false;          // хост спит — отправлять некому
    panel_awake = false;
#if SLEEP_CPU_MHZ > 0
    setCpuFrequencyMhz(SLEEP_CPU_MHZ);   // снижаем в самом конце
#endif
}

static void panel_wake() {
#if SLEEP_CPU_MHZ > 0
    setCpuFrequencyMhz(WORK_CPU_MHZ);    // возвращаем частоту до всего остального
#endif
    digitalWrite(PANEL_SLP_PIN, !PANEL_SLP_SLEEP_LVL);

    // короткий аппаратный сброс
    digitalWrite(PANEL_RES_PIN, PANEL_RES_ACTIVE_LVL);
    delay(5);
    digitalWrite(PANEL_RES_PIN, !PANEL_RES_ACTIVE_LVL);
    delay(PANEL_WAKE_MS);

    drain_uart();
    buf_len = 0;
    reset_pen_state();
    Serial1.write(WACOM4_CMD_START);
    Serial1.flush();
    panel_awake = true;

    // сообщаем хосту актуальное состояние (перо вне зоны, кнопки отпущены)
    report_pending = true;
    pending_since_ms = millis();
}

// Отправить последнее состояние, если USB готов
static void try_send() {
    if (!report_pending) return;
    if (wacom.sendReport(&pen_report)) {
        report_pending = false;
    }
}

// =============================================================================
//  Разбор пакета пера протокола ISDV4 (9 байт, без наклона):
//
//  byte0: bit7=1(sync) bit6=0(event) bit5=proximity bit2=eraser/side2
//         bit1=side1 bit0=tip
//  byte1: X[13:7]                      (все 7 бит)
//  byte2: биты 2-6 = X[6:2]
//  byte3: Y[13:7]
//  byte4: биты 2-6 = Y[6:2]
//  byte5: Pressure[6:0]
//  byte6: биты 5-6 = X[1:0], биты 3-4 = Y[1:0], биты 0-2 = Pressure[9:7]
//  byte7: Y tilt (не используется, всегда 0)
//  byte8: X tilt (не используется, всегда 0)
// =============================================================================
PenState parse_wacom4(uint8_t* pkt) {
    PenState s;
    s.proximity   = pkt[0] & 0x20;
    s.is_eraser   = pkt[0] & 0x04;
    s.side_button = pkt[0] & 0x02;
    s.tip         = pkt[0] & 0x01;

    uint16_t x_hi  = pkt[1] & 0x7F;           // X[13:7]
    uint16_t x_mid = (pkt[2] >> 2) & 0x1F;    // X[6:2]
    uint16_t x_lo  = (pkt[6] >> 5) & 0x03;    // X[1:0]
    s.x = (x_hi << 7) | (x_mid << 2) | x_lo;

    uint16_t y_hi  = pkt[3] & 0x7F;           // Y[13:7]
    uint16_t y_mid = (pkt[4] >> 2) & 0x1F;    // Y[6:2]
    uint16_t y_lo  = (pkt[6] >> 3) & 0x03;    // Y[1:0]
    s.y = (y_hi << 7) | (y_mid << 2) | y_lo;

    uint16_t p_lo = pkt[5] & 0x7F;            // Pressure[6:0]
    uint16_t p_hi = pkt[6] & 0x07;            // Pressure[9:7]
    s.pressure = (p_hi << 7) | p_lo;          // у этой панели факт. максимум 255

    return s;
}

// =============================================================================
//  SETUP
// =============================================================================
void setup() {
    // Радио (WiFi/BT) в этом проекте не используется вообще — на всякий случай
    // выключаем явно, чтобы точно не жгло лишний ток в простое.
    WiFi.mode(WIFI_OFF);
    btStop();

    // Управляющие линии панели — сразу в безопасное (рабочее) состояние
    pinMode(PANEL_RES_PIN, OUTPUT);
    digitalWrite(PANEL_RES_PIN, !PANEL_RES_ACTIVE_LVL);
    pinMode(PANEL_SLP_PIN, OUTPUT);
    digitalWrite(PANEL_SLP_PIN, !PANEL_SLP_SLEEP_LVL);
#if PANEL_PDCT_PIN >= 0
    pinMode(PANEL_PDCT_PIN, INPUT);
#endif

    // USB HID
    USB.onEvent(usbEventCallback);
    USB.productName("DIGITIZER UNIT SU5E-12W13AS-X07");
    USB.manufacturerName("WACOM");
    wacom.begin();
    wacom.hid.begin();
    USB.begin();

    delay(1000);

    Serial1.setRxBufferSize(256);
    Serial1.begin(WACOM4_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    delay(100);

    panel_wake();   // сброс + явный старт потока координат
}

// =============================================================================
//  LOOP
// =============================================================================
void loop() {
    // --- Переходы сон/пробуждение ---
    bool want = usb_want_awake;
    if (want && !panel_awake)      panel_wake();
    else if (!want && panel_awake) panel_sleep();

    // --- После resume: принудительная переэнумерация, как usbreset ---
    if (usb_resumed) {
        usb_resumed = false;
        delay(RESUME_SETTLE_MS);
        tud_disconnect();
        delay(RECONNECT_MS);
        tud_connect();
    }

    // --- Во сне всё игнорируем ---
    if (!panel_awake) {
        drain_uart();
        delay(SLEEP_LOOP_MS);
        return;
    }

    // --- Чтение UART ---
    while (Serial1.available()) {
        uint8_t b = Serial1.read();

        // Синхронизация: первый байт пакета должен иметь бит 7 = 1
        if (buf_len == 0 && !(b & 0x80)) continue;

        buf[buf_len++] = b;

        if (buf_len >= WACOM4_PACKET_LEN) {
            PenState pen = parse_wacom4(buf);
            buf_len = 0;

            if (pen.proximity) {
                // Фиксируем инструмент при первом появлении в зоне
                if (active_tool == TOOL_NONE) {
                    active_tool = pen.is_eraser ? TOOL_ERASER : TOOL_PEN;
                }

                // Ограничиваем значения лимитами дескриптора
                pen_report.x        = pen.x > MAX_X ? MAX_X : pen.x;
                pen_report.y        = pen.y > MAX_Y ? MAX_Y : pen.y;
                pen_report.pressure = (uint8_t)(pen.pressure > MAX_PRESSURE ? MAX_PRESSURE : pen.pressure);

                uint8_t btns = (1 << 4);  // In Range всегда при proximity

                if (active_tool == TOOL_ERASER) {
                    btns |= (1 << 3);  // Invert
                    btns |= (1 << 2);  // Eraser
                    if (pen.pressure > 0) btns |= (1 << 0);  // Tip (касание по давлению)
                } else {
                    if (pen.tip)         btns |= (1 << 0);  // Tip Switch
                    if (pen.side_button) btns |= (1 << 1);  // Barrel Switch
                    // Barrel Switch 2 — резерв, пока не используется
                }

                pen_report.buttons = btns;

            } else {
                // Перо вышло из зоны — сброс
                active_tool         = TOOL_NONE;
                pen_report.buttons  = 0;
                pen_report.pressure = 0;
            }

            if (!report_pending) pending_since_ms = millis();
            report_pending = true;
            try_send();
        }
    }

    // --- Повторная попытка, если эндпоинт был занят ---
    try_send();

    // --- Страховка: USB жив, но отчёты не уходят ---
    if (report_pending && tud_mounted() && !tud_suspended() &&
        (millis() - pending_since_ms) > STUCK_TIMEOUT_MS) {
        ESP.restart();
    }
}
