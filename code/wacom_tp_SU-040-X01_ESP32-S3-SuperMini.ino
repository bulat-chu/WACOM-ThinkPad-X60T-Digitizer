// =============================================================================
//  WACOM IV → USB HID Digitizer для ESP32-S3 SuperMini
//  Панель: WACOM DIGITIZER UNIT SU-040-X01 (ThinkPad X60/X61 Tablet)
// =============================================================================
//
//  ПОДКЛЮЧЕНИЕ:
//  ┌─────────────────────┬───────────────────────┐
//  │ Датчик (SU-040-X01) │ ESP32-S3 SuperMini    │
//  ├─────────────────────┼───────────────────────┤
//  │ TX                  │ GPIO4  (RX1)          │
//  │ RX                  │ GPIO5  (TX1)          │
//  │ GND                 │ GND                   │
//  │ VCC (3.3В)          │ 3V3                   │
//  │ RESET               │ GPIO6                 │
//  │ SLEEP               │ GPIO7                 │
//  │ PROXIMITY           │ GPIO8                 │
//  └─────────────────────┴───────────────────────┘
//
//  Кнопки (подтянуты к 3.3В, нажатие — LOW):
//  ┌─────────────────────┬───────────────────────┐
//  │ Кнопка RESET        │ GPIO9                 │
//  │ Кнопка SLEEP        │ GPIO10                │
//  └─────────────────────┴───────────────────────┘
//
//  НАСТРОЙКА Arduino IDE:
//  - USB Mode:         "USB-OTG (TinyUSB)"
//  - USB CDC On Boot:  "Disabled"
//  - CPU Frequency:    80 MHz
//  - Плата:            "ESP32S3 Dev Module", в Board Manager "esp32" by Espressif Systems, v2.0.17
//  - Adafruit TinyUSB Library НЕ нужна
// =============================================================================

#include "USB.h"
#include "USBHID.h"

// --- Пины UART ---
#define UART_RX_PIN     4
#define UART_TX_PIN     5

// --- Пины управления панелью ---
#define PIN_RESET       6   // Выход: сброс панели (активный уровень — см. ниже)
#define PIN_SLEEP       7   // Выход: sleep панели, LOW = активна
#define PIN_PROXIMITY   8   // Вход:  сигнал proximity из панели (индикатор)

// --- Пины кнопок ---
#define PIN_BTN_RESET   9   // Кнопка сброса панели
#define PIN_BTN_SLEEP   10  // Кнопка вкл/выкл панели

// --- Протокол Wacom IV ---
#define WACOM4_BAUD       19200
#define WACOM4_PACKET_LEN 7

// --- Лимиты координат ---
#define MAX_X        6144
#define MAX_Y        4608
#define MAX_PRESSURE 255   // 7 бит давления + 1 бит из byte 6

// =============================================================================
//  HID REPORT DESCRIPTOR — Digitizer / Stylus
//
//  Byte 0:
//    bit0 = Tip
//    bit1 = Side Button 1
//    bit2 = Side Button 2 / Eraser
//    bit3 = Reserved         (перо перевёрнуто — ластик в зоне)
//    bit4 = Reserved
//    bit5 = Proximity
//    bit6 = Always 0
//    bit7 = Always 1
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
      0x27, 0x00, 0x18, 0x00, 0x00,  // max 6144
      0x47, 0x00, 0x18, 0x00, 0x00,
      0x55, 0x0D,
      0x65, 0x11,
      0x75, 0x10,
      0x95, 0x01,
      0x81, 0x02,
      // Y
      0x09, 0x31,
      0x15, 0x00,
      0x27, 0x00, 0x12, 0x00, 0x00,  // max 4608
      0x47, 0x00, 0x12, 0x00, 0x00,
      0x75, 0x10,
      0x95, 0x01,
      0x81, 0x02,
      // Pressure (0–255, 8 бит)
      0x05, 0x0D,
      0x09, 0x30,
      0x15, 0x00,
      0x26, 0xFF, 0x00,
      0x46, 0xFF, 0x00,
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

// Результат парсинга пакета Wacom IV
struct PenState {
  bool     proximity;
  bool     is_eraser;
  bool     tip;
  bool     side_button;
  uint16_t x, y;
  uint8_t  pressure;
};

// =============================================================================
//  HID устройство
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

  bool sendReport(PenReport* report) {
    return hid.SendReport(0, report, sizeof(PenReport));
  }

  USBHID hid;
};

WacomHID wacom;
PenReport pen_report = {};

// active_tool: фиксируется при входе в proximity
#define TOOL_NONE   0
#define TOOL_PEN    1
#define TOOL_ERASER 2
uint8_t active_tool = TOOL_NONE;

// Состояние sleep (панель активна по умолчанию)
bool panel_sleeping = false;

// =============================================================================
//  УПРАВЛЕНИЕ ПАНЕЛЬЮ
// =============================================================================
void panel_reset() {
  digitalWrite(PIN_RESET, LOW);   // Активный LOW
  // digitalWrite(PIN_RESET, HIGH);  // Активный HIGH — раскомментировать если наоборот
  delay(10);
  digitalWrite(PIN_RESET, HIGH);  // Активный LOW
  // digitalWrite(PIN_RESET, LOW);   // Активный HIGH — раскомментировать если наоборот
  delay(100);
}

void panel_sleep_on() {
  digitalWrite(PIN_SLEEP, HIGH);  // Отпускаем — панель засыпает
  panel_sleeping = true;
}

void panel_sleep_off() {
  digitalWrite(PIN_SLEEP, LOW);   // Притягиваем к земле — панель активна
  panel_sleeping = false;
  delay(100);
}

// =============================================================================
PenState parse_wacom4(uint8_t* pkt) {
  PenState s;
  s.proximity   = pkt[0] & 0x20;  // бит 5
  s.is_eraser   = pkt[0] & 0x04;  // бит 2
  s.side_button = pkt[0] & 0x02;  // бит 1
  s.tip         = pkt[0] & 0x01;  // бит 0

  s.x = ((uint16_t)(pkt[1] & 0x7F) << 7) | (pkt[2] & 0x7F);
  s.y = ((uint16_t)(pkt[3] & 0x7F) << 7) | (pkt[4] & 0x7F);

  uint8_t pressure = ((pkt[6] & 0x01) << 7) | (pkt[5] & 0x7F);
  s.pressure = pressure;

  return s;
}

// =============================================================================
//  SETUP
// =============================================================================
void setup() {
  // Пины управления панелью
  pinMode(PIN_RESET,     OUTPUT);
  pinMode(PIN_SLEEP,     OUTPUT);
  pinMode(PIN_PROXIMITY, INPUT);

  // Пины кнопок (внутренняя подтяжка к 3.3В)
  pinMode(PIN_BTN_RESET, INPUT_PULLUP);
  pinMode(PIN_BTN_SLEEP, INPUT_PULLUP);

  // Панель активна сразу при включении
  panel_sleep_off();

  // Сброс панели при старте
  panel_reset();

  // USB HID
  USB.productName("DIGITIZER UNIT SU-040-X01");
  USB.manufacturerName("WACOM");
  wacom.begin();
  wacom.hid.begin();
  USB.begin();

  delay(1000);

  Serial1.begin(WACOM4_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  delay(100);
}

// =============================================================================
//  LOOP
// =============================================================================
uint8_t buf[WACOM4_PACKET_LEN * 2];
uint8_t buf_len = 0;

// Для debounce кнопок
uint32_t btn_reset_last = 0;
uint32_t btn_sleep_last = 0;
#define DEBOUNCE_MS 50

void loop() {
  // --- Кнопка RESET ---
  if (digitalRead(PIN_BTN_RESET) == LOW && (millis() - btn_reset_last) > DEBOUNCE_MS) {
    btn_reset_last = millis();
    panel_reset();
  }

  // --- Кнопка SLEEP (toggle) ---
  if (digitalRead(PIN_BTN_SLEEP) == LOW && (millis() - btn_sleep_last) > DEBOUNCE_MS) {
    btn_sleep_last = millis();
    if (panel_sleeping) {
      panel_sleep_off();
    } else {
      panel_sleep_on();
    }
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

        pen_report.x        = pen.x;
        pen_report.y        = pen.y;
        pen_report.pressure = pen.pressure;

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

      wacom.sendReport(&pen_report);
    }
  }
}
