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

// --- Пины UART ---
#define UART_RX_PIN     4
#define UART_TX_PIN     5

// --- Протокол ISDV4 ---
#define WACOM4_BAUD       19200
#define WACOM4_PACKET_LEN 9   // 9 байт (byte7/8 — X/Y tilt, не используются)

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

  // USB HID
  USB.productName("DIGITIZER UNIT SU5E-12W13AS-X07");
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

void loop() {
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

      wacom.sendReport(&pen_report);
    }
  }
}
