// =============================================================================
//  WACOM ISDV4 → USB HID Digitizer для ESP32-S3 SuperMini
//  Датчик: WACOM DIGITIZER UNIT SU-1208E-01X (ThinkPad X6x Tablet)
// =============================================================================
//
//  ПОДКЛЮЧЕНИЕ (всё напрямую, датчик 3.3В):
//  ┌─────────────────────┬───────────────────────┐
//  │ Датчик (SU-1208E)   │ ESP32-S3 SuperMini     │
//  ├─────────────────────┼───────────────────────┤
//  │ TX                  │ GPIO4  (RX1)           │
//  │ RX                  │ GPIO5  (TX1)           │
//  │ GND                 │ GND                    │
//  │ VCC (3.3В)          │ 3V3                    │
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
#define UART_RX_PIN   4
#define UART_TX_PIN   5

// --- Протокол ISDV4 ---
#define ISDV4_BAUD        38400
#define ISDV4_CMD_STOP    0x30
#define ISDV4_CMD_QUERY   '*'
#define ISDV4_CMD_START   0x31
#define ISDV4_QUERY_LEN   11
#define ISDV4_PACKET_LEN  8

// --- Лимиты (как в оригинале) ---
#define DEFAULT_MAX_X        6144
#define DEFAULT_MAX_Y        4608
#define MAX_PRESSURE         255

uint16_t sensor_max_x = DEFAULT_MAX_X;
uint16_t sensor_max_y = DEFAULT_MAX_Y;

// =============================================================================
//  HID REPORT DESCRIPTOR — Digitizer / Stylus
//
//  Byte 0: buttons
//    bit0 = Tip Switch
//    bit1 = Barrel Switch  (side1, боковая кнопка)
//    bit2 = Eraser
//    bit3 = Invert         (перо перевёрнуто — ластик в зоне)
//    bit4 = In Range
//    bit5 = Barrel Switch2 (sw2_bit + tip)
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
      0x09, 0x46,  // Tablet Pick (используем как Barrel Switch 2)
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
  uint8_t  buttons;   // bit0=tip, bit1=barrel, bit2=eraser, bit3=invert, bit4=inRange, bit5=barrel2
  uint16_t x;
  uint16_t y;
  uint8_t  pressure;  // 8 бит
};

// Результат парсинга пакета
struct PenState {
  uint16_t x, y;
  uint8_t  pressure;  // raw, 0–255 (обрезаем старшие биты ибо иначе работает неправильно)
  bool proximity;
  bool tip, side1, sw2_bit;
  bool is_eraser;     // sw2_bit && !tip
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

// active_tool: 0=нет, 1=перо, 2=ластик — фиксируется при входе в proximity
uint8_t active_tool = 0;
#define TOOL_NONE   0
#define TOOL_PEN    1
#define TOOL_ERASER 2

// =============================================================================
//  ИНИЦИАЛИЗАЦИЯ ДАТЧИКА
// =============================================================================
void isdv4_init() {
  Serial1.write(ISDV4_CMD_STOP);
  delay(250);
  while (Serial1.available()) Serial1.read();

  Serial1.write(ISDV4_CMD_QUERY);
  uint8_t resp[ISDV4_QUERY_LEN];
  uint32_t t = millis();
  uint8_t got = 0;
  while (got < ISDV4_QUERY_LEN && (millis() - t) < 500) {
    if (Serial1.available()) resp[got++] = Serial1.read();
  }
  if (got >= ISDV4_QUERY_LEN) {
    uint16_t mx = ((resp[2] & 0x7F) << 7) | (resp[3] & 0x7F);
    uint16_t my = ((resp[4] & 0x7F) << 7) | (resp[5] & 0x7F);
    if (mx) sensor_max_x = mx;
    if (my) sensor_max_y = my;
  }

  Serial1.write(ISDV4_CMD_START);
  delay(50);
}

// =============================================================================
//  ПАРСИНГ ПАКЕТА
// =============================================================================
PenState parse_isdv4(uint8_t* pkt) {
  PenState s;
  s.proximity = pkt[0] & 0x20;
  s.tip       = pkt[0] & 0x01;
  s.side1     = pkt[0] & 0x02;
  s.sw2_bit   = pkt[0] & 0x04;

  s.x = ((uint16_t)(pkt[1] & 0x7F) << 7) | (pkt[2] & 0x7F);
  s.y = ((uint16_t)(pkt[3] & 0x7F) << 7) | (pkt[4] & 0x7F);

  // Давление 10 бит raw, но берём только младшие 8 (как MAX_PRESSURE=255)
  uint16_t raw_p = ((uint16_t)(pkt[6] & 0x07) << 7) | (pkt[5] & 0x7F);
  s.pressure = (raw_p > 255) ? 255 : (uint8_t)raw_p;

  // Ластик: sw2_bit активен И перо НЕ касается
  s.is_eraser = s.sw2_bit && !s.tip;

  return s;
}

// =============================================================================
//  SETUP
// =============================================================================
void setup() {
  USB.productName("WACOM DIGITIZER UNIT SU-1208E-01X (ThinkPad X6x Tablet)");
  USB.manufacturerName("WACOM");
  wacom.begin();
  wacom.hid.begin();
  USB.begin();

  delay(1000);

  Serial1.begin(ISDV4_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  delay(100);
  isdv4_init();
}

// =============================================================================
//  LOOP
// =============================================================================
uint8_t buf[ISDV4_PACKET_LEN * 2];
uint8_t buf_len = 0;

void loop() {
  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    if (buf_len == 0 && !(b & 0x80)) continue;
    buf[buf_len++] = b;

    if (buf_len >= ISDV4_PACKET_LEN) {
      PenState pen = parse_isdv4(buf);
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
          // Ластик: касание по давлению, invert бит
          btns |= (1 << 3);  // Invert
          btns |= (1 << 2);  // Eraser
          if (pen.pressure > 0) btns |= (1 << 0);  // Tip (касание)
        } else {
          // Перо
          if (pen.tip)     btns |= (1 << 0);  // Tip Switch
          if (pen.side1)   btns |= (1 << 1);  // Barrel Switch
          // BTN_STYLUS2: sw2_bit активен И есть касание (как в оригинале)
          if (pen.sw2_bit && pen.tip) btns |= (1 << 5);
        }

        pen_report.buttons = btns;

      } else {
        // Перо вышло из зоны — сброс
        active_tool = TOOL_NONE;
        pen_report.buttons  = 0;
        pen_report.pressure = 0;
      }

      wacom.sendReport(&pen_report);
    }
  }
}
