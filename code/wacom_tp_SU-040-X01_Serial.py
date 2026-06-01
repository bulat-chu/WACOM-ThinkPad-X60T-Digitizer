#!/usr/bin/env python3

## For WACOM DIGITIZER UNIT SU-040-X01 (wich used in ThinkPad X60/X61 Tablet) ##

import serial
from evdev import UInput, AbsInfo, ecodes as e

# --- Настройки ---
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 19200
WACOM4_PACKET_LEN = 7

MAX_X = 6144
MAX_Y = 4608
MAX_PRESSURE = 255

#  ┌─────────────────────┬───────────────────────┐
#  │ Датчик (SU-040-X01) │ UART                  │
#  ├─────────────────────┼───────────────────────┤
#  │ TX                  │ RX                    │
#  │ RX                  │ TX                    │
#  │ GND                 │ GND                   │
#  │ VCC (3.3В)          │ 3V3                   │
#  │ RESET               │ NC                    │
#  │ SLEEP               │ GND via 4.7-10kOhm    │
#  │ PROXIMITY           │ NC                    │
#  └─────────────────────┴───────────────────────┘

#  HID REPORT DESCRIPTOR — Digitizer / Stylus
#
#  Byte 0:
#    bit0 = Tip
#    bit1 = Side Button 1
#    bit2 = Side Button 2 / Eraser
#    bit3 = Reserved         (перо перевёрнуто — ластик в зоне)
#    bit4 = Reserved
#    bit5 = Proximity
#    bit6 = Always 0
#    bit7 = Always 1
#  Byte 1-2: X (uint16 LE)
#  Byte 3-4: Y (uint16 LE)
#  Byte 5:   Pressure (uint8, 0–255)

def open_serial():
    return serial.Serial(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=0.05)

def create_uinput():
    capabilities = {
        e.EV_KEY: [
            e.BTN_TOOL_PEN,
            e.BTN_TOOL_RUBBER,
            e.BTN_TOUCH,
            e.BTN_STYLUS,
            e.BTN_STYLUS2
        ],
        e.EV_ABS: [
            (e.ABS_X,        AbsInfo(value=0, min=0, max=MAX_X,        fuzz=0, flat=0, resolution=1)),
            (e.ABS_Y,        AbsInfo(value=0, min=0, max=MAX_Y,        fuzz=0, flat=0, resolution=1)),
            (e.ABS_PRESSURE, AbsInfo(value=0, min=0, max=MAX_PRESSURE, fuzz=0, flat=0, resolution=0)),
        ],
    }
    return UInput(capabilities, name="WACOM SU-040-X01 (Serial)", version=0x1)

def parse_wacom4_7byte(packet):
    header = packet[0]

    prox       = bool(header & 0x20)  # бит 5
    is_eraser  = bool(header & 0x04)  # бит 2
    side_btn   = bool(header & 0x02)  # бит 1
    tip        = bool(header & 0x01)  # бит 0

    x        = (packet[1] << 7) | packet[2]
    y        = (packet[3] << 7) | packet[4]
    pressure = ((packet[6] & 0x01) << 7) | packet[5]

    return {
        "prox":       prox,
        "is_eraser":  is_eraser,
        "side_button": side_btn,
        "tip":        tip,
        "x":          x,
        "y":          y,
        "pressure":   pressure,
    }

def release_tool(ui, active_tool):
    """Отправить событие ухода пера из зоны."""
    ui.write(e.EV_KEY, e.BTN_TOUCH,   0)
    ui.write(e.EV_KEY, e.BTN_STYLUS,  0)
    ui.write(e.EV_KEY, e.BTN_STYLUS2, 0)
    ui.write(e.EV_ABS, e.ABS_PRESSURE, 0)
    ui.write(e.EV_KEY, active_tool,    0)
    ui.syn()

def main():
    print(f"Driver started on {SERIAL_PORT} (Wacom IV 7-byte mode)")
    try:
        ser = open_serial()
        ui  = create_uinput()
    except Exception as ex:
        print(f"Init error: {ex}")
        return

    buf         = bytearray()
    active_tool = None  # None = перо вне зоны

    try:
        while True:
            data = ser.read(16)
            if data:
                buf.extend(data)

            # Обработка накопленного буфера
            while len(buf) >= WACOM4_PACKET_LEN:
                # Синхронизация: бит 7 должен быть взведён
                if not (buf[0] & 0x80):
                    buf.pop(0)
                    continue

                pkt = parse_wacom4_7byte(buf[:WACOM4_PACKET_LEN])
                buf = buf[WACOM4_PACKET_LEN:]

                if not pkt["prox"]:
                    # Перо вышло из зоны — устройство само сообщило об этом
                    if active_tool is not None:
                        release_tool(ui, active_tool)
                        active_tool = None
                    continue

                # Перо в зоне: определяем/подтверждаем тип инструмента
                new_tool = e.BTN_TOOL_RUBBER if pkt["is_eraser"] else e.BTN_TOOL_PEN
                if active_tool != new_tool:
                    if active_tool is not None:
                        # Смена инструмента (перевернули перо)
                        release_tool(ui, active_tool)
                    active_tool = new_tool
                    ui.write(e.EV_KEY, active_tool, 1)

                # Координаты и давление
                ui.write(e.EV_ABS, e.ABS_X,        pkt["x"])
                ui.write(e.EV_ABS, e.ABS_Y,        pkt["y"])
                ui.write(e.EV_ABS, e.ABS_PRESSURE, pkt["pressure"])

                # Касание
                if active_tool == e.BTN_TOOL_RUBBER:
                    is_touch = pkt["pressure"] > 0 or pkt["tip"]
                else:
                    is_touch = pkt["tip"]

                ui.write(e.EV_KEY, e.BTN_TOUCH,   1 if is_touch          else 0)
                ui.write(e.EV_KEY, e.BTN_STYLUS,  1 if pkt["side_button"] else 0)
                ui.write(e.EV_KEY, e.BTN_STYLUS2, 0)  # резерв для второй кнопки

                ui.syn()

    except KeyboardInterrupt:
        print("\nStopping driver...")
    finally:
        if active_tool is not None:
            release_tool(ui, active_tool)
        ser.close()
        ui.close()

if __name__ == "__main__":
    main()
