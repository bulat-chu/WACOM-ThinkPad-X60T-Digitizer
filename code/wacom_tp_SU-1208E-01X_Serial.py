#!/usr/bin/env python3

## For WACOM DIGITIZER UNIT SU-1208E-01X (wich used in ThinkPad X60 Tablet) ##

import serial
import evdev
from evdev import UInput, AbsInfo, ecodes as e

# --- Config ---
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 38400

MAX_X = 6144
MAX_Y = 4608
MAX_PRESSURE = 255

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
    return UInput(capabilities, name="WACOM DIGITIZER UNIT SU-1208E-01X", version=0x1)

def parse_isdv4_8byte(packet):
    header = packet[0]
    proximity = bool(header & 0x20)
    tip       = bool(header & 0x01)
    side1     = bool(header & 0x02)
    sw2_bit   = bool(header & 0x04)

    x = (packet[1] << 7) | packet[2]
    y = (packet[3] << 7) | packet[4]
    pressure = ((packet[6] & 0x07) << 7) | (packet[5] & 0x7F)

    is_eraser_physically = sw2_bit and not tip

    return {
        "x": x, "y": y, "pressure": pressure,
        "proximity": proximity, 
        "is_eraser_physically": is_eraser_physically,
        "tip": tip, "side1": side1, "sw2_bit": sw2_bit
    }

def main():
    print(f"Driver on {SERIAL_PORT}")
    try:
        ser = open_serial()
        ui = create_uinput()
    except Exception as ex:
        print(f"Init error: {ex}"); return

    buf = bytearray()
    active_tool = None

    try:
        while True:
            data = ser.read(16)
            if not data: continue
            buf.extend(data)

            while len(buf) >= 8:
                if not (buf[0] & 0x80):
                    buf.pop(0); continue
                
                pkt = parse_isdv4_8byte(buf[:8])
                buf = buf[8:]

                if pkt["proximity"]:
                    if active_tool is None:
                        if pkt["is_eraser_physically"]:
                            active_tool = e.BTN_TOOL_RUBBER
                        else:
                            active_tool = e.BTN_TOOL_PEN
                        ui.write(e.EV_KEY, active_tool, 1)

                    ui.write(e.EV_ABS, e.ABS_X, pkt["x"])
                    ui.write(e.EV_ABS, e.ABS_Y, pkt["y"])
                    ui.write(e.EV_ABS, e.ABS_PRESSURE, pkt["pressure"])

                    if active_tool == e.BTN_TOOL_RUBBER:
                        is_touch = pkt["pressure"] > 0
                    else:
                        is_touch = pkt["tip"]
                        is_btn2 = pkt["sw2_bit"] and pkt["tip"]
                        ui.write(e.EV_KEY, e.BTN_STYLUS2, 1 if is_btn2 else 0)

                    ui.write(e.EV_KEY, e.BTN_TOUCH, 1 if is_touch else 0)
                    ui.write(e.EV_KEY, e.BTN_STYLUS, 1 if pkt["side1"] else 0)

                else:
                    if active_tool is not None:
                        ui.write(e.EV_KEY, e.BTN_TOUCH, 0)
                        ui.write(e.EV_ABS, e.ABS_PRESSURE, 0)
                        ui.write(e.EV_KEY, active_tool, 0)
                        active_tool = None

                ui.syn()

    except KeyboardInterrupt:
        print("\nStopping driver...")
    finally:
        ser.close(); ui.close()

if __name__ == "__main__":
    main()
