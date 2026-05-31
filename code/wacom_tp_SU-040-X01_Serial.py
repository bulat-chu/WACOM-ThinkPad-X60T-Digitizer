#!/usr/bin/env python3

## For WACOM DIGITIZER UNIT SU-040-X01 (wich used in ThinkPad X60/X61 Tablet) ##

import serial
import evdev
from evdev import UInput, AbsInfo, ecodes as e
import time

# --- Настройки на основе реальных тестов устройства ---
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 19200
WACOM4_PACKET_LEN = 7

# Координатная сетка
MAX_X = 6144
MAX_Y = 4608
MAX_PRESSURE = 127  # Датчик выдает 7 бит давления

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
    return UInput(capabilities, name="WACOM DIGITIZER UNIT SU-1208E-01X (Serial)", version=0x1)

def parse_wacom4_7byte(packet):
    header = packet[0]
    
    x = (packet[1] << 7) | packet[2]
    y = (packet[3] << 7) | packet[4]
    
    pressure = ((packet[6] & 0x01) << 7) | packet[5]

    is_eraser = (header == 0xa4)
    tip = header in [0xa1, 0xa3, 0xa4]
    side_button = header in [0xa2, 0xa3]

    return {
        "x": x, 
        "y": y, 
        "pressure": pressure,
        "is_eraser": is_eraser,
        "tip": tip, 
        "side_button": side_button
    }

def main():
    print(f"Driver started on {SERIAL_PORT} (Wacom IV 7-byte mode)")
    try:
        ser = open_serial()
        ui = create_uinput()
    except Exception as ex:
        print(f"Init error: {ex}")
        return

    buf = bytearray()
    active_tool = None
    last_packet_time = time.time()

    try:
        while True:
            # Читаем данные из COM-порта
            data = ser.read(16)
            
            if data:
                buf.extend(data)
                last_packet_time = time.time() # Обновляем время активности пера

            # Таймаут отсутствия данных: если пакетов нет дольше 0.1 сек — перо вышло из зоны
            if active_tool is not None and (time.time() - last_packet_time) > 0.1:
                ui.write(e.EV_KEY, e.BTN_TOUCH, 0)
                ui.write(e.EV_ABS, e.ABS_PRESSURE, 0)
                ui.write(e.EV_KEY, active_tool, 0)
                ui.write(e.EV_KEY, e.BTN_STYLUS, 0)
                ui.write(e.EV_KEY, e.BTN_STYLUS2, 0)
                ui.syn()
                active_tool = None

            # Обработка накопленного буфера пакетов
            while len(buf) >= WACOM4_PACKET_LEN:
                # Синхронизация: заголовок должен начинаться со взведенного 7-го бита (>= 0x80)
                if not (buf[0] & 0x80):
                    buf.pop(0)
                    continue
                
                # Достаем пакет в 7 байт и парсим его
                pkt = parse_wacom4_7byte(buf[:WACOM4_PACKET_LEN])
                buf = buf[WACOM4_PACKET_LEN:]

                # В Wacom IV сам факт разбора валидного пакета означает Proximity = True
                if active_tool is None:
                    if pkt["is_eraser"]:
                        active_tool = e.BTN_TOOL_RUBBER
                    else:
                        active_tool = e.BTN_TOOL_PEN
                    ui.write(e.EV_KEY, active_tool, 1)

                # Передаем обновляемые координаты и давление
                ui.write(e.EV_ABS, e.ABS_X, pkt["x"])
                ui.write(e.EV_ABS, e.ABS_Y, pkt["y"])
                ui.write(e.EV_ABS, e.ABS_PRESSURE, pkt["pressure"])

                # Логика нажатий (касаний экрана)
                if active_tool == e.BTN_TOOL_RUBBER:
                    is_touch = pkt["pressure"] > 0 or pkt["tip"]
                else:
                    is_touch = pkt["tip"]

                # Отправляем состояния кнопок в систему
                ui.write(e.EV_KEY, e.BTN_TOUCH, 1 if is_touch else 0)
                ui.write(e.EV_KEY, e.BTN_STYLUS, 1 if pkt["side_button"] else 0)
                
                # Если появится двухкнопочное перо (в логах была бы отметка), 
                # то BTN_STYLUS2 можно будет повесить сюда. Пока держим в 0.
                ui.write(e.EV_KEY, e.BTN_STYLUS2, 0)

                ui.syn()

    except KeyboardInterrupt:
        print("\nStopping driver...")
    finally:
        ser.close()
        ui.close()

if __name__ == "__main__":
    main()
