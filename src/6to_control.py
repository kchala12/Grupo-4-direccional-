#!/usr/bin/env python3
"""
Control Robot Diferencial JGY370 - Tiempo Real (WASD)
Comunicación USB Serial con ESP32
----------------------------------------------
Instalar: pip install pyserial
Uso:
  python3 motor_control_usb.py
  python3 motor_control_usb.py --port /dev/ttyUSB0
"""

import serial
import serial.tools.list_ports
import threading
import time
import argparse
import sys
import tty
import termios

class C:
    OK    = "\033[92m"
    ERR   = "\033[91m"
    INFO  = "\033[94m"
    WARN  = "\033[93m"
    RESET = "\033[0m"
    BOLD  = "\033[1m"
    CLEAR = "\033[2J\033[H"

def find_esp32_port():
    known_chips = ["CP210", "CH340", "CH341", "FTDI", "FT232", "USB Serial", "ESP32"]
    for port in serial.tools.list_ports.comports():
        desc = (port.description or "").upper()
        mfr  = (port.manufacturer or "").upper()
        if any(chip.upper() in desc or chip.upper() in mfr for chip in known_chips):
            return port.device
    ports = serial.tools.list_ports.comports()
    return ports[0].device if ports else None

def reader_thread(ser, stop_event, esp_lines):
    """Acumula las respuestas de la ESP32 para mostrarlas en el HUD."""
    while not stop_event.is_set():
        try:
            if ser.in_waiting:
                line = ser.readline().decode("utf-8", errors="replace").strip()
                if line and not line.startswith(">"):
                    esp_lines.append(line)
                    if len(esp_lines) > 6:
                        esp_lines.pop(0)
        except serial.SerialException:
            break
        except Exception:
            pass
        time.sleep(0.02)

def get_key():
    """Lee un solo carácter del teclado sin necesitar Enter."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        # Detectar teclas de flecha (secuencia ESC [ A/B/C/D)
        if ch == '\x1b':
            ch2 = sys.stdin.read(1)
            if ch2 == '[':
                ch3 = sys.stdin.read(1)
                return {'A': 'w', 'B': 's', 'C': 'd', 'D': 'a'}.get(ch3, '')
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

def draw_hud(current_cmd, speed, esp_lines, last_key):
    """Dibuja el HUD de control en tiempo real."""
    arrows = {
        "adelante":  "      [ W / ↑ ]      \n   avanzando...   ",
        "atras":     "      [ S / ↓ ]      \n  retrocediendo...  ",
        "derecha":   "      [ D / → ]      \n  girando derecha...  ",
        "izquierda": "      [ A / ← ]      \n  girando izquierda... ",
        "stop":      "                     \n       parado        ",
    }
    state_color = {
        "adelante":  C.OK,
        "atras":     C.WARN,
        "derecha":   C.INFO,
        "izquierda": C.INFO,
        "stop":      C.RESET,
    }
    color = state_color.get(current_cmd, C.RESET)
    arrow = arrows.get(current_cmd, "       parado        ")

    print(C.CLEAR, end="")
    print(f"{C.BOLD}╔══════════════════════════════════════════╗")
    print(f"║   Robot Diferencial JGY370 - WASD        ║")
    print(f"╚══════════════════════════════════════════╝{C.RESET}")
    print()
    print(f"  {C.BOLD}Velocidad:{C.RESET} {C.OK}{speed}%{C.RESET}   "
          f"(+/- para ajustar, 0-9 para preset)")
    print()
    print(f"  {C.BOLD}Teclas:{C.RESET}")
    print(f"         {C.BOLD}W / ↑{C.RESET}  — Adelante")
    print(f"   {C.BOLD}A / ←{C.RESET}  {C.BOLD}S / ↓{C.RESET}  {C.BOLD}D / →{C.RESET}  — Izq / Atrás / Der")
    print(f"         {C.BOLD}Espacio{C.RESET} — Stop")
    print(f"         {C.BOLD}+  /  -{C.RESET} — Velocidad +10 / -10")
    print(f"         {C.BOLD}0..9{C.RESET}   — Velocidad 0%..90%  (x10)")
    print(f"         {C.BOLD}Q{C.RESET}      — Salir")
    print()
    print(f"  {C.BOLD}Estado:{C.RESET} {color}{arrow}{C.RESET}")
    print()
    print(f"  {C.BOLD}Última tecla:{C.RESET} {repr(last_key)}")
    print()
    print(f"  {C.BOLD}ESP32:{C.RESET}")
    for line in esp_lines[-4:]:
        print(f"    {C.INFO}{line}{C.RESET}")
    sys.stdout.flush()

def wasd_loop(ser, stop_event, esp_lines):
    """Bucle principal de control en tiempo real con WASD."""
    speed      = 50
    current    = "stop"
    last_key   = ""
    last_sent  = "stop"

    KEY_MAP = {
        'w': 'adelante', 'W': 'adelante',
        's': 'atras',    'S': 'atras',
        'a': 'izquierda','A': 'izquierda',
        'd': 'derecha',  'D': 'derecha',
        ' ': 'stop',
    }

    def send(cmd):
        ser.write((cmd + "\n").encode("utf-8"))
        time.sleep(0.02)

    def set_speed(v):
        nonlocal speed
        speed = max(0, min(100, v))
        send(f"velocidad {speed}")

    draw_hud(current, speed, esp_lines, last_key)

    while not stop_event.is_set():
        key = get_key()
        last_key = key

        # Salir
        if key in ('q', 'Q', '\x03'):   # q o Ctrl+C
            send("stop")
            break

        # Movimiento
        if key in KEY_MAP:
            cmd = KEY_MAP[key]
            if cmd != last_sent:
                send(cmd)
                last_sent = cmd
            current = cmd

        # Velocidad con + / -
        elif key == '+' or key == '=':
            set_speed(speed + 10)
        elif key == '-' or key == '_':
            set_speed(speed - 10)

        # Presets 0-9 → 0%..90%
        elif key.isdigit():
            set_speed(int(key) * 10)

        draw_hud(current, speed, esp_lines, last_key)

def list_ports():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print(f"{C.WARN}No se encontraron puertos seriales.{C.RESET}")
        return
    print(f"\n{C.INFO}Puertos disponibles:{C.RESET}")
    for p in ports:
        print(f"  {p.device:20s} — {p.description}")
    print()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=None)
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    port = args.port
    if port is None:
        port = find_esp32_port()
        if port:
            print(f"{C.OK}✔ ESP32 detectada en: {port}{C.RESET}")
        else:
            print(f"{C.ERR}✘ No se detectó ninguna ESP32.{C.RESET}")
            list_ports()
            print(f"{C.WARN}Usa: python3 motor_control_usb.py --port /dev/ttyUSB0{C.RESET}")
            sys.exit(1)

    try:
        ser = serial.Serial(port=port, baudrate=args.baud, timeout=1,
                            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE)
        print(f"{C.OK}✔ Conectado a {port} a {args.baud} bps{C.RESET}")
    except serial.SerialException as e:
        print(f"{C.ERR}✘ No se pudo abrir {port}: {e}{C.RESET}")
        list_ports()
        sys.exit(1)

    print(f"{C.WARN}Esperando que la ESP32 arranque...{C.RESET}")
    time.sleep(2)
    ser.reset_input_buffer()

    esp_lines  = []
    stop_event = threading.Event()

    t = threading.Thread(target=reader_thread,
                         args=(ser, stop_event, esp_lines), daemon=True)
    t.start()

    try:
        wasd_loop(ser, stop_event, esp_lines)
    except Exception as e:
        print(f"\n{C.ERR}Error: {e}{C.RESET}")
    finally:
        stop_event.set()
        try:
            ser.write(b"stop\n")
            time.sleep(0.1)
        except Exception:
            pass
        ser.close()
        # Restaurar terminal
        print(f"\n{C.INFO}Puerto cerrado. Hasta luego.{C.RESET}")

if __name__ == "__main__":
    main()
