#!/usr/bin/env python3
"""
Control Robot Diferencial JGY370 - Modo WASD + Modo Comandos
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

# ─────────────────────────────────────────────────────────
class C:
    OK    = "\033[92m"
    ERR   = "\033[91m"
    INFO  = "\033[94m"
    WARN  = "\033[93m"
    RESET = "\033[0m"
    BOLD  = "\033[1m"
    DIM   = "\033[2m"
    CLEAR = "\033[2J\033[H"

# ─────────────────────────────────────────────────────────
def find_esp32_port():
    known = ["CP210", "CH340", "CH341", "FTDI", "FT232", "USB Serial", "ESP32"]
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").upper()
        mfr  = (p.manufacturer or "").upper()
        if any(k in desc or k in mfr for k in known):
            return p.device
    ports = serial.tools.list_ports.comports()
    return ports[0].device if ports else None

def list_ports():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print(f"{C.WARN}No se encontraron puertos seriales.{C.RESET}")
        return
    print(f"\n{C.INFO}Puertos disponibles:{C.RESET}")
    for p in ports:
        print(f"  {p.device:20s} — {p.description}")
    print()

# ─────────────────────────────────────────────────────────
# Hilo lector de respuestas ESP32
# ─────────────────────────────────────────────────────────
def reader_thread(ser, stop_event, esp_lines):
    while not stop_event.is_set():
        try:
            if ser.in_waiting:
                line = ser.readline().decode("utf-8", errors="replace").strip()
                if line and not line.startswith(">"):
                    esp_lines.append(line)
                    if len(esp_lines) > 8:
                        esp_lines.pop(0)
        except serial.SerialException:
            break
        except Exception:
            pass
        time.sleep(0.02)

# ─────────────────────────────────────────────────────────
# Lectura de tecla sin Enter (modo WASD)
# ─────────────────────────────────────────────────────────
def get_key():
    fd  = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            ch2 = sys.stdin.read(1)
            if ch2 == '[':
                ch3 = sys.stdin.read(1)
                return {'A':'w','B':'s','C':'d','D':'a'}.get(ch3, '')
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

# ─────────────────────────────────────────────────────────
# MODO WASD
# ─────────────────────────────────────────────────────────
def mode_wasd(ser, stop_event, esp_lines):
    speed     = 50
    current   = "stop"
    last_sent = "stop"
    last_key  = ""

    KEY_MAP = {
        'w':'adelante','W':'adelante',
        's':'atras',   'S':'atras',
        'a':'izquierda','A':'izquierda',
        'd':'derecha', 'D':'derecha',
        ' ':'stop',
    }
    STATE_LABEL = {
        "adelante":  f"{C.OK}▲  avanzando{C.RESET}",
        "atras":     f"{C.WARN}▼  retrocediendo{C.RESET}",
        "derecha":   f"{C.INFO}▶  girando derecha{C.RESET}",
        "izquierda": f"{C.INFO}◀  girando izquierda{C.RESET}",
        "stop":      f"{C.DIM}■  parado{C.RESET}",
    }

    def send(cmd):
        ser.write((cmd + "\n").encode())
        time.sleep(0.02)

    def redraw():
        print(C.CLEAR, end="")
        print(f"{C.BOLD}╔══════════════════════════════════════════╗")
        print(f"║        MODO WASD  —  Tiempo Real         ║")
        print(f"╚══════════════════════════════════════════╝{C.RESET}")
        print(f"\n  {C.BOLD}Velocidad:{C.RESET} {C.OK}{speed}%{C.RESET}")
        print(f"\n  {C.BOLD}Controles:{C.RESET}")
        print(f"           {C.BOLD}W / ↑{C.RESET}  Adelante")
        print(f"     {C.BOLD}A / ←{C.RESET}   {C.BOLD}S / ↓{C.RESET}   {C.BOLD}D / →{C.RESET}  Izq / Atrás / Der")
        print(f"           {C.BOLD}Esp{C.RESET}    Stop")
        print(f"           {C.BOLD}+ / -{C.RESET}  Velocidad ±10%")
        print(f"           {C.BOLD}0..9{C.RESET}   Velocidad 0%..90%")
        print(f"           {C.BOLD}M{C.RESET}      Cambiar a modo COMANDOS")
        print(f"           {C.BOLD}Q{C.RESET}      Salir")
        print(f"\n  {C.BOLD}Estado:{C.RESET}  {STATE_LABEL.get(current, current)}")
        print(f"  {C.BOLD}Tecla:{C.RESET}   {repr(last_key)}")
        print(f"\n  {C.BOLD}ESP32:{C.RESET}")
        for ln in esp_lines[-5:]:
            print(f"    {C.INFO}{ln}{C.RESET}")
        sys.stdout.flush()

    redraw()

    while not stop_event.is_set():
        key = get_key()
        last_key = key

        if key in ('q','Q','\x03'):
            send("stop")
            return "exit"

        if key in ('m','M'):
            send("stop")
            return "cmd"

        if key in KEY_MAP:
            cmd = KEY_MAP[key]
            if cmd != last_sent:
                send(cmd)
                last_sent = cmd
            current = cmd

        elif key in ('+','='):
            speed = min(100, speed + 10)
            send(f"velocidad {speed}")
        elif key in ('-','_'):
            speed = max(0, speed - 10)
            send(f"velocidad {speed}")
        elif key.isdigit():
            speed = int(key) * 10
            send(f"velocidad {speed}")

        redraw()

    return "exit"

# ─────────────────────────────────────────────────────────
# Validador de comandos (modo texto)
# ─────────────────────────────────────────────────────────
MOVE_CMDS = ("adelante", "atras", "derecha", "izquierda")
INFO_CMDS = ("stop", "status", "help", "distancia", "velocidad",
             "reset", "diag", "ports")

def validate_command(cmd: str) -> tuple[bool, str]:
    parts = cmd.strip().lower().split()
    if not parts:
        return False, "Comando vacío."

    word = parts[0]

    if word in INFO_CMDS:
        return True, ""

    # velocidad <numero>
    if word == "velocidad" and len(parts) == 2:
        try:
            v = int(parts[1])
            if not 0 <= v <= 100:
                return False, "Velocidad debe ser 0-100."
        except ValueError:
            return False, f"'{parts[1]}' no es un número válido."
        return True, ""

    # movimiento [valor]
    if word in MOVE_CMDS:
        if len(parts) == 1:
            return True, ""
        if len(parts) == 2:
            try:
                v = int(parts[1])
                if v <= 0:
                    return False, "El valor debe ser positivo."
            except ValueError:
                return False, f"'{parts[1]}' no es un número válido."
            return True, ""
        return False, f"Uso: {word} [mm_o_ms]"

    return False, f"'{word}' no reconocido. Escribe 'help'."

# ─────────────────────────────────────────────────────────
# MODO COMANDOS
# ─────────────────────────────────────────────────────────
def mode_cmd(ser, stop_event, esp_lines):

    def send(cmd):
        ser.write((cmd + "\n").encode())
        time.sleep(0.05)

    print(C.CLEAR, end="")
    print(f"{C.BOLD}╔══════════════════════════════════════════╗")
    print(f"║        MODO COMANDOS                     ║")
    print(f"╚══════════════════════════════════════════╝{C.RESET}")
    print(f"""
  {C.BOLD}MOVIMIENTO:{C.RESET}
    adelante [val]     Avanzar
    atras [val]        Retroceder
    derecha [val]      Girar derecha (sobre eje)
    izquierda [val]    Girar izquierda (sobre eje)
    stop               Parar

    {C.DIM}[val] <= 9999  →  milímetros (usa encoders){C.RESET}
    {C.DIM}[val] >= 10000 →  milisegundos{C.RESET}
    {C.DIM}sin [val]      →  indefinido{C.RESET}

  {C.BOLD}CONFIGURACION:{C.RESET}
    velocidad <0-100>  Setear velocidad global

  {C.BOLD}INFORMACION:{C.RESET}
    distancia          mm recorridos por cada rueda
    velocidad          mm/s actuales (sin número)
    status             Estado detallado
    reset              Resetear encoders
    diag               Pulsos + mm en tiempo real

  {C.BOLD}NAVEGACION:{C.RESET}
    wasd               Cambiar a modo WASD
    exit / quit        Salir

  {C.BOLD}EJEMPLOS:{C.RESET}
    {C.OK}velocidad 60{C.RESET}       Setea 60%
    {C.OK}adelante 500{C.RESET}       Avanza 500 mm (para solo)
    {C.OK}derecha 10000{C.RESET}      Gira 10 segundos
    {C.OK}atras{C.RESET}              Retrocede indefinido
""")

    while not stop_event.is_set():
        # Mostrar mensajes ESP32 pendientes
        while esp_lines:
            print(f"  {C.INFO}[ESP32]{C.RESET} {esp_lines.pop(0)}")

        try:
            cmd = input(f"{C.BOLD}cmd>{C.RESET} ").strip()
        except EOFError:
            return "exit"

        if not cmd:
            continue

        low = cmd.lower()

        if low == "ports":
            list_ports()
            continue

        if low == "wasd":
            send("stop")
            return "wasd"

        if low in ("exit", "quit", "salir"):
            return "exit"

        valid, err = validate_command(low)
        if not valid:
            print(f"  {C.ERR}[ERROR]{C.RESET} {err}")
            continue

        send(cmd)

    return "exit"

# ─────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default=None)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--mode", choices=["wasd","cmd"], default="wasd",
                        help="Modo inicial (default: wasd)")
    args = parser.parse_args()

    port = args.port
    if port is None:
        port = find_esp32_port()
        if port:
            print(f"{C.OK}✔ ESP32 detectada en: {port}{C.RESET}")
        else:
            print(f"{C.ERR}✘ No se detectó ninguna ESP32.{C.RESET}")
            list_ports()
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

    current_mode = args.mode

    try:
        while True:
            if current_mode == "wasd":
                result = mode_wasd(ser, stop_event, esp_lines)
            else:
                result = mode_cmd(ser, stop_event, esp_lines)

            if result == "exit":
                break
            elif result == "wasd":
                current_mode = "wasd"
            elif result == "cmd":
                current_mode = "cmd"

    except Exception as e:
        print(f"\n{C.ERR}Error inesperado: {e}{C.RESET}")
    finally:
        stop_event.set()
        try:
            ser.write(b"stop\n")
            time.sleep(0.1)
        except Exception:
            pass
        ser.close()
        print(f"\n{C.INFO}Puerto cerrado. Hasta luego.{C.RESET}")

if __name__ == "__main__":
    main()
