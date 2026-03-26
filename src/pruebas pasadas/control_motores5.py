#!/usr/bin/env python3
"""
Control Robot Diferencial JGY370 desde Raspberry Pi
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

class C:
    OK    = "\033[92m"
    ERR   = "\033[91m"
    INFO  = "\033[94m"
    WARN  = "\033[93m"
    RESET = "\033[0m"
    BOLD  = "\033[1m"

def find_esp32_port():
    known_chips = ["CP210", "CH340", "CH341", "FTDI", "FT232", "USB Serial", "ESP32"]
    for port in serial.tools.list_ports.comports():
        desc = (port.description or "").upper()
        mfr  = (port.manufacturer or "").upper()
        if any(chip.upper() in desc or chip.upper() in mfr for chip in known_chips):
            return port.device
    ports = serial.tools.list_ports.comports()
    return ports[0].device if ports else None

def reader_thread(ser, stop_event):
    while not stop_event.is_set():
        try:
            if ser.in_waiting:
                line = ser.readline().decode("utf-8", errors="replace").strip()
                if line:
                    print(f"\n{C.INFO}[ESP32]{C.RESET} {line}")
                    print(f"{C.BOLD}cmd>{C.RESET} ", end="", flush=True)
        except serial.SerialException:
            break
        except Exception:
            pass
        time.sleep(0.05)

MOVE_CMDS  = ("adelante", "atras", "derecha", "izquierda")
INFO_CMDS  = ("status", "help", "distancia", "velocidad", "reset", "diag", "stop", "ports")

def validate_command(cmd: str) -> tuple[bool, str]:
    """
    Formatos válidos:
      adelante | atras | derecha | izquierda       (usa velocidad global)
      adelante | atras | derecha | izquierda <ms>  (con tiempo)
      velocidad <0-100>                            (setear velocidad)
      stop | status | help | distancia | velocidad | reset | diag | ports
    """
    parts = cmd.strip().lower().split()
    if not parts:
        return False, "Comando vacío."

    word = parts[0]

    if word in INFO_CMDS:
        return True, ""

    # Setear velocidad: "velocidad <numero>"
    if word == "velocidad" and len(parts) == 2:
        try:
            v = int(parts[1])
            if not 0 <= v <= 100:
                return False, "La velocidad debe ser 0-100."
        except ValueError:
            return False, f"'{parts[1]}' no es un número válido."
        return True, ""

    # Comandos de movimiento
    if word in MOVE_CMDS:
        if len(parts) == 1:
            return True, ""   # sin tiempo
        if len(parts) == 2:
            try:
                ms = int(parts[1])
                if ms <= 0:
                    return False, "La duración debe ser un número positivo en ms."
            except ValueError:
                return False, f"'{parts[1]}' no es un número válido para la duración."
            return True, ""
        return False, f"Demasiados argumentos. Uso: {word} [ms]"

    return False, f"Comando '{word}' no reconocido. Escribe 'help'."

def list_ports():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print(f"{C.WARN}No se encontraron puertos seriales.{C.RESET}")
        return
    print(f"\n{C.INFO}Puertos disponibles:{C.RESET}")
    for p in ports:
        print(f"  {p.device:20s} — {p.description}")
    print()

def print_banner(port, baud):
    print(f"""
{C.BOLD}╔══════════════════════════════════════════╗
║   Robot Diferencial JGY370               ║
║   Raspberry Pi → USB Serial → ESP32      ║
╚══════════════════════════════════════════╝{C.RESET}
{C.INFO}Puerto : {port}  |  Baudios: {baud}{C.RESET}

{C.BOLD}MOVIMIENTO:{C.RESET}
  adelante [ms]      → Avanzar (indefinido o X milisegundos)
  atras [ms]         → Retroceder
  derecha [ms]       → Girar a la derecha
  izquierda [ms]     → Girar a la izquierda
  stop               → Parar

{C.BOLD}CONFIGURACION:{C.RESET}
  velocidad <0-100>  → Setear velocidad global (persiste)

{C.BOLD}INFORMACION:{C.RESET}
  status             → Estado actual + tiempo restante
  distancia          → mm recorridos por cada rueda
  velocidad          → mm/s actuales (sin número)
  reset              → Resetear encoders
  diag               → Pulsos crudos en tiempo real
  ports              → Listar puertos disponibles
  exit / quit        → Salir (para motores)

{C.BOLD}EJEMPLOS:{C.RESET}
  {C.OK}velocidad 70{C.RESET}       → Setea 70% para todos los movimientos
  {C.OK}adelante 3000{C.RESET}      → Avanza 3 segundos al 70% y para solo
  {C.OK}derecha 500{C.RESET}        → Gira derecha 500ms
  {C.OK}atras{C.RESET}              → Retrocede indefinido
""")

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

    print_banner(port, args.baud)

    stop_event = threading.Event()
    t = threading.Thread(target=reader_thread, args=(ser, stop_event), daemon=True)
    t.start()

    try:
        while True:
            try:
                cmd = input(f"{C.BOLD}cmd>{C.RESET} ").strip()
            except EOFError:
                break

            if not cmd:
                continue

            if cmd.lower() == "ports":
                list_ports()
                continue

            if cmd.lower() in ("exit", "quit", "salir"):
                print(f"{C.WARN}Deteniendo motores y saliendo...{C.RESET}")
                ser.write(b"stop\n")
                time.sleep(0.2)
                break

            valid, error_msg = validate_command(cmd)
            if not valid:
                print(f"{C.ERR}[ERROR] {error_msg}{C.RESET}")
                continue

            ser.write((cmd + "\n").encode("utf-8"))
            time.sleep(0.05)

    except KeyboardInterrupt:
        print(f"\n{C.WARN}Interrupción — deteniendo motores...{C.RESET}")
        ser.write(b"stop\n")

    finally:
        stop_event.set()
        ser.close()
        print(f"{C.INFO}Puerto cerrado. Hasta luego.{C.RESET}")

if __name__ == "__main__":
    main()
