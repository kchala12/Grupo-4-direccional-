#!/usr/bin/env python3
"""
Control de Motores JGY370 desde Raspberry Pi
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

def validate_command(cmd: str) -> tuple[bool, str]:
    """
    Formatos válidos:
      mX f|b <0-100>          (indefinido)
      mX f|b <0-100> <ms>     (con tiempo en milisegundos)
      mX s
      status | help | distancia | velocidad | reset | diag | ports
    """
    cmd = cmd.strip().lower()

    if cmd in ("status", "help", "distancia", "velocidad", "reset", "diag", "ports"):
        return True, ""

    parts = cmd.split()
    if len(parts) < 2:
        return False, "Formato incorrecto. Ej: m1 f 80  o  m1 f 80 2000"

    if parts[0] not in ("m1", "m2"):
        return False, "Motor inválido. Usa 'm1' o 'm2'."

    direction = parts[1]
    if direction == "s":
        return True, ""

    if direction not in ("f", "b"):
        return False, "Dirección inválida. Usa 'f', 'b' o 's'."

    if len(parts) < 3:
        return False, "Falta el porcentaje. Ej: m1 f 80"

    try:
        pct = int(parts[2])
        if not 0 <= pct <= 100:
            return False, "El porcentaje debe ser 0-100."
    except ValueError:
        return False, f"'{parts[2]}' no es un número válido."

    # Validar tiempo opcional
    if len(parts) >= 4:
        try:
            ms = int(parts[3])
            if ms <= 0:
                return False, "La duración debe ser un número positivo en milisegundos."
        except ValueError:
            return False, f"'{parts[3]}' no es un número válido para la duración."

    return True, ""

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
║   Control Motores JGY370 - Raspberry Pi  ║
║   USB Serial → ESP32                     ║
╚══════════════════════════════════════════╝{C.RESET}
{C.INFO}Puerto : {port}
Baudios: {baud}{C.RESET}

{C.BOLD}COMANDOS:{C.RESET}
  m1 f <pct>           → Motor 1 adelante (indefinido)
  m1 f <pct> <ms>      → Motor 1 adelante durante X milisegundos
  m1 b <pct> <ms>      → Motor 1 atrás durante X milisegundos
  m1 s                 → Motor 1 stop
  m2 f <pct> <ms>      → Motor 2 adelante durante X milisegundos
  m2 b <pct> <ms>      → Motor 2 atrás durante X milisegundos
  m2 s                 → Motor 2 stop
  distancia            → mm recorridos por cada rueda
  velocidad            → mm/s actuales
  diag                 → Pulsos crudos en tiempo real
  reset                → Resetear encoders
  status               → Estado + tiempo restante
  ports                → Listar puertos disponibles
  exit / quit          → Salir (para motores automáticamente)

{C.BOLD}EJEMPLOS:{C.RESET}
  m1 f 80 3000         → Motor 1 adelante al 80% durante 3 segundos
  m2 b 50 1500         → Motor 2 atrás al 50% durante 1.5 segundos
  m1 f 60              → Motor 1 adelante al 60% sin límite de tiempo
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
                ser.write(b"m1 s\n")
                time.sleep(0.1)
                ser.write(b"m2 s\n")
                time.sleep(0.1)
                break

            valid, error_msg = validate_command(cmd)
            if not valid:
                print(f"{C.ERR}[ERROR] {error_msg}{C.RESET}")
                continue

            ser.write((cmd + "\n").encode("utf-8"))
            time.sleep(0.05)

    except KeyboardInterrupt:
        print(f"\n{C.WARN}Interrupción — deteniendo motores...{C.RESET}")
        ser.write(b"m1 s\n")
        time.sleep(0.1)
        ser.write(b"m2 s\n")

    finally:
        stop_event.set()
        ser.close()
        print(f"{C.INFO}Puerto cerrado. Hasta luego.{C.RESET}")

if __name__ == "__main__":
    main()
