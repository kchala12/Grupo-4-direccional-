#!/usr/bin/env python3
"""
Control de Motores JGY370 desde Raspberry Pi
Comunicación USB Serial con ESP32
----------------------------------------------
Conexión:
  Cable USB directamente entre Raspberry Pi y ESP32
  (el mismo cable que usas para programar la ESP32)

Instalar dependencia:
  pip install pyserial

Uso:
  python3 motor_control_usb.py
  python3 motor_control_usb.py --port /dev/ttyUSB0
  python3 motor_control_usb.py --port /dev/ttyACM0
"""

import serial
import serial.tools.list_ports
import threading
import time
import argparse
import sys

# ── Colores para terminal ──────────────────────────────────
class C:
    OK    = "\033[92m"
    ERR   = "\033[91m"
    INFO  = "\033[94m"
    WARN  = "\033[93m"
    RESET = "\033[0m"
    BOLD  = "\033[1m"

# ── Detectar puerto ESP32 automáticamente ──────────────────
def find_esp32_port() -> str | None:
    """Busca automáticamente el puerto USB de la ESP32."""
    # VIDs/PIDs comunes de chips USB en ESP32 (CP2102, CH340, FTDI)
    known_chips = [
        "CP210",   # Silicon Labs CP2102/CP2104
        "CH340",   # CH340G
        "CH341",
        "FTDI",
        "FT232",
        "USB Serial",
        "ESP32",
    ]
    ports = serial.tools.list_ports.comports()
    for port in ports:
        desc = (port.description or "").upper()
        mfr  = (port.manufacturer or "").upper()
        for chip in known_chips:
            if chip.upper() in desc or chip.upper() in mfr:
                return port.device
    # Si no se detecta por nombre, devolver el primero disponible
    if ports:
        return ports[0].device
    return None

# ── Leer respuestas de la ESP32 en hilo separado ───────────
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

# ── Validar comando antes de enviarlo ─────────────────────
def validate_command(cmd: str) -> tuple[bool, str]:
    cmd = cmd.strip().lower()

    if cmd in ("status", "help"):
        return True, ""

    parts = cmd.split()
    if len(parts) < 2:
        return False, "Formato incorrecto. Ej: m1 f 80"

    motor = parts[0]
    if motor not in ("m1", "m2"):
        return False, "Motor inválido. Usa 'm1' o 'm2'."

    direction = parts[1]
    if direction == "s":
        return True, ""

    if direction not in ("f", "b"):
        return False, "Dirección inválida. Usa 'f' (adelante), 'b' (atrás) o 's' (stop)."

    if len(parts) < 3:
        return False, "Falta el porcentaje. Ej: m1 f 80"

    try:
        pct = int(parts[2])
        if not 0 <= pct <= 100:
            return False, "El porcentaje debe estar entre 0 y 100."
    except ValueError:
        return False, f"'{parts[2]}' no es un número válido."

    return True, ""

# ── Listar puertos disponibles ─────────────────────────────
def list_ports():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print(f"{C.WARN}No se encontraron puertos seriales.{C.RESET}")
        return
    print(f"\n{C.INFO}Puertos disponibles:{C.RESET}")
    for p in ports:
        print(f"  {p.device:20s} — {p.description}")
    print()

# ── Banner ─────────────────────────────────────────────────
def print_banner(port: str, baud: int):
    print(f"""
{C.BOLD}╔══════════════════════════════════════════╗
║   Control Motores JGY370 - Raspberry Pi  ║
║   USB Serial → ESP32                     ║
╚══════════════════════════════════════════╝{C.RESET}
{C.INFO}Puerto : {port}
Baudios: {baud}{C.RESET}

{C.BOLD}COMANDOS:{C.RESET}
  m1 f <0-100>  → Motor 1 adelante al X%
  m1 b <0-100>  → Motor 1 atrás al X%
  m1 s          → Motor 1 stop
  m2 f <0-100>  → Motor 2 adelante al X%
  m2 b <0-100>  → Motor 2 atrás al X%
  m2 s          → Motor 2 stop
  status        → Estado de motores
  help          → Ayuda en ESP32
  ports         → Listar puertos disponibles
  exit / quit   → Salir
""")

# ── Main ───────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="Control de motores via USB Serial")
    parser.add_argument("--port", default=None,
                        help="Puerto USB (ej: /dev/ttyUSB0 o /dev/ttyACM0). "
                             "Si no se indica, se detecta automáticamente.")
    parser.add_argument("--baud", type=int, default=115200,
                        help="Baudios (default: 115200)")
    args = parser.parse_args()

    # Detectar puerto si no se especificó
    port = args.port
    if port is None:
        port = find_esp32_port()
        if port:
            print(f"{C.OK}✔ ESP32 detectada en: {port}{C.RESET}")
        else:
            print(f"{C.ERR}✘ No se detectó ninguna ESP32.{C.RESET}")
            list_ports()
            print(f"{C.WARN}Especifica el puerto con: python3 motor_control_usb.py --port /dev/ttyUSB0{C.RESET}")
            sys.exit(1)

    # Abrir puerto serial
    try:
        ser = serial.Serial(
            port     = port,
            baudrate = args.baud,
            timeout  = 1,
            bytesize = serial.EIGHTBITS,
            parity   = serial.PARITY_NONE,
            stopbits = serial.STOPBITS_ONE
        )
        print(f"{C.OK}✔ Conectado a {port} a {args.baud} bps{C.RESET}")
    except serial.SerialException as e:
        print(f"{C.ERR}✘ No se pudo abrir {port}: {e}{C.RESET}")
        list_ports()
        sys.exit(1)

    # Esperar reset de la ESP32 (al conectar USB hace reset automático)
    print(f"{C.WARN}Esperando que la ESP32 arranque...{C.RESET}")
    time.sleep(2)
    ser.reset_input_buffer()  # Limpiar basura del reset

    print_banner(port, args.baud)

    # Hilo lector
    stop_event = threading.Event()
    t = threading.Thread(target=reader_thread, args=(ser, stop_event), daemon=True)
    t.start()

    # Bucle principal
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
