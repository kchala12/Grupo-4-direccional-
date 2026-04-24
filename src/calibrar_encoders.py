"""
calibrar_encoders.py — Calibración de pulsos por vuelta
========================================================
Uso:
    python3 calibrar_encoders.py

Ingresa milisegundos y el robot avanza ese tiempo.
Al detenerse muestra los pulsos de cada motor.
Comando 'r' resetea los contadores.
Comando 'exit' para salir.
"""

import serial
import serial.tools.list_ports
import threading
import time
import sys

# ══════════════════════════════════════════════
#  CONFIGURACIÓN
# ══════════════════════════════════════════════
VELOCIDAD   = 50     # velocidad fija para la prueba (0-100)
BAUD        = 115200

# ══════════════════════════════════════════════
#  ESTADO
# ══════════════════════════════════════════════
enc = {"c1": 0, "c2": 0}
_ser = None
_ser_lock = threading.Lock()

class C:
    OK="\033[92m"; ERR="\033[91m"; INFO="\033[94m"
    RESET="\033[0m"; BOLD="\033[1m"


# ══════════════════════════════════════════════
#  SERIAL
# ══════════════════════════════════════════════
def enviar(cmd: str):
    with _ser_lock:
        if _ser and _ser.is_open:
            try:
                _ser.write((cmd.strip() + "\n").encode("utf-8"))
            except Exception:
                pass

def _hilo_lector():
    while True:
        try:
            with _ser_lock:
                hay = _ser.in_waiting if _ser else 0
            if hay:
                with _ser_lock:
                    linea = _ser.readline().decode("utf-8", errors="replace").strip()
                if linea.startswith("E "):
                    p = linea.split()
                    if len(p) == 3:
                        enc["c1"] = int(p[1])
                        enc["c2"] = int(p[2])
        except Exception:
            pass
        time.sleep(0.01)

def conectar():
    global _ser
    chips = ["CP210","CH340","CH341","FTDI","FT232","USB Serial","ESP32"]
    puerto = None
    for p in serial.tools.list_ports.comports():
        d = (p.description or "").upper()
        m = (p.manufacturer or "").upper()
        if any(c in d or c in m for c in chips):
            puerto = p.device
            break
    if not puerto:
        pts = serial.tools.list_ports.comports()
        puerto = pts[0].device if pts else None
    if not puerto:
        print(f"{C.ERR}ESP32 no encontrada.{C.RESET}")
        sys.exit(1)
    try:
        _ser = serial.Serial(puerto, baudrate=BAUD, timeout=1)
        print(f"{C.OK}Conectado: {puerto}{C.RESET}")
    except Exception as e:
        print(f"{C.ERR}{e}{C.RESET}")
        sys.exit(1)
    time.sleep(2)
    _ser.reset_input_buffer()
    threading.Thread(target=_hilo_lector, daemon=True).start()


# ══════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════
def main():
    conectar()

    print(f"""{C.BOLD}
╔══════════════════════════════════════════════╗
║   CALIBRACIÓN DE ENCODERS                    ║
║                                              ║
║   Ingresa milisegundos para avanzar.         ║
║   r     → resetear contadores               ║
║   exit  → salir                             ║
╚══════════════════════════════════════════════╝{C.RESET}
Velocidad fija: {VELOCIDAD}%
""")

    while True:
        try:
            cmd = input(f"{C.BOLD}ms>{C.RESET} ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            break

        if not cmd:
            continue

        if cmd in ("exit", "quit"):
            break

        if cmd == "r":
            enviar("reset 0")
            enc["c1"] = 0
            enc["c2"] = 0
            print(f"  {C.OK}Contadores reseteados.{C.RESET}")
            continue

        try:
            ms = int(cmd)
        except ValueError:
            print(f"  {C.ERR}Ingresa solo un número de milisegundos, 'r' o 'exit'.{C.RESET}")
            continue

        if ms <= 0:
            print(f"  {C.ERR}El valor debe ser mayor a 0.{C.RESET}")
            continue

        antes1 = enc["c1"]
        antes2 = enc["c2"]

        enviar(f"f {VELOCIDAD}")
        time.sleep(ms / 1000.0)
        enviar("s 0")

        time.sleep(0.2)  # esperar último reporte de encoders

        d1 = abs(enc["c1"] - antes1)
        d2 = abs(enc["c2"] - antes2)

        print(f"""
  Tiempo:  {ms} ms  a  {VELOCIDAD}% velocidad
  ├─ M1:  {d1} pulsos
  └─ M2:  {d2} pulsos
""")

    enviar("s 0")
    if _ser and _ser.is_open:
        _ser.close()
    print(f"{C.INFO}Listo.{C.RESET}")


if __name__ == "__main__":
    main()
