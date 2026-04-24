"""
motor_control.py — Librería control motores JGY370
===================================================
Uso como librería:
    import motor_control
    motor_control.iniciar()          # conecta ESP32 y arranca hilos

    motor_control.enviar("f 60")     # adelante al 60%
    motor_control.enviar("b 60")     # atrás
    motor_control.enviar("l 50")     # izquierda
    motor_control.enviar("r 50")     # derecha
    motor_control.enviar("s 0")      # stop
    motor_control.set_velocidades(55, 45, "f")  # PID dirección

    motor_control.enc["c1"]          # pulsos encoder M1
    motor_control.enc["c2"]          # pulsos encoder M2

Modo test standalone:
    python3 motor_control.py
    Comandos: f|b|l|r <vel>  /  f|b|l|r <N> mm  /  f|b|l|r <N> ms  /  s  /  reset  /  exit
"""

import serial
import serial.tools.list_ports
import threading
import time
import math
import sys

# ══════════════════════════════════════════════
#  CALIBRACIÓN
# ══════════════════════════════════════════════
M1_PULSOS_POR_VUELTA = 3816
M2_PULSOS_POR_VUELTA = 2689
RADIO_MM             = 30.0
M1_MM_POR_PULSO      = (2 * math.pi * RADIO_MM) / M1_PULSOS_POR_VUELTA
M2_MM_POR_PULSO      = (2 * math.pi * RADIO_MM) / M2_PULSOS_POR_VUELTA

# ══════════════════════════════════════════════
#  PARÁMETROS PID VELOCIDAD
# ══════════════════════════════════════════════
CTRL_KP             = 1.0
CTRL_INTERVALO_S    = 0.1
CTRL_CORRECCION_MAX = 20

# ══════════════════════════════════════════════
#  ESTADO INTERNO
# ══════════════════════════════════════════════
enc = {"c1": 0, "c2": 0}

_pid = {
    "activo":   False,
    "vel_base": 50,
    "dir":      "f",
}

_ser: serial.Serial | None = None
_ser_lock = threading.Lock()
_stop_ev: threading.Event | None = None

class C:
    OK="\033[92m"; ERR="\033[91m"; INFO="\033[94m"
    RESET="\033[0m"; BOLD="\033[1m"; DIM="\033[2m"

_DIRS_PID = ("f", "b")


# ══════════════════════════════════════════════
#  COMUNICACIÓN
# ══════════════════════════════════════════════
def enviar(cmd: str):
    """Envía un comando a la ESP32. Formato: 'f 60', 'b 50', 's 0', etc."""
    with _ser_lock:
        if _ser and _ser.is_open:
            try:
                _ser.write((cmd.strip() + "\n").encode("utf-8"))
            except Exception:
                pass

def set_velocidades(vel_izq: int, vel_der: int, dir_letra: str):
    """Velocidades independientes por motor para el PID de dirección."""
    enviar(f"pid {vel_izq} {vel_der} {dir_letra}")


# ══════════════════════════════════════════════
#  HILOS INTERNOS
# ══════════════════════════════════════════════
def _hilo_lector(stop_ev):
    while not stop_ev.is_set():
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


def _hilo_controlador(stop_ev):
    t0 = 0.0; p0c1 = 0; p0c2 = 0
    tasa1 = 0.0; tasa2 = 0.0

    while not stop_ev.is_set():
        time.sleep(CTRL_INTERVALO_S)

        if not _pid["activo"]:
            t0 = 0.0; tasa1 = 0.0; tasa2 = 0.0
            continue

        if t0 == 0.0:
            t0 = time.time(); p0c1 = enc["c1"]; p0c2 = enc["c2"]
            continue

        dt = time.time() - t0
        r1 = abs(enc["c1"] - p0c1)
        r2 = abs(enc["c2"] - p0c2)

        if tasa1 == 0.0 and r1 > 0: tasa1 = r1 / dt
        if tasa2 == 0.0 and r2 > 0: tasa2 = r2 / dt
        if tasa1 == 0.0 or tasa2 == 0.0: continue

        e1 = r1 - tasa1 * dt
        e2 = r2 - tasa2 * dt
        c1 = max(-CTRL_CORRECCION_MAX, min(CTRL_CORRECCION_MAX, CTRL_KP * e1))
        c2 = max(-CTRL_CORRECCION_MAX, min(CTRL_CORRECCION_MAX, CTRL_KP * e2))
        base = _pid["vel_base"]
        m1   = int(max(0, min(100, base - c1)))
        m2   = int(max(0, min(100, base - c2)))

        if _pid["dir"]:
            enviar(f"pid {m1} {m2} {_pid['dir']}")


# ══════════════════════════════════════════════
#  CONEXIÓN
# ══════════════════════════════════════════════
def _buscar_puerto():
    chips = ["CP210","CH340","CH341","FTDI","FT232","USB Serial","ESP32"]
    for p in serial.tools.list_ports.comports():
        d = (p.description or "").upper()
        m = (p.manufacturer or "").upper()
        if any(c in d or c in m for c in chips):
            return p.device
    pts = serial.tools.list_ports.comports()
    return pts[0].device if pts else None


def iniciar(port=None, baud=115200):
    """Conecta a la ESP32 y arranca hilos. Retorna True si tuvo éxito."""
    global _ser, _stop_ev

    puerto = port or _buscar_puerto()
    if not puerto:
        print(f"{C.ERR}[MOTORES] ESP32 no encontrada.{C.RESET}")
        return False
    try:
        _ser = serial.Serial(puerto, baudrate=baud, timeout=1)
        print(f"{C.OK}[MOTORES] Conectado: {puerto}{C.RESET}")
    except serial.SerialException as e:
        print(f"{C.ERR}[MOTORES] {e}{C.RESET}")
        return False

    time.sleep(2)
    _ser.reset_input_buffer()
    _stop_ev = threading.Event()
    threading.Thread(target=_hilo_lector,      args=(_stop_ev,), daemon=True).start()
    threading.Thread(target=_hilo_controlador, args=(_stop_ev,), daemon=True).start()
    return True


def detener():
    """Para hilos y cierra serial."""
    if _stop_ev:
        _stop_ev.set()
    _pid["activo"] = False
    try:
        enviar("s 0")
        time.sleep(0.1)
    except Exception:
        pass
    if _ser and _ser.is_open:
        _ser.close()
    print(f"{C.INFO}[MOTORES] Desconectado.{C.RESET}")


# ══════════════════════════════════════════════
#  MODO TEST
# ══════════════════════════════════════════════
def _modo_test():
    vel = 50
    print(f"""{C.BOLD}
╔══════════════════════════════════════════════╗
║  MODO TEST MOTORES                           ║
║  f|b|l|r <vel>      movimiento indefinido    ║
║  f|b|l|r <N> mm     distancia en mm         ║
║  f|b|l|r <N> ms     tiempo en ms            ║
║  s                  stop                    ║
║  vel <0-100>        velocidad global        ║
║  reset              encoders a cero         ║
║  exit               volver al menú          ║
╚══════════════════════════════════════════════╝{C.RESET}
""")

    while True:
        try:
            cmd = input(f"{C.BOLD}test>{C.RESET} ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            break

        if not cmd: continue
        if cmd in ("exit", "quit"): break

        p = cmd.split()
        letra = p[0]

        if letra == "s":
            _pid["activo"] = False
            enviar("s 0")
            print(f"  {C.OK}Stop.{C.RESET}")

        elif letra == "reset":
            enviar("reset 0")
            print(f"  {C.OK}Encoders reseteados.{C.RESET}")

        elif letra == "vel":
            if len(p) == 2:
                try:
                    vel = int(p[1]); _pid["vel_base"] = vel
                    print(f"  {C.OK}Velocidad: {vel}%{C.RESET}")
                except ValueError:
                    print(f"  {C.ERR}Número inválido.{C.RESET}")

        elif letra in ("f","b","l","r"):
            if len(p) == 1:
                # indefinido
                if letra in _DIRS_PID:
                    _pid["dir"] = letra; _pid["vel_base"] = vel; _pid["activo"] = True
                enviar(f"{letra} {vel}")
                print(f"  {C.OK}{letra} al {vel}% — 's' para parar{C.RESET}")

            elif len(p) == 3:
                try: n = int(p[1])
                except ValueError:
                    print(f"  {C.ERR}Número inválido.{C.RESET}"); continue
                unidad = p[2]

                if unidad == "mm":
                    a1, a2 = enc["c1"], enc["c2"]
                    o1, o2 = n/M1_MM_POR_PULSO, n/M2_MM_POR_PULSO
                    if letra in _DIRS_PID:
                        _pid["dir"]=letra; _pid["vel_base"]=vel; _pid["activo"]=True
                    enviar(f"{letra} {vel}")
                    print(f"  {C.OK}{letra} → {n} mm{C.RESET}")
                    while True:
                        if abs(enc["c1"]-a1)>=o1 and abs(enc["c2"]-a2)>=o2:
                            _pid["activo"]=False; enviar("s 0")
                            d1,d2=enc["c1"]-a1,enc["c2"]-a2
                            print(f"  Alcanzado. M1:{d1:+d}p/{d1*M1_MM_POR_PULSO:+.1f}mm  M2:{d2:+d}p/{d2*M2_MM_POR_PULSO:+.1f}mm")
                            break
                        time.sleep(0.02)

                elif unidad == "ms":
                    a1, a2 = enc["c1"], enc["c2"]
                    if letra in _DIRS_PID:
                        _pid["dir"]=letra; _pid["vel_base"]=vel; _pid["activo"]=True
                    enviar(f"{letra} {vel}")
                    print(f"  {C.OK}{letra} → {n} ms{C.RESET}")
                    time.sleep(n/1000.0)
                    _pid["activo"]=False; enviar("s 0")
                    d1,d2=enc["c1"]-a1,enc["c2"]-a2
                    print(f"  Cumplido. M1:{d1:+d}p/{d1*M1_MM_POR_PULSO:+.1f}mm  M2:{d2:+d}p/{d2*M2_MM_POR_PULSO:+.1f}mm")
                else:
                    print(f"  {C.ERR}Unidad inválida: usa mm o ms{C.RESET}")
            else:
                print(f"  {C.ERR}Uso: {letra} <vel>  |  {letra} <N> mm  |  {letra} <N> ms{C.RESET}")
        else:
            print(f"  {C.ERR}Comando no reconocido: '{letra}'{C.RESET}")


# ══════════════════════════════════════════════
#  STANDALONE
# ══════════════════════════════════════════════
if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()
    if not iniciar(port=args.port, baud=args.baud):
        sys.exit(1)
    try:
        _modo_test()
    finally:
        detener()
