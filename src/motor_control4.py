"""
motor_control.py  —  Librería control motores JGY370 — Raspberry Pi
=====================================================================
Uso como librería:
    import motor_control
    motor_control.iniciar()          # conecta a la ESP32 y arranca hilos

    # Desde el código principal:
    motor_control.enviar_cmd("f 60")   # adelante al 60%
    motor_control.enviar_cmd("b 60")   # atrás
    motor_control.enviar_cmd("l 50")   # izquierda
    motor_control.enviar_cmd("r 50")   # derecha
    motor_control.enviar_cmd("s 0")    # stop
    motor_control.set_velocidades(55, 45, "f")  # PID dirección desde principal

    # Leer encoders:
    motor_control.enc["c1"]
    motor_control.enc["c2"]

Modo test interactivo (sin WASD):
    python3 motor_control.py
    Comandos: f|b|l|r <vel>  /  f|b|l|r <N> mm  /  f|b|l|r <N> ms  /  s  /  exit
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
M1_PULSOS_POR_VUELTA = 297
M2_PULSOS_POR_VUELTA = 2682
RADIO_MM             = 30.0

M1_MM_POR_PULSO = (2 * math.pi * RADIO_MM) / M1_PULSOS_POR_VUELTA
M2_MM_POR_PULSO = (2 * math.pi * RADIO_MM) / M2_PULSOS_POR_VUELTA

# ══════════════════════════════════════════════
#  PARÁMETROS PID DE VELOCIDAD
# ══════════════════════════════════════════════
CTRL_KP             = 1.0
CTRL_INTERVALO_S    = 0.1
CTRL_CORRECCION_MAX = 20

# ══════════════════════════════════════════════
#  ESTADO INTERNO
# ══════════════════════════════════════════════
enc = {"c1": 0, "c2": 0}

pid_estado = {
    "activo":   False,
    "vel_base": 50,
    "vel_m1":   50,
    "vel_m2":   50,
    "dir":      "",
}

_ser: serial.Serial | None = None
_ser_lock = threading.Lock()

# Colores terminal
class C:
    OK   = "\033[92m"; ERR  = "\033[91m"; INFO = "\033[94m"
    WARN = "\033[93m"; RESET= "\033[0m";  BOLD = "\033[1m"
    DIM  = "\033[2m";  CLEAR= "\033[2J\033[H"

# La ESP32 espera letras directas: f, b, l, r, s
# (no palabras en español — eso causaba que ningún comando fuera reconocido)
_DIR_MAP = {"f": "f", "b": "b", "l": "l", "r": "r", "s": "s"}


# ══════════════════════════════════════════════
#  COMUNICACIÓN SERIAL
# ══════════════════════════════════════════════
def enviar_raw(cmd: str):
    """Envía un comando crudo a la ESP32 (formato que ella entiende)."""
    with _ser_lock:
        if _ser and _ser.is_open:
            _ser.write((cmd + "\n").encode("utf-8"))

def enviar_cmd(cmd: str):
    """
    Envía un comando usando letras: 'f 60', 'b 50', 'l 40', 'r 40', 's 0'.
    Traduce la letra a la palabra que espera la ESP32.
    """
    partes = cmd.strip().lower().split()
    if not partes:
        return
    letra = partes[0]
    vel   = partes[1] if len(partes) > 1 else "0"
    dir_esp = _DIR_MAP.get(letra, letra)
    enviar_raw(f"{dir_esp} {vel}")

def set_velocidades(vel_izq: int, vel_der: int, dir_letra: str):
    """
    Envía velocidades independientes por motor (usado por el PID de dirección
    del código principal). dir_letra: 'f' o 'b'.
    """
    # La ESP32 espera "pid vel1 vel2 f" o "pid vel1 vel2 b"
    dir_ok = dir_letra if dir_letra in ("f", "b") else "f"
    pwm_cmd = f"pid {vel_izq} {vel_der} {dir_ok}"
    enviar_raw(pwm_cmd)
    pid_estado["vel_m1"] = vel_izq
    pid_estado["vel_m2"] = vel_der


# ══════════════════════════════════════════════
#  HILOS INTERNOS
# ══════════════════════════════════════════════
def _hilo_lector(stop_ev: threading.Event):
    while not stop_ev.is_set():
        try:
            with _ser_lock:
                waiting = _ser.in_waiting if _ser else 0
            if waiting:
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


def _hilo_controlador(stop_ev: threading.Event):
    tiempo_inicio  = 0.0
    pulsos_inicio1 = 0
    pulsos_inicio2 = 0
    tasa_esperada1 = 0.0
    tasa_esperada2 = 0.0

    while not stop_ev.is_set():
        time.sleep(CTRL_INTERVALO_S)

        if not pid_estado["activo"]:
            tiempo_inicio  = 0.0
            tasa_esperada1 = 0.0
            tasa_esperada2 = 0.0
            continue

        if tiempo_inicio == 0.0:
            tiempo_inicio  = time.time()
            pulsos_inicio1 = enc["c1"]
            pulsos_inicio2 = enc["c2"]
            continue

        t_elapsed = time.time() - tiempo_inicio
        reales1   = abs(enc["c1"] - pulsos_inicio1)
        reales2   = abs(enc["c2"] - pulsos_inicio2)

        if tasa_esperada1 == 0.0 and reales1 > 0:
            tasa_esperada1 = reales1 / t_elapsed
        if tasa_esperada2 == 0.0 and reales2 > 0:
            tasa_esperada2 = reales2 / t_elapsed
        if tasa_esperada1 == 0.0 or tasa_esperada2 == 0.0:
            continue

        esperados1 = tasa_esperada1 * t_elapsed
        esperados2 = tasa_esperada2 * t_elapsed
        error1     = reales1 - esperados1
        error2     = reales2 - esperados2

        corr1    = max(-CTRL_CORRECCION_MAX, min(CTRL_CORRECCION_MAX, CTRL_KP * error1))
        corr2    = max(-CTRL_CORRECCION_MAX, min(CTRL_CORRECCION_MAX, CTRL_KP * error2))
        base     = pid_estado["vel_base"]
        nueva_m1 = int(max(0, min(100, base - corr1)))
        nueva_m2 = int(max(0, min(100, base - corr2)))

        pid_estado["vel_m1"] = nueva_m1
        pid_estado["vel_m2"] = nueva_m2

        direccion = pid_estado.get("dir", "")
        if direccion:
            enviar_raw(f"pid {nueva_m1} {nueva_m2} {direccion}")


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


_stop_ev: threading.Event | None = None

def iniciar(port: str | None = None, baud: int = 115200):
    """
    Conecta a la ESP32 y arranca los hilos de lectura y control.
    Llama esto una sola vez desde el código principal.
    """
    global _ser, _stop_ev

    puerto = port or _buscar_puerto()
    if not puerto:
        print(f"{C.ERR}[MOTORES] No se detectó ESP32.{C.RESET}")
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
    """Para los hilos y cierra la conexión serial."""
    if _stop_ev:
        _stop_ev.set()
    try:
        pid_estado["activo"] = False
        enviar_raw("s 0")
        time.sleep(0.1)
    except Exception:
        pass
    if _ser and _ser.is_open:
        _ser.close()
        print(f"{C.INFO}[MOTORES] Desconectado.{C.RESET}")


# ══════════════════════════════════════════════
#  MODO TEST INTERACTIVO (standalone)
# ══════════════════════════════════════════════
_DIRS_CON_PID = ("f", "b")

def _modo_test():
    print(f"""{C.BOLD}
╔══════════════════════════════════════════════╗
║   MOTOR CONTROL — MODO TEST                  ║
╠══════════════════════════════════════════════╣
║  Movimiento:                                 ║
║    f|b|l|r <vel>        indefinido           ║
║    f|b|l|r <N> mm       N milímetros         ║
║    f|b|l|r <N> ms       N milisegundos       ║
║    s                    stop                 ║
║  Config:                                     ║
║    vel <0-100>          velocidad global     ║
║    reset                encoders a cero      ║
║    exit                 salir                ║
╚══════════════════════════════════════════════╝{C.RESET}
""")

    velocidad = 50

    while True:
        try:
            cmd = input(f"{C.BOLD}test>{C.RESET} ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            break

        if not cmd:
            continue
        if cmd in ("exit", "quit"):
            break

        partes = cmd.split()
        letra  = partes[0]

        # Stop
        if letra == "s":
            pid_estado["activo"] = False
            enviar_raw("s 0")
            print(f"  {C.OK}[OK]{C.RESET} Stop.")
            continue

        # Reset encoders
        if letra == "reset":
            enviar_raw("reset 0")
            print(f"  {C.OK}[OK]{C.RESET} Encoders reseteados.")
            continue

        # Velocidad global
        if letra == "vel":
            if len(partes) != 2:
                print(f"  {C.ERR}Uso: vel <0-100>{C.RESET}"); continue
            try:
                velocidad = int(partes[1])
                pid_estado["vel_base"] = velocidad
                print(f"  {C.OK}[OK]{C.RESET} Velocidad: {velocidad}%")
            except ValueError:
                print(f"  {C.ERR}Número inválido.{C.RESET}")
            continue

        # Direcciones
        if letra not in _DIR_MAP or letra == "s":
            print(f"  {C.ERR}Comando no reconocido: '{letra}'{C.RESET}"); continue

        dir_esp = _DIR_MAP[letra]

        # Sin unidad → movimiento indefinido
        if len(partes) == 1:
            if dir_esp in _DIRS_CON_PID:
                pid_estado["dir"]      = dir_esp
                pid_estado["vel_base"] = velocidad
                pid_estado["activo"]   = True
            enviar_raw(f"{dir_esp} {velocidad}")
            print(f"  {C.OK}[OK]{C.RESET} {dir_esp} al {velocidad}% — indefinido (usa 's' para parar)")
            continue

        # Con unidad mm o ms
        if len(partes) == 3:
            try:
                valor = int(partes[1])
            except ValueError:
                print(f"  {C.ERR}Número inválido.{C.RESET}"); continue
            unidad = partes[2]

            if unidad == "mm":
                antes1, antes2 = enc["c1"], enc["c2"]
                obj1 = valor / M1_MM_POR_PULSO
                obj2 = valor / M2_MM_POR_PULSO
                if dir_esp in _DIRS_CON_PID:
                    pid_estado["dir"]      = dir_esp
                    pid_estado["vel_base"] = velocidad
                    pid_estado["activo"]   = True
                enviar_raw(f"{dir_esp} {velocidad}")
                print(f"  {C.OK}[OK]{C.RESET} {dir_esp} al {velocidad}% → {valor} mm")
                while True:
                    if abs(enc["c1"]-antes1) >= obj1 and abs(enc["c2"]-antes2) >= obj2:
                        pid_estado["activo"] = False
                        enviar_raw("s 0")
                        d1 = enc["c1"] - antes1
                        d2 = enc["c2"] - antes2
                        print(f"  {C.OK}[AUTO]{C.RESET} Distancia alcanzada.")
                        print(f"    M1: {d1:+d} pulsos | {d1*M1_MM_POR_PULSO:+.1f} mm")
                        print(f"    M2: {d2:+d} pulsos | {d2*M2_MM_POR_PULSO:+.1f} mm")
                        break
                    time.sleep(0.02)

            elif unidad == "ms":
                antes1, antes2 = enc["c1"], enc["c2"]
                if dir_esp in _DIRS_CON_PID:
                    pid_estado["dir"]      = dir_esp
                    pid_estado["vel_base"] = velocidad
                    pid_estado["activo"]   = True
                enviar_raw(f"{dir_esp} {velocidad}")
                print(f"  {C.OK}[OK]{C.RESET} {dir_esp} al {velocidad}% → {valor} ms")
                time.sleep(valor / 1000.0)
                pid_estado["activo"] = False
                enviar_raw("s 0")
                d1 = enc["c1"] - antes1
                d2 = enc["c2"] - antes2
                print(f"  {C.OK}[AUTO]{C.RESET} Tiempo cumplido.")
                print(f"    M1: {d1:+d} pulsos | {d1*M1_MM_POR_PULSO:+.1f} mm")
                print(f"    M2: {d2:+d} pulsos | {d2*M2_MM_POR_PULSO:+.1f} mm")
            else:
                print(f"  {C.ERR}Unidad inválida. Usa 'mm' o 'ms'.{C.RESET}")
            continue

        print(f"  {C.ERR}Formato: {letra} <vel> | {letra} <N> mm | {letra} <N> ms{C.RESET}")


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
