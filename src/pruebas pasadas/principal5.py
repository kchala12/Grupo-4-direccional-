"""
principal.py — Código principal robot seguidor de pista
========================================================
Arranca cámara (streaming HTTP) y motores en paralelo.
La terminal queda libre para dar comandos.

Comandos:
  go     → seguimiento autónomo de pista
  stop   → detener
  test   → modo test de motores
  exit   → salir

Uso:
  python3 principal.py
  python3 principal.py --motor-port /dev/ttyUSB0
"""

import threading
import time
import sys
import argparse

import camara_detector
import motor_control

# ══════════════════════════════════════════════
#  PARÁMETROS PID DIRECCIÓN
# ══════════════════════════════════════════════
KP_DIR      = 0.08
VEL_BASE    = 50
VEL_MIN     = 25
VEL_MAX     = 75
INTERVALO_S = 0.1

# ══════════════════════════════════════════════
#  ESTADO AUTÓNOMO
# ══════════════════════════════════════════════
_autonomo      = False
_autonomo_lock = threading.Lock()

class C:
    OK="\033[92m"; ERR="\033[91m"; INFO="\033[94m"
    WARN="\033[93m"; RESET="\033[0m"; BOLD="\033[1m"


# ══════════════════════════════════════════════
#  HILO AUTÓNOMO
# ══════════════════════════════════════════════
def _hilo_autonomo(stop_ev):
    while not stop_ev.is_set():
        time.sleep(INTERVALO_S)

        with _autonomo_lock:
            activo = _autonomo

        if not activo:
            continue

        with camara_detector.estado_lock:
            obstaculo = camara_detector.estado["obstaculo_detectado"]
            error     = camara_detector.estado["error_linea"]

        if obstaculo:
            motor_control.enviar("s 0")
            print(f"\r{C.WARN}[AUTO] OBSTÁCULO — detenido{C.RESET}          ")
            continue

        if error is None:
            motor_control.enviar("s 0")
            continue

        corr    = KP_DIR * error
        vel_izq = int(max(VEL_MIN, min(VEL_MAX, VEL_BASE + corr)))
        vel_der = int(max(VEL_MIN, min(VEL_MAX, VEL_BASE - corr)))
        motor_control.set_velocidades(vel_izq, vel_der, "f")

    motor_control.enviar("s 0")


# ══════════════════════════════════════════════
#  CONSOLA
# ══════════════════════════════════════════════
def _consola(stop_ev):
    global _autonomo

    print(f"""{C.BOLD}
╔══════════════════════════════════════════════╗
║  ROBOT SEGUIDOR DE PISTA                     ║
║  go     → seguimiento autónomo               ║
║  stop   → detener                            ║
║  fondo  → recapturar fondo de referencia     ║
║  test   → test de motores                    ║
║  exit   → salir                              ║
╚══════════════════════════════════════════════╝{C.RESET}
""")

    while True:
        try:
            cmd = input(f"{C.BOLD}>{C.RESET} ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            cmd = "exit"

        if not cmd:
            continue

        if cmd == "go":
            with _autonomo_lock:
                _autonomo = True
            print(f"{C.OK}[OK] Seguimiento autónomo iniciado.{C.RESET}")

        elif cmd == "stop":
            with _autonomo_lock:
                _autonomo = False
            motor_control.enviar("s 0")
            print(f"{C.OK}[OK] Detenido.{C.RESET}")

        elif cmd == "fondo":
            camara_detector.resetear_fondo()
            print(f"{C.OK}[OK] Recapturando fondo...{C.RESET}")

        elif cmd == "test":
            with _autonomo_lock:
                _autonomo = False
            motor_control.enviar("s 0")
            motor_control._modo_test()

        elif cmd in ("exit", "quit"):
            with _autonomo_lock:
                _autonomo = False
            stop_ev.set()
            break

        else:
            print(f"  {C.ERR}Comando no reconocido: '{cmd}'{C.RESET}")


# ══════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cam-host",   default="0.0.0.0")
    ap.add_argument("--cam-port",   type=int, default=5000)
    ap.add_argument("--motor-port", default=None)
    ap.add_argument("--motor-baud", type=int, default=115200)
    args = ap.parse_args()

    # Cámara — no bloquea, arranca sus propios hilos daemon
    camara_detector.iniciar(host=args.cam_host, port=args.cam_port)

    # Motores
    if not motor_control.iniciar(port=args.motor_port, baud=args.motor_baud):
        print(f"{C.ERR}No se pudo conectar a la ESP32. Saliendo.{C.RESET}")
        sys.exit(1)

    # Hilo autónomo
    stop_ev = threading.Event()
    threading.Thread(target=_hilo_autonomo, args=(stop_ev,), daemon=True).start()

    # Consola en hilo principal (mantiene el proceso vivo)
    try:
        _consola(stop_ev)
    finally:
        motor_control.detener()
        print(f"{C.INFO}Hasta luego.{C.RESET}")


if __name__ == "__main__":
    main()
