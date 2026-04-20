"""
principal.py  —  Código principal robot seguidor de pista
=========================================================
Integra:
  • camara_detector  → streaming MJPEG + estado (error_linea, obstaculo)
  • motor_control    → conexión ESP32 + PID velocidad + comandos

Comandos disponibles en consola:
  go       → iniciar seguimiento autónomo de pista
  stop     → detener todo
  test     → modo test de motores (f/b/l/r <vel>, mm, ms)
  exit     → salir

Uso:
  python3 principal.py
  python3 principal.py --cam-port 5000 --motor-port /dev/ttyUSB0
"""

import argparse
import threading
import time
import sys

import camara_detector
import motor_control

# ══════════════════════════════════════════════
#  PARÁMETROS PID DE DIRECCIÓN (seguimiento de línea)
# ══════════════════════════════════════════════
KP_DIR       = 0.08   # Ganancia: cuánto girar por pixel de error lateral
VEL_BASE     = 50     # Velocidad base en modo autónomo (0-100)
VEL_MIN      = 25     # Velocidad mínima por motor al corregir
VEL_MAX      = 75     # Velocidad máxima por motor al corregir
INTERVALO_S  = 0.1    # Cada cuántos segundos se recalcula la dirección

# ══════════════════════════════════════════════
#  ESTADO DEL MODO AUTÓNOMO
# ══════════════════════════════════════════════
_autonomo       = False
_autonomo_lock  = threading.Lock()

class C:
    OK   = "\033[92m"; ERR  = "\033[91m"; INFO = "\033[94m"
    WARN = "\033[93m"; RESET= "\033[0m";  BOLD = "\033[1m"


# ══════════════════════════════════════════════
#  HILO DE SEGUIMIENTO AUTÓNOMO
# ══════════════════════════════════════════════
def _hilo_autonomo(stop_ev: threading.Event):
    """
    Lee error_linea y obstaculo_detectado de la cámara y ajusta
    las velocidades de cada motor para mantener el robot centrado.

    PID de dirección (solo proporcional por ahora):
      corrección = KP_DIR * error_linea
      vel_izq    = VEL_BASE + corrección
      vel_der    = VEL_BASE - corrección

    Si error > 0 el robot está desviado a la derecha → acelera izq, frena der.
    Si error < 0 está desviado a la izquierda → acelera der, frena izq.
    """
    print(f"{C.INFO}[AUTO] Hilo de seguimiento activo.{C.RESET}")

    while not stop_ev.is_set():
        time.sleep(INTERVALO_S)

        with _autonomo_lock:
            if not _autonomo:
                motor_control.enviar_raw("s 0")
                continue

        # Leer estado de la cámara
        with camara_detector.estado_lock:
            obstaculo = camara_detector.estado["obstaculo_detectado"]
            error     = camara_detector.estado["error_linea"]

        # Obstáculo → parar
        if obstaculo:
            motor_control.enviar_raw("s 0")
            print(f"{C.WARN}[AUTO] OBSTÁCULO — detenido.{C.RESET}")
            continue

        # Sin líneas detectadas → parar y esperar
        if error is None:
            motor_control.enviar_raw("s 0")
            continue

        # Calcular velocidades corregidas
        correccion = KP_DIR * error
        vel_izq = int(max(VEL_MIN, min(VEL_MAX, VEL_BASE + correccion)))
        vel_der = int(max(VEL_MIN, min(VEL_MAX, VEL_BASE - correccion)))

        motor_control.set_velocidades(vel_izq, vel_der, "f")

    motor_control.enviar_raw("s 0")
    print(f"{C.INFO}[AUTO] Hilo de seguimiento detenido.{C.RESET}")


# ══════════════════════════════════════════════
#  CONSOLA PRINCIPAL
# ══════════════════════════════════════════════
def _consola(stop_ev_autonomo: threading.Event):
    global _autonomo

    print(f"""{C.BOLD}
╔══════════════════════════════════════════════╗
║   ROBOT SEGUIDOR DE PISTA — CONTROL          ║
╠══════════════════════════════════════════════╣
║  go     → seguimiento autónomo               ║
║  stop   → detener                            ║
║  test   → modo test de motores               ║
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
            print(f"{C.OK}[OK]{C.RESET} Seguimiento autónomo iniciado.")

        elif cmd == "stop":
            with _autonomo_lock:
                _autonomo = False
            motor_control.enviar_raw("s 0")
            print(f"{C.OK}[OK]{C.RESET} Detenido.")

        elif cmd == "test":
            with _autonomo_lock:
                _autonomo = False
            motor_control.enviar_raw("s 0")
            print(f"{C.INFO}[TEST] Entrando en modo test. Escribe 'exit' para volver.{C.RESET}")
            motor_control._modo_test()
            print(f"{C.INFO}[TEST] Volviendo al menú principal.{C.RESET}")

        elif cmd in ("exit", "quit"):
            with _autonomo_lock:
                _autonomo = False
            stop_ev_autonomo.set()
            break

        else:
            print(f"  {C.ERR}Comando no reconocido: '{cmd}'{C.RESET}")


# ══════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════
def main():
    ap = argparse.ArgumentParser(description="Robot seguidor de pista")
    ap.add_argument("--cam-host",   default="0.0.0.0")
    ap.add_argument("--cam-port",   type=int, default=5000)
    ap.add_argument("--motor-port", default=None,
                    help="Puerto serial ESP32, ej: /dev/ttyUSB0")
    ap.add_argument("--motor-baud", type=int, default=115200)
    args = ap.parse_args()

    # ── Arrancar cámara (no bloqueante — Flask corre en su propio hilo) ───
    camara_detector.iniciar(host=args.cam_host, port=args.cam_port)
    time.sleep(1)   # dar tiempo a que Flask arranque antes de continuar

    # ── Conectar motores ──────────────────────────────────────────────────
    if not motor_control.iniciar(port=args.motor_port, baud=args.motor_baud):
        print(f"{C.ERR}No se pudo conectar a la ESP32. Saliendo.{C.RESET}")
        sys.exit(1)

    # ── Hilo de seguimiento autónomo ──────────────────────────────────────
    stop_ev_autonomo = threading.Event()
    threading.Thread(
        target=_hilo_autonomo,
        args=(stop_ev_autonomo,),
        daemon=True
    ).start()

    # ── Consola (hilo principal) ──────────────────────────────────────────
    try:
        _consola(stop_ev_autonomo)
    finally:
        motor_control.detener()
        print(f"{C.INFO}Hasta luego.{C.RESET}")


if __name__ == "__main__":
    main()
