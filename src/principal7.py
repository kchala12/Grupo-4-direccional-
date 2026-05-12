"""
principal.py — Código principal robot seguidor de pista
========================================================
Arranca cámara (streaming HTTP) y motores en paralelo.

Comandos:
  go     → seguimiento autónomo
  stop   → detener
  fondo  → recapturar fondo de referencia
  test   → test de motores
  exit   → salir
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
#  PARÁMETROS DE GIROS
#  Cada giro tiene 3 valores ajustables:
#    delay_ms  → tiempo de espera antes de iniciar el giro (ms)
#    vel       → velocidad del giro (0-100), controla qué tan sharp es
#    duracion_ms → cuánto tiempo gira antes de volver a navegación (ms)
# ══════════════════════════════════════════════
GIROS = {
    # Giro derecha en intersección (tag 5 → entrar al carril central)
    "derecha_interseccion": {
        "delay_ms":    300,
        "vel":          60,
        "duracion_ms": 900,
    },
    # Giro derecha al salir del carril central (tag 9 → volver a carriles externos)
    "derecha_salida": {
        "delay_ms":    300,
        "vel":          60,
        "duracion_ms": 900,
    },
}

# ══════════════════════════════════════════════
#  DELAY DE DESCARGA (simula tiempo de descarga)
# ══════════════════════════════════════════════
DELAY_DESCARGA_S = 7

# ══════════════════════════════════════════════
#  SECUENCIA DE MISIÓN
#  Define qué tags escucha el robot según el paso actual
#  y qué acción ejecuta al detectar cada uno.
# ══════════════════════════════════════════════
SECUENCIA_MISION_1 = [
    {
        "espera_tag": 2,
        "descripcion": "Esperando misión (tag 2)",
    },
    {
        "espera_tag": 5,
        "descripcion": "Navegando hacia intersección (esperando tag 5)",
        "accion": "giro_derecha_interseccion",
    },
    {
        "espera_tag": 6,
        "descripcion": "Navegando hacia descarga 1 (esperando tag 6)",
        "accion": "descarga",
    },
    {
        "espera_tag": 9,
        "descripcion": "Navegando hacia salida carril (esperando tag 9)",
        "accion": "giro_derecha_salida",
    },
]

# ══════════════════════════════════════════════
#  ESTADO AUTÓNOMO
# ══════════════════════════════════════════════
_autonomo      = False
_autonomo_lock = threading.Lock()

class C:
    OK="\033[92m"; ERR="\033[91m"; INFO="\033[94m"
    WARN="\033[93m"; RESET="\033[0m"; BOLD="\033[1m"


# ══════════════════════════════════════════════
#  RUTINAS DE MOVIMIENTO
# ══════════════════════════════════════════════
def _ejecutar_giro(config_nombre):
    """Ejecuta un giro con los parámetros definidos en GIROS."""
    cfg = GIROS[config_nombre]
    print(f"{C.INFO}[GIRO] {config_nombre} — delay:{cfg['delay_ms']}ms vel:{cfg['vel']} dur:{cfg['duracion_ms']}ms{C.RESET}")

    # Detener antes del giro
    motor_control.enviar("s 0")
    time.sleep(cfg["delay_ms"] / 1000.0)

    # Girar
    motor_control.enviar(f"r {cfg['vel']}")
    time.sleep(cfg["duracion_ms"] / 1000.0)

    # Detener al terminar
    motor_control.enviar("s 0")
    time.sleep(0.2)
    print(f"{C.OK}[GIRO] Completado.{C.RESET}")


def _ejecutar_descarga():
    """Simula el proceso de descarga con un delay."""
    print(f"{C.WARN}[DESCARGA] Detenido para descarga — {DELAY_DESCARGA_S}s{C.RESET}")
    motor_control.enviar("s 0")
    time.sleep(DELAY_DESCARGA_S)
    print(f"{C.OK}[DESCARGA] Completada, reanudando.{C.RESET}")


# ══════════════════════════════════════════════
#  HILO AUTÓNOMO
# ══════════════════════════════════════════════
def _hilo_autonomo(stop_ev):
    paso_mision    = 0      # índice en SECUENCIA_MISION_1
    tag_visto_ant  = None   # último tag visto (para detectar flanco de subida)
    en_rutina      = False  # True mientras ejecuta un giro o descarga

    while not stop_ev.is_set():
        time.sleep(INTERVALO_S)

        with _autonomo_lock:
            activo = _autonomo
        if not activo:
            paso_mision   = 0
            tag_visto_ant = None
            continue

        with camara_detector.estado_lock:
            obstaculo  = camara_detector.estado["obstaculo_detectado"]
            error      = camara_detector.estado["error_linea"]
            tag_actual = camara_detector.estado["ultimo_tag"]

        # ── Leer tag actual de la secuencia ──────────────────────────────
        if paso_mision < len(SECUENCIA_MISION_1):
            paso = SECUENCIA_MISION_1[paso_mision]
            tag_esperado = paso["espera_tag"]

            # Flanco de subida: tag apareció y no lo habíamos visto antes
            if tag_actual == tag_esperado and tag_visto_ant != tag_esperado:
                accion = paso.get("accion")
                print(f"{C.INFO}[MISIÓN] Paso {paso_mision+1}: tag {tag_esperado} detectado — {paso['descripcion']}{C.RESET}")

                if accion == "giro_derecha_interseccion":
                    _ejecutar_giro("derecha_interseccion")
                elif accion == "giro_derecha_salida":
                    _ejecutar_giro("derecha_salida")
                elif accion == "descarga":
                    _ejecutar_descarga()

                paso_mision += 1
                if paso_mision >= len(SECUENCIA_MISION_1):
                    print(f"{C.OK}[MISIÓN] Recorrido completo.{C.RESET}")

        # Actualizar tag anterior
        tag_visto_ant = tag_actual

        # ── Obstáculo ────────────────────────────────────────────────────
        if obstaculo:
            motor_control.enviar("s 0")
            continue

        # ── Sin líneas ───────────────────────────────────────────────────
        if error is None:
            motor_control.enviar("s 0")
            continue

        # ── Navegación normal ─────────────────────────────────────────────
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

    camara_detector.iniciar(host=args.cam_host, port=args.cam_port)

    if not motor_control.iniciar(port=args.motor_port, baud=args.motor_baud):
        print(f"{C.ERR}No se pudo conectar a la ESP32. Saliendo.{C.RESET}")
        sys.exit(1)

    stop_ev = threading.Event()
    threading.Thread(target=_hilo_autonomo, args=(stop_ev,), daemon=True).start()

    try:
        _consola(stop_ev)
    finally:
        motor_control.detener()
        print(f"{C.INFO}Hasta luego.{C.RESET}")


if __name__ == "__main__":
    main()
