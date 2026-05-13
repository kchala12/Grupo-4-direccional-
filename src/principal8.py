"""
principal.py — Código principal robot seguidor de pista
========================================================
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
#  delay_ms:    navega con PID antes de iniciar el giro (ms)
#  recto_ms:    avanza siguiendo solo amarilla antes del giro (ms)
#               si no hay amarilla, va recto
#  vel:         velocidad del giro — más alto = más cerrado (0-100)
#  duracion_ms: cuánto tiempo dura el giro (ms)
# ══════════════════════════════════════════════
GIROS = {
    "derecha_interseccion": {
        "delay_ms":    7500,
        "recto_ms":    0,
        "vel":         60,
        "duracion_ms": 900,
    },
    "derecha_salida": {
        "delay_ms":    12000,
        "recto_ms":    2500,
        "vel":         60,
        "duracion_ms": 900,
    },
}

# ══════════════════════════════════════════════
#  PARÁMETROS DE DESCARGA
#  llegada_s: navega con PID hasta llegar al punto (s)
#  espera_s:  tiempo detenido simulando descarga (s)
# ══════════════════════════════════════════════
DELAY_LLEGADA_DESCARGA_S = 3
DELAY_DESCARGA_S         = 7

# ══════════════════════════════════════════════
#  SECUENCIA MISIÓN 1 (descarga #1 — carril central)
#  espera_tag: tag que activa este paso
#  accion:     qué hacer al detectarlo
# ══════════════════════════════════════════════
SECUENCIA_MISION_1 = [
    {
        "espera_tag":  2,
        "descripcion": "Esperando misión (tag 2)",
    },
    {
        "espera_tag":  5,
        "descripcion": "Navegando hacia intersección (tag 5)",
        "accion":      "giro_derecha_interseccion",
    },
    {
        "espera_tag":  6,
        "descripcion": "Navegando hacia descarga 1 (tag 6)",
        "accion":      "descarga",
    },
    {
        "espera_tag":  9,
        "descripcion": "Navegando hacia salida carril (tag 9)",
        "accion":      "giro_derecha_salida",
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
#  HELPERS PID
# ══════════════════════════════════════════════
def _navegar_pid(segundos):
    """Navega con PID completo durante N segundos."""
    t0 = time.time()
    while time.time() - t0 < segundos:
        with camara_detector.estado_lock:
            error = camara_detector.estado["error_linea"]
        if error is not None:
            corr    = KP_DIR * error
            vel_izq = int(max(VEL_MIN, min(VEL_MAX, VEL_BASE + corr)))
            vel_der = int(max(VEL_MIN, min(VEL_MAX, VEL_BASE - corr)))
            motor_control.set_velocidades(vel_izq, vel_der, "f")
        time.sleep(0.05)


def _navegar_solo_amarilla(segundos):
    """
    Navega durante N segundos ignorando la línea roja.
    Sigue la amarilla si la ve; si no, va recto.
    """
    camara_detector.estado["ignorar_roja"] = True
    t0 = time.time()
    while time.time() - t0 < segundos:
        with camara_detector.estado_lock:
            error = camara_detector.estado["error_linea"]
        if error is not None:
            corr    = KP_DIR * error
            vel_izq = int(max(VEL_MIN, min(VEL_MAX, VEL_BASE + corr)))
            vel_der = int(max(VEL_MIN, min(VEL_MAX, VEL_BASE - corr)))
            motor_control.set_velocidades(vel_izq, vel_der, "f")
        else:
            motor_control.set_velocidades(VEL_BASE, VEL_BASE, "f")
        time.sleep(0.05)
    camara_detector.estado["ignorar_roja"] = False


# ══════════════════════════════════════════════
#  RUTINAS DE MOVIMIENTO
# ══════════════════════════════════════════════
def _ejecutar_giro(config_nombre):
    cfg = GIROS[config_nombre]
    print(f"{C.INFO}[GIRO] {config_nombre} — delay:{cfg['delay_ms']}ms "
          f"recto:{cfg.get('recto_ms',0)}ms vel:{cfg['vel']} dur:{cfg['duracion_ms']}ms{C.RESET}")

    # Fase 1: navegar con PID normal
    if cfg["delay_ms"] > 0:
        _navegar_pid(cfg["delay_ms"] / 1000.0)

    # Fase 2: navegar solo con amarilla (o recto si no la ve)
    if cfg.get("recto_ms", 0) > 0:
        _navegar_solo_amarilla(cfg["recto_ms"] / 1000.0)

    # Fase 3: girar
    motor_control.enviar("s 0")
    time.sleep(0.1)
    motor_control.enviar(f"r {cfg['vel']}")
    time.sleep(cfg["duracion_ms"] / 1000.0)
    motor_control.enviar("s 0")
    time.sleep(0.2)
    print(f"{C.OK}[GIRO] Completado.{C.RESET}")


def _ejecutar_descarga():
    # Fase 1: navegar con PID hasta el punto de descarga
    print(f"{C.WARN}[DESCARGA] Navegando al punto — {DELAY_LLEGADA_DESCARGA_S}s{C.RESET}")
    _navegar_pid(DELAY_LLEGADA_DESCARGA_S)

    # Fase 2: detenerse y esperar
    print(f"{C.WARN}[DESCARGA] Descargando — {DELAY_DESCARGA_S}s{C.RESET}")
    motor_control.enviar("s 0")
    time.sleep(DELAY_DESCARGA_S)
    print(f"{C.OK}[DESCARGA] Completada.{C.RESET}")


# ══════════════════════════════════════════════
#  HILO AUTÓNOMO
# ══════════════════════════════════════════════
def _hilo_autonomo(stop_ev):
    paso_mision   = 0
    tag_visto_ant = None

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

        # Procesar tag si corresponde al paso actual
        if paso_mision < len(SECUENCIA_MISION_1):
            paso         = SECUENCIA_MISION_1[paso_mision]
            tag_esperado = paso["espera_tag"]

            if tag_actual == tag_esperado and tag_visto_ant != tag_esperado:
                accion = paso.get("accion")
                print(f"{C.INFO}[MISIÓN] Paso {paso_mision+1}: tag {tag_esperado} — {paso['descripcion']}{C.RESET}")

                if accion == "giro_derecha_interseccion":
                    _ejecutar_giro("derecha_interseccion")
                elif accion == "giro_derecha_salida":
                    _ejecutar_giro("derecha_salida")
                elif accion == "descarga":
                    _ejecutar_descarga()

                paso_mision += 1
                if paso_mision >= len(SECUENCIA_MISION_1):
                    print(f"{C.OK}[MISIÓN] Recorrido completo.{C.RESET}")

        tag_visto_ant = tag_actual

        # Obstáculo
        if obstaculo:
            motor_control.enviar("s 0")
            continue

        # Sin líneas
        if error is None:
            motor_control.enviar("s 0")
            continue

        # Navegación normal
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
            camara_detector.estado["ignorar_roja"] = False
            motor_control.enviar("s 0")
            print(f"{C.OK}[OK] Detenido.{C.RESET}")

        elif cmd == "fondo":
            camara_detector.resetear_fondo()
            print(f"{C.OK}[OK] Recapturando fondo...{C.RESET}")

        elif cmd == "test":
            with _autonomo_lock:
                _autonomo = False
            camara_detector.estado["ignorar_roja"] = False
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
