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
KP_DIR           = 0.08
KP_DIR_SOLO_ROJA = 0.15   # ganancia cuando solo ve la roja
OFFSET_CURVA     = 50     # offset en px al estar en curva viendo solo roja
VEL_BASE         = 70
VEL_MIN          = 25
VEL_MAX          = 80
INTERVALO_S      = 0.1

# ══════════════════════════════════════════════
#  PARÁMETROS DE GIROS
#  delay_ms:    navega con PID antes de iniciar el giro (ms)
#  recto_ms:    navega solo con amarilla antes del giro (ms)
#  vel:         velocidad del giro (0-100)
#  duracion_ms: duración del giro (ms)
# ══════════════════════════════════════════════
GIROS = {
    "derecha_interseccion": {
        "delay_ms":    4800,
        "recto_ms":    0,
        "vel":         60,
        "duracion_ms": 700,
    },
    "derecha_salida": {
        "delay_ms":    1000,
        "recto_ms":    9200,
        "vel":         60,
        "duracion_ms": 900,
    },
}

# ══════════════════════════════════════════════
#  PARÁMETROS DE CARGA
#  antes_ms: navega con PID tras leer tag 1 antes de detenerse
#  espera_ms: tiempo detenido esperando la carga
# ══════════════════════════════════════════════
DELAY_ANTES_CARGA_MS = 3000
DELAY_CARGA_MS       = 5000

# ══════════════════════════════════════════════
#  PARÁMETROS DE DESCARGA
#  antes_ms: navega con PID antes de detenerse (usado si antes_mm es None)
#  antes_mm: distancia en mm antes de detenerse (tiene prioridad sobre antes_ms)
#  espera_ms: tiempo detenido en la descarga
# ══════════════════════════════════════════════
DELAY_ANTES_DESCARGA_1_MS = 3000
ANTES_DESCARGA_1_MM       = None  # ej: 500 para usar encoders en lugar de ms
DELAY_DESCARGA_1_MS       = 7000

DELAY_ANTES_DESCARGA_2_MS = 3000
ANTES_DESCARGA_2_MM       = None
DELAY_DESCARGA_2_MS       = 7000

DELAY_ANTES_DESCARGA_3_MS = 3000
ANTES_DESCARGA_3_MM       = None
DELAY_DESCARGA_3_MS       = 7000

# ══════════════════════════════════════════════
#  PARÁMETROS ULTRASONIDO
#  DIST_CARGA_CM:   distancia en cm para detectar carga presente
#  DELAY_POST_CARGA_MS:   delay tras detectar carga antes de arrancar
#  DELAY_POST_DESCARGA_MS: delay tras detectar que no hay carga antes de arrancar
# ══════════════════════════════════════════════
DIST_CARGA_CM          = 10    # objeto a menos de 10cm = carga presente
DELAY_POST_CARGA_MS    = 2000  # espera tras detectar carga
DELAY_POST_DESCARGA_MS = 2000  # espera tras detectar que no hay carga

# ══════════════════════════════════════════════
#  TIEMPO RECTO SIN LÍNEAS
# ══════════════════════════════════════════════
TIEMPO_RECTO_SIN_LINEA_S = 2.0

# ══════════════════════════════════════════════
#  SECUENCIA MISIÓN 1 (descarga #1 — carril central)
# ══════════════════════════════════════════════
SECUENCIA_MISION_1 = [
    {
        "espera_tag":  5,
        "descripcion": "Navegando hacia intersección (tag 5)",
        "accion":      "giro_derecha_interseccion",
    },
    {
        "espera_tag":  6,
        "descripcion": "Navegando hacia descarga 1 (tag 6)",
        "accion":      "descarga_1",
    },
    {
        "espera_tag":  9,
        "descripcion": "Navegando hacia salida carril (tag 9)",
        "accion":      "giro_derecha_salida",
    },
]

SECUENCIA_MISION_2 = [
    {
        "espera_tag":  7,
        "descripcion": "Navegando hacia descarga 2 (tag 7)",
        "accion":      "descarga_2",
    },
]

SECUENCIA_MISION_3 = [
    {
        "espera_tag":  8,
        "descripcion": "Navegando hacia descarga 3 (tag 8)",
        "accion":      "descarga_3",
    },
]

# ══════════════════════════════════════════════
#  MAPA DE MISIONES
# ══════════════════════════════════════════════
MAPA_MISIONES = {
    2: SECUENCIA_MISION_1,
    3: SECUENCIA_MISION_2,
    4: SECUENCIA_MISION_3,
}

# ══════════════════════════════════════════════
#  ESTADO AUTÓNOMO
# ══════════════════════════════════════════════
_autonomo      = False
_autonomo_lock = threading.Lock()

class C:
    OK="\033[92m"; ERR="\033[91m"; INFO="\033[94m"
    WARN="\033[93m"; RESET="\033[0m"; BOLD="\033[1m"


# ══════════════════════════════════════════════
#  HELPERS DE NAVEGACIÓN
# ══════════════════════════════════════════════
def _navegar_normal():
    """Un ciclo de navegación. Retorna True=ok, False=stop, None=sin líneas."""
    with _autonomo_lock:
        if not _autonomo:
            return False
    obstaculo = camara_detector.estado["obstaculo_detectado"]
    error     = camara_detector.estado["error_linea"]
    if obstaculo:
        motor_control.enviar("s 0")
        return True
    if error is None:
        return None
    solo_roja = camara_detector.estado.get("solo_roja", False)
    en_curva  = camara_detector.estado.get("en_curva", False)
    kp        = KP_DIR_SOLO_ROJA if solo_roja else KP_DIR
    offset    = OFFSET_CURVA if en_curva else 0
    corr      = kp * (error - offset)
    vel_izq   = int(max(VEL_MIN, min(VEL_MAX, VEL_BASE + corr)))
    vel_der   = int(max(VEL_MIN, min(VEL_MAX, VEL_BASE - corr)))
    motor_control.set_velocidades(vel_izq, vel_der, "f")
    return True


def _navegar_pid(segundos):
    """Navega con PID durante N segundos. Se detiene si stop."""
    t0 = time.time()
    while time.time() - t0 < segundos:
        with _autonomo_lock:
            if not _autonomo:
                return
        _navegar_normal()
        time.sleep(0.05)


def _navegar_solo_amarilla(segundos):
    """Navega ignorando la roja. Sigue amarilla si la ve; si no, va recto."""
    camara_detector.estado["ignorar_roja"] = True
    t0 = time.time()
    while time.time() - t0 < segundos:
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


def _esperar_tag(tag_id):
    """Navega hasta detectar un tag. Retorna False si se detiene el sistema."""
    tag_ant     = None
    t_sin_linea = None

    while True:
        with _autonomo_lock:
            if not _autonomo:
                return False
        tag_actual = camara_detector.estado["ultimo_tag"]
        if tag_actual == tag_id and tag_ant != tag_id:
            return True
        if tag_actual != tag_id:
            tag_ant = tag_actual

        resultado = _navegar_normal()
        if resultado is None:
            if t_sin_linea is None:
                t_sin_linea = time.time()
            elif time.time() - t_sin_linea > TIEMPO_RECTO_SIN_LINEA_S:
                motor_control.enviar("s 0")
            else:
                motor_control.set_velocidades(VEL_BASE, VEL_BASE, "f")
        else:
            t_sin_linea = None
        time.sleep(0.02)


def _esperar_sin_tag(tag_id):
    """Navega hasta que el tag deja de verse. Retorna False si se detiene."""
    while True:
        with _autonomo_lock:
            if not _autonomo:
                return False
        if camara_detector.estado["ultimo_tag"] != tag_id:
            return True
        _navegar_normal()
        time.sleep(0.02)


def _sleep_con_obstaculos(segundos):
    """Espera N segundos pero pausa motores si hay obstáculo y reanuda al despejarse."""
    t0 = time.time()
    while time.time() - t0 < segundos:
        with _autonomo_lock:
            if not _autonomo:
                return
        if camara_detector.estado["obstaculo_detectado"]:
            motor_control.enviar("s 0")
            while camara_detector.estado["obstaculo_detectado"]:
                with _autonomo_lock:
                    if not _autonomo:
                        return
                time.sleep(0.05)
        time.sleep(0.05)


# ══════════════════════════════════════════════
#  RUTINAS DE MOVIMIENTO
# ══════════════════════════════════════════════
def _ejecutar_giro(config_nombre):
    cfg = GIROS[config_nombre]
    print(f"{C.INFO}[GIRO] {config_nombre} — delay:{cfg['delay_ms']}ms "
          f"recto:{cfg.get('recto_ms',0)}ms vel:{cfg['vel']} dur:{cfg['duracion_ms']}ms{C.RESET}")

    if cfg["delay_ms"] > 0:
        _navegar_pid(cfg["delay_ms"] / 1000.0)

    if cfg.get("recto_ms", 0) > 0:
        _navegar_solo_amarilla(cfg["recto_ms"] / 1000.0)

    motor_control.enviar("s 0")
    time.sleep(0.1)
    motor_control.enviar(f"r {cfg['vel']}")
    _sleep_con_obstaculos(cfg["duracion_ms"] / 1000.0)
    motor_control.enviar("s 0")
    time.sleep(0.2)
    print(f"{C.OK}[GIRO] Completado.{C.RESET}")


def _ejecutar_descarga(numero, tag_id):
    antes_ms  = [None, DELAY_ANTES_DESCARGA_1_MS,
                       DELAY_ANTES_DESCARGA_2_MS,
                       DELAY_ANTES_DESCARGA_3_MS][numero]
    antes_mm  = [None, ANTES_DESCARGA_1_MM,
                       ANTES_DESCARGA_2_MM,
                       ANTES_DESCARGA_3_MM][numero]
    espera_ms = [None, DELAY_DESCARGA_1_MS,
                       DELAY_DESCARGA_2_MS,
                       DELAY_DESCARGA_3_MS][numero]

    print(f"{C.WARN}[DESCARGA {numero}] Esperando pasar el tag...{C.RESET}")
    _esperar_sin_tag(tag_id)

    if antes_mm is not None:
        # Usar encoders para la distancia
        print(f"{C.WARN}[DESCARGA {numero}] Navegando {antes_mm}mm con encoders{C.RESET}")
        M1_MM = (2 * 3.14159 * 30.0) / 3816
        M2_MM = (2 * 3.14159 * 30.0) / 2689
        obj1  = antes_mm / M1_MM
        obj2  = antes_mm / M2_MM
        c1_0  = motor_control.enc["c1"]
        c2_0  = motor_control.enc["c2"]
        while True:
            with _autonomo_lock:
                if not _autonomo:
                    return
            d1 = abs(motor_control.enc["c1"] - c1_0)
            d2 = abs(motor_control.enc["c2"] - c2_0)
            if d1 >= obj1 and d2 >= obj2:
                break
            _navegar_normal()
            time.sleep(0.02)
    else:
        print(f"{C.WARN}[DESCARGA {numero}] Navegando — {antes_ms}ms{C.RESET}")
        _navegar_pid(antes_ms / 1000.0)

    print(f"{C.WARN}[DESCARGA {numero}] Esperando retirar carga — ultrasonido > {DIST_CARGA_CM}cm{C.RESET}")
    motor_control.enviar("s 0")
    while motor_control.enc["dist_cm"] <= DIST_CARGA_CM:
        with _autonomo_lock:
            if not _autonomo:
                return
        time.sleep(0.05)
    print(f"{C.WARN}[DESCARGA {numero}] Carga retirada — esperando {DELAY_POST_DESCARGA_MS}ms{C.RESET}")
    _sleep_con_obstaculos(DELAY_POST_DESCARGA_MS / 1000.0)
    print(f"{C.OK}[DESCARGA {numero}] Completada.{C.RESET}")


# ══════════════════════════════════════════════
#  HILO AUTÓNOMO
# ══════════════════════════════════════════════
def _hilo_autonomo(stop_ev):
    while not stop_ev.is_set():
        time.sleep(INTERVALO_S)

        with _autonomo_lock:
            if not _autonomo:
                continue

        # FASE 1: navegar hasta tag 1 (estación de carga)
        print(f"{C.INFO}[AUTO] Navegando a estación de carga...{C.RESET}")
        if not _esperar_tag(1):
            continue

        # FASE 2: navegar hasta dejar de ver tag 1 y esperar carga
        print(f"{C.WARN}[CARGA] Esperando pasar el tag...{C.RESET}")
        _esperar_sin_tag(1)
        print(f"{C.WARN}[CARGA] Navegando al punto — {DELAY_ANTES_CARGA_MS}ms{C.RESET}")
        _navegar_pid(DELAY_ANTES_CARGA_MS / 1000.0)
        print(f"{C.WARN}[CARGA] Esperando carga — ultrasonido < {DIST_CARGA_CM}cm{C.RESET}")
        motor_control.enviar("s 0")
        while motor_control.enc["dist_cm"] > DIST_CARGA_CM:
            with _autonomo_lock:
                if not _autonomo:
                    break
            time.sleep(0.05)
        print(f"{C.WARN}[CARGA] Carga detectada — esperando {DELAY_POST_CARGA_MS}ms{C.RESET}")
        _sleep_con_obstaculos(DELAY_POST_CARGA_MS / 1000.0)
        print(f"{C.OK}[CARGA] Carga lista.{C.RESET}")

        # FASE 3: navegar hasta leer tag de misión
        print(f"{C.INFO}[AUTO] Esperando tag de misión...{C.RESET}")
        secuencia  = None
        tag_mision = None
        tag_ant    = camara_detector.estado.get("ultimo_tag")
        while secuencia is None:
            with _autonomo_lock:
                if not _autonomo:
                    break
            tag_actual = camara_detector.estado["ultimo_tag"]
            if tag_actual in MAPA_MISIONES and tag_actual != tag_ant:
                secuencia  = MAPA_MISIONES[tag_actual]
                tag_mision = tag_actual
            if tag_actual not in MAPA_MISIONES:
                tag_ant = tag_actual
            resultado = _navegar_normal()
            if resultado is None:
                motor_control.set_velocidades(VEL_BASE, VEL_BASE, "f")
            time.sleep(INTERVALO_S)

        if secuencia is None:
            continue

        print(f"{C.INFO}[MISIÓN] Tag {tag_mision} — iniciando secuencia{C.RESET}")

        # FASE 4: ejecutar secuencia de misión paso a paso
        for i, paso in enumerate(secuencia):
            tag_esperado = paso["espera_tag"]
            accion       = paso.get("accion")
            print(f"{C.INFO}[MISIÓN] Paso {i+1}: {paso['descripcion']}{C.RESET}")

            if not _esperar_tag(tag_esperado):
                break

            if accion == "giro_derecha_interseccion":
                _ejecutar_giro("derecha_interseccion")
            elif accion == "giro_derecha_salida":
                _ejecutar_giro("derecha_salida")
            elif accion == "descarga_1":
                _ejecutar_descarga(1, 6)
            elif accion == "descarga_2":
                _ejecutar_descarga(2, 7)
            elif accion == "descarga_3":
                _ejecutar_descarga(3, 8)

        print(f"{C.OK}[MISIÓN] Recorrido completo. Volviendo a carga...{C.RESET}")

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
