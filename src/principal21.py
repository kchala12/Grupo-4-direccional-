"""
principal.py — Código principal robot seguidor de pista
========================================================
Comandos:
  go     → seguimiento autónomo
  stop   → detener
  fondo  → recapturar fondo de referencia
  test   → test de motores
  ir     → ver estado sensor infrarrojo
  ultra  → ver distancia ultrasonido
  exit   → salir
"""

import threading
import time
import sys
import argparse

import camara_detector27 as camara_detector
import motor_control9 as motor_control

try:
    import RPi.GPIO as GPIO
    GPIO_DISPONIBLE = True
except ImportError:
    GPIO_DISPONIBLE = False

# ══════════════════════════════════════════════
#  SENSOR INFRARROJO
#  IR_DETECCION_VALUE: 1=activo en HIGH, 0=activo en LOW
# ══════════════════════════════════════════════
IR_PIN             = 17
IR_DETECCION_VALUE = 0
IR_CICLOS_CONFIRM  = 14
IR_INTERVALO_S     = 0.05

_ir_lock    = threading.Lock()
_ir_detecta = False

def _hilo_ir():
    global _ir_detecta
    contador = 0
    while True:
        if GPIO_DISPONIBLE:
            lectura = GPIO.input(IR_PIN)
            if lectura == IR_DETECCION_VALUE:
                contador = min(contador + 1, IR_CICLOS_CONFIRM)
            else:
                contador = max(contador - 1, 0)
        with _ir_lock:
            _ir_detecta = (contador >= IR_CICLOS_CONFIRM)
        time.sleep(IR_INTERVALO_S)

def ir_detecta():
    with _ir_lock:
        return _ir_detecta

def _iniciar_ir():
    if not GPIO_DISPONIBLE:
        print("[IR] RPi.GPIO no disponible — simulación")
        return
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(IR_PIN, GPIO.IN)
    threading.Thread(target=_hilo_ir, daemon=True).start()
    print(f"[IR] Sensor infrarrojo en GPIO{IR_PIN}")

# ══════════════════════════════════════════════
#  ULTRASONIDO HC-SR04
#  TRIG → GPIO23 / ECHO → GPIO24
# ══════════════════════════════════════════════
ULTRA_TRIG = 23
ULTRA_ECHO = 24

_ultra_dist = 999
_ultra_lock = threading.Lock()

def _medir_ultra():
    if not GPIO_DISPONIBLE:
        return 999
    GPIO.output(ULTRA_TRIG, False)
    time.sleep(0.000002)
    GPIO.output(ULTRA_TRIG, True)
    time.sleep(0.00001)
    GPIO.output(ULTRA_TRIG, False)
    t0 = time.time()
    while GPIO.input(ULTRA_ECHO) == 0:
        if time.time() - t0 > 0.1:
            return 999
    inicio = time.time()
    while GPIO.input(ULTRA_ECHO) == 1:
        if time.time() - inicio > 0.1:
            return 999
    return int((time.time() - inicio) * 17150)

def _hilo_ultra():
    global _ultra_dist
    while True:
        dist = _medir_ultra()
        with _ultra_lock:
            _ultra_dist = dist
        time.sleep(0.1)

def ultra_dist():
    with _ultra_lock:
        return _ultra_dist

def _iniciar_ultra():
    if not GPIO_DISPONIBLE:
        print("[ULTRA] RPi.GPIO no disponible — simulación")
        return
    GPIO.setup(ULTRA_TRIG, GPIO.OUT)
    GPIO.setup(ULTRA_ECHO, GPIO.IN)
    GPIO.output(ULTRA_TRIG, False)
    time.sleep(0.1)
    threading.Thread(target=_hilo_ultra, daemon=True).start()
    print(f"[ULTRA] HC-SR04 en GPIO{ULTRA_TRIG}/GPIO{ULTRA_ECHO}")

# ══════════════════════════════════════════════
#  PARÁMETROS PID DIRECCIÓN
# ══════════════════════════════════════════════
KP_DIR           = 0.08
KP_DIR_SOLO_ROJA = 0.15
OFFSET_CURVA     = 50
VEL_BASE         = 50
VEL_MIN          = 25
VEL_MAX          = 75
INTERVALO_S      = 0.1

# ══════════════════════════════════════════════
#  PARÁMETROS DE GIROS
# ══════════════════════════════════════════════
GIROS = {
    "derecha_interseccion": {
        "delay_ms":    10900,
        "delay_mm":    1200,
        "recto_ms":    0,
        "vel":         60,
        "duracion_ms": 1200,
    },
    "derecha_salida": {
        "delay_ms":    100,
        "delay_mm":    None,
        "recto_ms":    13000,
        "vel":         60,
        "duracion_ms": 1000,
    },
}

# ══════════════════════════════════════════════
#  PARÁMETROS DE CARGA
#  _MM tiene prioridad sobre _MS si no es None
# ══════════════════════════════════════════════
DELAY_ANTES_IR_CARGA_MS = 2000
DELAY_ANTES_IR_CARGA_MM = 200
DELAY_POST_IR_CARGA_MS  = 200
DELAY_POST_IR_CARGA_MM  = 35

# ══════════════════════════════════════════════
#  PARÁMETROS DE DESCARGA
#  _MM tiene prioridad sobre _MS si no es None
# ══════════════════════════════════════════════
DELAY_ANTES_IR_DESCARGA_1_MS = 17000
DELAY_ANTES_IR_DESCARGA_1_MM = 1700
DELAY_POST_IR_DESCARGA_1_MS  = 750
DELAY_POST_IR_DESCARGA_1_MM  = 75

DELAY_ANTES_IR_DESCARGA_2_MS = 16000
DELAY_ANTES_IR_DESCARGA_2_MM = 1600
DELAY_POST_IR_DESCARGA_2_MS  = 800
DELAY_POST_IR_DESCARGA_2_MM  = 80

DELAY_ANTES_IR_DESCARGA_3_MS = 17500
DELAY_ANTES_IR_DESCARGA_3_MM = 1750
DELAY_POST_IR_DESCARGA_3_MS  = 800
DELAY_POST_IR_DESCARGA_3_MM  = 80

# ══════════════════════════════════════════════
#  PARÁMETROS ULTRASONIDO
# ══════════════════════════════════════════════
DIST_CARGA_CM          = 6
DELAY_POST_CARGA_MS    = 6000
DELAY_POST_DESCARGA_MS = 6000

# ══════════════════════════════════════════════
#  OFFSET DE LÍNEA POR ESTACIÓN
#  positivo = acercarse a la roja
#  negativo = acercarse a la amarilla
#  None = usar OFFSET_CENTRO_DEFAULT
# ══════════════════════════════════════════════
OFFSET_LINEA_CARGA  = 120
OFFSET_LINEA_DESC_1 = None
OFFSET_LINEA_DESC_2 = 40
OFFSET_LINEA_DESC_3 = None

# ══════════════════════════════════════════════
#  TIEMPO RECTO SIN LÍNEAS / FRAMES SIN TAG
# ══════════════════════════════════════════════
TIEMPO_RECTO_SIN_LINEA_S = 2.0
FRAMES_SIN_TAG           = 30

# ══════════════════════════════════════════════
#  SECUENCIAS DE MISIÓN
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
def _set_offset_linea(valor):
    if valor is not None:
        camara_detector.OFFSET_CENTRO = valor

def _restaurar_offset_linea():
    camara_detector.OFFSET_CENTRO = camara_detector.OFFSET_CENTRO_DEFAULT

def _navegar_normal():
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
    t0 = time.time()
    while time.time() - t0 < segundos:
        with _autonomo_lock:
            if not _autonomo:
                return
        _navegar_normal()
        time.sleep(0.05)


def _navegar_mm(mm):
    """Navega una distancia en mm usando encoders."""
    M1_MM = (2 * 3.14159 * 30.0) / 3816
    M2_MM = (2 * 3.14159 * 30.0) / 2689
    obj1  = mm / M1_MM
    obj2  = mm / M2_MM
    c1_0  = motor_control.enc["c1"]
    c2_0  = motor_control.enc["c2"]
    while True:
        with _autonomo_lock:
            if not _autonomo:
                return
        d1 = abs(motor_control.enc["c1"] - c1_0)
        d2 = abs(motor_control.enc["c2"] - c2_0)
        if (d1 / obj1 + d2 / obj2) / 2 >= 1.0:
            break
        _navegar_normal()
        time.sleep(0.02)


def _navegar_ms_o_mm(ms, mm):
    """Navega por mm si mm no es None, sino por ms."""
    if mm is not None:
        _navegar_mm(mm)
    else:
        _navegar_pid(ms / 1000.0)


def _navegar_solo_amarilla(segundos):
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


def _esperar_ir():
    """Navega hasta que el sensor IR detecta la estación."""
    while True:
        with _autonomo_lock:
            if not _autonomo:
                return False
        if ir_detecta():
            return True
        resultado = _navegar_normal()
        if resultado is None:
            motor_control.set_velocidades(VEL_BASE, VEL_BASE, "f")
        time.sleep(IR_INTERVALO_S)


def _sleep_con_obstaculos(segundos):
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

    if cfg.get("delay_mm") is not None:
        print(f"{C.INFO}[GIRO] Navegando {cfg['delay_mm']}mm con encoders{C.RESET}")
        _navegar_mm(cfg["delay_mm"])
    elif cfg["delay_ms"] > 0:
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
    antes_ms = [None, DELAY_ANTES_IR_DESCARGA_1_MS, DELAY_ANTES_IR_DESCARGA_2_MS, DELAY_ANTES_IR_DESCARGA_3_MS][numero]
    antes_mm = [None, DELAY_ANTES_IR_DESCARGA_1_MM, DELAY_ANTES_IR_DESCARGA_2_MM, DELAY_ANTES_IR_DESCARGA_3_MM][numero]
    post_ms  = [None, DELAY_POST_IR_DESCARGA_1_MS,  DELAY_POST_IR_DESCARGA_2_MS,  DELAY_POST_IR_DESCARGA_3_MS][numero]
    post_mm  = [None, DELAY_POST_IR_DESCARGA_1_MM,  DELAY_POST_IR_DESCARGA_2_MM,  DELAY_POST_IR_DESCARGA_3_MM][numero]
    offset   = [None, OFFSET_LINEA_DESC_1, OFFSET_LINEA_DESC_2, OFFSET_LINEA_DESC_3][numero]
    _set_offset_linea(offset)

    print(f"{C.WARN}[DESCARGA {numero}] Esperando sensor IR...{C.RESET}")
    if not _esperar_ir():
        _restaurar_offset_linea()
        return

    unidad = f"{post_mm}mm" if post_mm is not None else f"{post_ms}ms"
    print(f"{C.WARN}[DESCARGA {numero}] IR detectado — navegando {unidad}{C.RESET}")
    _navegar_ms_o_mm(post_ms, post_mm)

    print(f"{C.WARN}[DESCARGA {numero}] Esperando retirar carga — ultrasonido > {DIST_CARGA_CM}cm{C.RESET}")
    motor_control.enviar("s 0")
    LECTURAS_SIN_CARGA = 5
    contador = 0
    while contador < LECTURAS_SIN_CARGA:
        with _autonomo_lock:
            if not _autonomo:
                _restaurar_offset_linea()
                return
        if ultra_dist() > DIST_CARGA_CM:
            contador += 1
        else:
            contador = 0
        time.sleep(0.05)
    print(f"{C.WARN}[DESCARGA {numero}] Carga retirada — esperando {DELAY_POST_DESCARGA_MS}ms{C.RESET}")
    _sleep_con_obstaculos(DELAY_POST_DESCARGA_MS / 1000.0)
    _restaurar_offset_linea()
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

        # FASE 1: navegar hasta tag 1
        print(f"{C.INFO}[AUTO] Navegando a estación de carga...{C.RESET}")
        if not _esperar_tag(1):
            continue
        _set_offset_linea(OFFSET_LINEA_CARGA)

        # FASE 2: pasar tag 1 y esperar IR
        print(f"{C.WARN}[CARGA] Esperando pasar tag 1...{C.RESET}")
        while camara_detector.estado["ultimo_tag"] == 1:
            with _autonomo_lock:
                if not _autonomo:
                    break
            _navegar_normal()
            time.sleep(0.02)
        frames_ok = 0
        while frames_ok < FRAMES_SIN_TAG:
            with _autonomo_lock:
                if not _autonomo:
                    break
            if camara_detector.estado["ultimo_tag"] != 1:
                frames_ok += 1
            else:
                frames_ok = 0
            _navegar_normal()
            time.sleep(0.02)

        unidad_antes = f"{DELAY_ANTES_IR_CARGA_MM}mm" if DELAY_ANTES_IR_CARGA_MM is not None else f"{DELAY_ANTES_IR_CARGA_MS}ms"
        print(f"{C.WARN}[CARGA] Navegando {unidad_antes} antes de IR...{C.RESET}")
        _navegar_ms_o_mm(DELAY_ANTES_IR_CARGA_MS, DELAY_ANTES_IR_CARGA_MM)

        print(f"{C.WARN}[CARGA] Esperando sensor IR...{C.RESET}")
        if not _esperar_ir():
            _restaurar_offset_linea()
            continue

        unidad_post = f"{DELAY_POST_IR_CARGA_MM}mm" if DELAY_POST_IR_CARGA_MM is not None else f"{DELAY_POST_IR_CARGA_MS}ms"
        print(f"{C.WARN}[CARGA] IR detectado — navegando {unidad_post}{C.RESET}")
        _navegar_ms_o_mm(DELAY_POST_IR_CARGA_MS, DELAY_POST_IR_CARGA_MM)

        print(f"{C.WARN}[CARGA] Esperando carga — ultrasonido < {DIST_CARGA_CM}cm{C.RESET}")
        motor_control.enviar("s 0")
        while ultra_dist() > DIST_CARGA_CM:
            with _autonomo_lock:
                if not _autonomo:
                    break
            time.sleep(0.05)
        _restaurar_offset_linea()
        print(f"{C.WARN}[CARGA] Carga detectada — esperando {DELAY_POST_CARGA_MS}ms{C.RESET}")
        _sleep_con_obstaculos(DELAY_POST_CARGA_MS / 1000.0)
        print(f"{C.OK}[CARGA] Carga lista.{C.RESET}")

        # FASE 3: navegar hasta tag de misión
        print(f"{C.INFO}[AUTO] Esperando tag de misión...{C.RESET}")
        secuencia  = None
        tag_mision = None
        tag_ant    = None
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

        # FASE 4: ejecutar secuencia paso a paso
        for i, paso in enumerate(secuencia):
            tag_esperado = paso["espera_tag"]
            accion       = paso.get("accion")
            print(f"{C.INFO}[MISIÓN] Paso {i+1}: {paso['descripcion']}{C.RESET}")

            if not _esperar_tag(tag_esperado):
                break

            # Para descargas: navegar delay configurable antes de buscar IR
            if accion and accion.startswith("descarga"):
                idx = int(accion[-1])
                antes_ms = [None, DELAY_ANTES_IR_DESCARGA_1_MS, DELAY_ANTES_IR_DESCARGA_2_MS, DELAY_ANTES_IR_DESCARGA_3_MS][idx]
                antes_mm = [None, DELAY_ANTES_IR_DESCARGA_1_MM, DELAY_ANTES_IR_DESCARGA_2_MM, DELAY_ANTES_IR_DESCARGA_3_MM][idx]
                unidad = f"{antes_mm}mm" if antes_mm is not None else f"{antes_ms}ms"
                print(f"{C.INFO}[MISIÓN] Tag {tag_esperado} visto — navegando {unidad} antes de IR{C.RESET}")
                _navegar_ms_o_mm(antes_ms, antes_mm)
                print(f"{C.INFO}[MISIÓN] Listo para IR{C.RESET}")

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
║  ir     → ver estado sensor infrarrojo       ║
║  ultra  → ver distancia ultrasonido          ║
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

        elif cmd == "ir":
            estado_ir = "DETECTANDO" if ir_detecta() else "libre"
            print(f"  Sensor IR: {estado_ir}")

        elif cmd == "ultra":
            dist  = ultra_dist()
            carga = dist <= DIST_CARGA_CM
            print(f"  Distancia: {dist}cm — {'CARGA DETECTADA' if carga else 'sin carga'} (umbral: {DIST_CARGA_CM}cm)")

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

    if GPIO_DISPONIBLE:
        GPIO.setmode(GPIO.BCM)
    _iniciar_ir()
    _iniciar_ultra()

    if not motor_control.iniciar(port=args.motor_port, baud=args.motor_baud):
        print(f"{C.ERR}No se pudo conectar a la ESP32. Saliendo.{C.RESET}")
        sys.exit(1)

    stop_ev = threading.Event()
    threading.Thread(target=_hilo_autonomo, args=(stop_ev,), daemon=True).start()

    try:
        _consola(stop_ev)
    finally:
        motor_control.detener()
        if GPIO_DISPONIBLE:
            GPIO.cleanup()
        print(f"{C.INFO}Hasta luego.{C.RESET}")


if __name__ == "__main__":
    main()
