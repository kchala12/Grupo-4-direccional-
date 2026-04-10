#!/usr/bin/env python3
"""
Control Robot Diferencial JGY370 — Raspberry Pi
Instalar: pip install pyserial
Uso: python3 motor_control_usb.py [--port /dev/ttyUSB0] [--mode cmd|wasd]
"""

import serial, serial.tools.list_ports
import threading, time, argparse, sys, tty, termios, math

# =========================================================
# CALIBRACION
# Pulsos por vuelta de rueda medidos experimentalmente.
# Son valores post-reducción, no se necesita saber el PPR
# del motor ni la relación de reducción por separado.
# =========================================================

M1_PULSOS_POR_VUELTA = 297    # Rueda izquierda ← ajustar si es necesario
M2_PULSOS_POR_VUELTA = 2682   # Rueda derecha   ← ajustar si es necesario
RADIO_MM             = 30.0   # Radio de rueda en mm (medida física, no cambiar)

# =========================================================
# CONFIGURACION CONTROLADOR POR MOTOR
#
# Cada motor tiene su propio controlador que compara cuántos
# pulsos lleva con cuántos se esperaban según el tiempo
# transcurrido, y ajusta su PWM de forma independiente.
#
# Kp: qué tan agresiva es la corrección al error.
#     Subir si el motor se queda atrás o adelanta mucho.
#     Bajar si la velocidad oscila o es inestable.
#
# CORRECCION_MAX: límite de ajuste de PWM por ciclo.
#     Evita cambios bruscos. Rango útil: 10-30.
# =========================================================

CTRL_KP          = 1.0   # Ganancia proporcional ← ajustar
CTRL_INTERVALO_S = 0.1   # Cada cuántos segundos se recalcula
CTRL_CORRECCION_MAX = 20 # Máxima corrección de PWM permitida

# =========================================================
# FIN CONFIGURACION
# =========================================================

# mm que recorre la rueda por cada pulso del encoder
M1_MM_POR_PULSO = (2 * math.pi * RADIO_MM) / M1_PULSOS_POR_VUELTA
M2_MM_POR_PULSO = (2 * math.pi * RADIO_MM) / M2_PULSOS_POR_VUELTA

# Colores para la terminal
class C:
    OK="\033[92m"; ERR="\033[91m"; INFO="\033[94m"
    WARN="\033[93m"; RESET="\033[0m"; BOLD="\033[1m"
    DIM="\033[2m";  CLEAR="\033[2J\033[H"

# Pulsos acumulados recibidos de la ESP32.
# El hilo lector los actualiza; el resto del programa los lee.
enc = {"c1": 0, "c2": 0}

# Estado del controlador compartido entre hilos.
pid_estado = {
    "activo":   False,
    "vel_base": 50,
    "vel_m1":   50,
    "vel_m2":   50,
    "dir":      "",
}

# ---------------------------------------------------------
# HILO LECTOR DE ENCODERS
# Corre en paralelo. Lee las líneas "E c1 c2\n" que envía
# la ESP32 cada 100ms y actualiza el diccionario enc.
# ---------------------------------------------------------
def hilo_lector(ser, stop_ev):
    while not stop_ev.is_set():
        try:
            if ser.in_waiting:
                linea = ser.readline().decode("utf-8", errors="replace").strip()
                if linea.startswith("E "):
                    p = linea.split()
                    if len(p) == 3:
                        enc["c1"] = int(p[1])
                        enc["c2"] = int(p[2])
        except Exception:
            pass
        time.sleep(0.01)

# ---------------------------------------------------------
# HILO CONTROLADOR — controlador independiente por motor
#
# Cada motor calcula cuántos pulsos debería haber hecho
# desde que arrancó el movimiento, según su propia tasa
# inicial medida en el primer intervalo. Compara eso con
# los pulsos reales y ajusta su PWM de forma independiente.
#
# Ventaja: no importa que los motores sean distintos porque
# cada uno se compara consigo mismo, no con el otro.
# No se activa en giros (las ruedas deben ir distinto a propósito).
# ---------------------------------------------------------
def hilo_controlador(ser, stop_ev_pid):
    tiempo_inicio  = 0.0
    pulsos_inicio1 = 0
    pulsos_inicio2 = 0
    tasa_esperada1 = 0.0   # pulsos/segundo de referencia para M1
    tasa_esperada2 = 0.0   # pulsos/segundo de referencia para M2

    while not stop_ev_pid.is_set():
        time.sleep(CTRL_INTERVALO_S)

        if not pid_estado["activo"]:
            # Resetear todo al parar para que el próximo movimiento
            # empiece desde cero sin arrastrar estado anterior
            tiempo_inicio  = 0.0
            tasa_esperada1 = 0.0
            tasa_esperada2 = 0.0
            continue

        # Primer ciclo activo: registrar punto de partida
        if tiempo_inicio == 0.0:
            tiempo_inicio  = time.time()
            pulsos_inicio1 = enc["c1"]
            pulsos_inicio2 = enc["c2"]
            continue   # Esperar un intervalo para tener datos reales

        t_elapsed = time.time() - tiempo_inicio

        # Pulsos reales acumulados desde el inicio del movimiento
        reales1 = abs(enc["c1"] - pulsos_inicio1)
        reales2 = abs(enc["c2"] - pulsos_inicio2)

        # Calcular tasa de referencia con los primeros pulsos reales.
        # Mide cuántos pulsos/segundo hace cada motor a vel_base,
        # y se usa como objetivo para todos los ciclos siguientes.
        if tasa_esperada1 == 0.0 and reales1 > 0:
            tasa_esperada1 = reales1 / t_elapsed
        if tasa_esperada2 == 0.0 and reales2 > 0:
            tasa_esperada2 = reales2 / t_elapsed

        if tasa_esperada1 == 0.0 or tasa_esperada2 == 0.0:
            continue   # Aún sin referencia, esperar otro ciclo

        # Pulsos que debería haber hecho cada motor hasta ahora
        esperados1 = tasa_esperada1 * t_elapsed
        esperados2 = tasa_esperada2 * t_elapsed

        # Error individual de cada motor:
        # positivo = va más rápido de lo esperado → frenar
        # negativo = va más lento de lo esperado  → acelerar
        error1 = reales1 - esperados1
        error2 = reales2 - esperados2

        corr1 = max(-CTRL_CORRECCION_MAX, min(CTRL_CORRECCION_MAX, CTRL_KP * error1))
        corr2 = max(-CTRL_CORRECCION_MAX, min(CTRL_CORRECCION_MAX, CTRL_KP * error2))

        base     = pid_estado["vel_base"]
        nueva_m1 = int(max(0, min(100, base - corr1)))
        nueva_m2 = int(max(0, min(100, base - corr2)))

        pid_estado["vel_m1"] = nueva_m1
        pid_estado["vel_m2"] = nueva_m2

        direccion = pid_estado.get("dir", "")
        if direccion:
            enviar(ser, f"pid {nueva_m1} {nueva_m2} {direccion}")

# Enviar un comando a la ESP32
def enviar(ser, cmd):
    ser.write((cmd + "\n").encode("utf-8"))

# Detectar automáticamente el puerto de la ESP32
def buscar_puerto():
    chips = ["CP210","CH340","CH341","FTDI","FT232","USB Serial","ESP32"]
    for p in serial.tools.list_ports.comports():
        d = (p.description or "").upper()
        m = (p.manufacturer or "").upper()
        if any(c in d or c in m for c in chips):
            return p.device
    pts = serial.tools.list_ports.comports()
    return pts[0].device if pts else None

# =========================================================
# MODO WASD — control en tiempo real tecla a tecla
# =========================================================

def leer_tecla():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            c2 = sys.stdin.read(1)
            if c2 == '[':
                c3 = sys.stdin.read(1)
                return {'A':'w','B':'s','C':'d','D':'a'}.get(c3,'')
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

TECLAS = {
    'w':'adelante','W':'adelante','s':'atras','S':'atras',
    'a':'izquierda','A':'izquierda','d':'derecha','D':'derecha',' ':'stop'
}
ICONOS = {
    "adelante":"▲ avanzando","atras":"▼ retrocediendo",
    "derecha":"▶ girando der","izquierda":"◀ girando izq","stop":"■ parado"
}
# Direcciones en las que se activa el PID (movimiento recto)
DIRS_CON_PID = ("adelante", "atras")

def modo_wasd(ser, stop_ev):
    velocidad  = 50
    estado     = "stop"
    ultimo_cmd = ""

    def redibujar(ultima_tecla):
        d1 = enc["c1"] * M1_MM_POR_PULSO
        d2 = enc["c2"] * M2_MM_POR_PULSO
        pid_str = f"{C.OK}ON{C.RESET}" if pid_estado["activo"] else f"{C.DIM}OFF{C.RESET}"
        print(C.CLEAR, end="")
        print(f"{C.BOLD}╔══════════════════════════════════════╗")
        print(f"║   WASD  |  Velocidad: {velocidad:3d}%  PID:{pid_str}     ")
        print(f"╚══════════════════════════════════════╝{C.RESET}")
        print(f"  W/↑=Adelante  A/←=Izq  S/↓=Atrás  D/→=Der")
        print(f"  Espacio=Stop  +/-=Vel±10  0-9=Vel×10")
        print(f"  M=Modo comandos  Q=Salir")
        print(f"\n  Estado: {C.OK}{ICONOS.get(estado,estado)}{C.RESET}  tecla:{repr(ultima_tecla)}")
        print(f"\n  {C.BOLD}Encoders:{C.RESET}")
        print(f"    M1 (izq): {enc['c1']:+7d} pulsos  |  {d1:+8.1f} mm")
        print(f"    M2 (der): {enc['c2']:+7d} pulsos  |  {d2:+8.1f} mm")
        sys.stdout.flush()

    redibujar("")
    while not stop_ev.is_set():
        tecla = leer_tecla()

        if tecla in ('q','Q','\x03'):
            pid_estado["activo"] = False
            enviar(ser, "stop 0"); return "exit"
        if tecla in ('m','M'):
            pid_estado["activo"] = False
            enviar(ser, "stop 0"); return "cmd"

        if tecla in TECLAS:
            cmd = TECLAS[tecla]
            if cmd != ultimo_cmd:
                if cmd in DIRS_CON_PID:
                    # Activar PID para movimiento recto
                    pid_estado["activo"]   = True
                    pid_estado["vel_base"] = velocidad
                    pid_estado["dir"]      = cmd
                    # Envío inicial sin corrección; el PID ajustará en su próximo ciclo
                    enviar(ser, f"{cmd} {velocidad}")
                else:
                    # En giros desactivar PID — las ruedas deben ir a distinta velocidad
                    pid_estado["activo"] = False
                    if cmd == "stop":
                        enviar(ser, "stop 0")
                    else:
                        enviar(ser, f"{cmd} {velocidad}")
                ultimo_cmd = cmd
            estado = cmd

        elif tecla in ('+','='):
            velocidad = min(100, velocidad + 10)
            pid_estado["vel_base"] = velocidad
            if estado not in ("stop",):
                enviar(ser, f"{estado} {velocidad}")
        elif tecla in ('-','_'):
            velocidad = max(0, velocidad - 10)
            pid_estado["vel_base"] = velocidad
            if estado not in ("stop",):
                enviar(ser, f"{estado} {velocidad}")
        elif tecla.isdigit():
            velocidad = int(tecla) * 10
            pid_estado["vel_base"] = velocidad
            if estado not in ("stop",):
                enviar(ser, f"{estado} {velocidad}")

        redibujar(tecla)

    return "exit"

# =========================================================
# MODO COMANDOS
# =========================================================

DIRECCIONES = ("adelante", "atras", "derecha", "izquierda")

def validar(cmd):
    p = cmd.strip().lower().split()
    if not p: return False, "Comando vacío.", ""
    if p[0] == "stop":   return True, "", "stop 0"
    if p[0] == "reset":  return True, "", "reset 0"
    if p[0] == "velocidad":
        if len(p) != 2: return False, "Uso: velocidad <0-100>", ""
        try:
            v = int(p[1])
            if not 0 <= v <= 100: return False, "Rango: 0-100.", ""
        except ValueError: return False, f"'{p[1]}' no es número.", ""
        return True, "", "__VELOCIDAD__"
    if p[0] in DIRECCIONES:
        if len(p) == 1: return True, "", f"{p[0]} __VEL__"
        if len(p) == 3:
            try:
                v = int(p[1])
                if v <= 0: return False, "El valor debe ser positivo.", ""
            except ValueError: return False, f"'{p[1]}' no es número.", ""
            if p[2] not in ("mm","ms"): return False, "Unidad: 'mm' o 'ms'.", ""
            return True, "", f"{p[0]} __VEL__ {p[1]} {p[2]}"
        if len(p) == 2: return False, f"Falta unidad. Ej: {p[0]} 500 mm", ""
        return False, f"Uso: {p[0]} [N mm|ms]", ""
    return False, f"'{p[0]}' no reconocido.", ""

def modo_cmd(ser, stop_ev):
    velocidad = 50

    print(C.CLEAR, end="")
    print(f"{C.BOLD}╔══════════════════════════════════════╗")
    print(f"║   MODO COMANDOS                      ║")
    print(f"╚══════════════════════════════════════╝{C.RESET}")
    print(f"""
  {C.BOLD}Movimiento:{C.RESET}
    adelante|atras|derecha|izquierda           indefinido
    adelante|atras|derecha|izquierda <N> mm    N milímetros
    adelante|atras|derecha|izquierda <N> ms    N milisegundos
    stop

  {C.BOLD}Configuración:{C.RESET}
    velocidad <0-100>    velocidad global (actual: {velocidad}%)
    reset                encoders a cero

  {C.BOLD}Navegación:{C.RESET}
    wasd / exit
""")

    while not stop_ev.is_set():
        try:
            cmd = input(f"{C.BOLD}cmd>{C.RESET} ").strip()
        except EOFError:
            return "exit"

        if not cmd: continue
        low = cmd.lower()

        if low in ("exit","quit"):      return "exit"
        if low == "wasd":               enviar(ser,"stop 0"); return "wasd"
        if low == "help":               return modo_cmd(ser, stop_ev)

        ok, err, esp_cmd = validar(low)
        if not ok:
            print(f"  {C.ERR}[ERROR]{C.RESET} {err}"); continue

        if esp_cmd == "__VELOCIDAD__":
            velocidad = int(low.split()[1])
            pid_estado["vel_base"] = velocidad
            print(f"  {C.OK}[OK]{C.RESET} Velocidad: {velocidad}%"); continue

        esp_cmd = esp_cmd.replace("__VEL__", str(velocidad))
        partes  = esp_cmd.split()

        if len(partes) == 2 and esp_cmd not in ("stop 0","reset 0"):
            # Movimiento indefinido
            direccion = partes[0]
            antes1, antes2 = enc["c1"], enc["c2"]
            if direccion in DIRS_CON_PID:
                pid_estado["dir"]      = direccion
                pid_estado["vel_base"] = velocidad
                pid_estado["activo"]   = True
            enviar(ser, esp_cmd)
            print(f"  {C.OK}[OK]{C.RESET} {direccion} al {velocidad}% — indefinido")
            print(f"  {C.DIM}(pulsos al arrancar — M1:{antes1:+d}  M2:{antes2:+d}){C.RESET}")

        elif len(partes) == 4 and partes[3] == "mm":
            dist_mm   = int(partes[2])
            direccion = partes[0]
            antes1, antes2 = enc["c1"], enc["c2"]
            objetivo1 = dist_mm / M1_MM_POR_PULSO
            objetivo2 = dist_mm / M2_MM_POR_PULSO
            if direccion in DIRS_CON_PID:
                pid_estado["dir"]      = direccion
                pid_estado["vel_base"] = velocidad
                pid_estado["activo"]   = True
            enviar(ser, f"{direccion} {velocidad}")
            print(f"  {C.OK}[OK]{C.RESET} {direccion} al {velocidad}% → {dist_mm} mm")
            while True:
                if abs(enc["c1"]-antes1) >= objetivo1 and abs(enc["c2"]-antes2) >= objetivo2:
                    pid_estado["activo"] = False
                    enviar(ser, "stop 0")
                    delta1 = enc["c1"] - antes1
                    delta2 = enc["c2"] - antes2
                    print(f"  {C.OK}[AUTO]{C.RESET} Distancia alcanzada.")
                    print(f"  {C.INFO}Pulsos durante el comando:{C.RESET}")
                    print(f"    M1: {delta1:+d} pulsos  |  {delta1*M1_MM_POR_PULSO:+.1f} mm")
                    print(f"    M2: {delta2:+d} pulsos  |  {delta2*M2_MM_POR_PULSO:+.1f} mm")
                    break
                time.sleep(0.02)

        elif len(partes) == 4 and partes[3] == "ms":
            ms        = int(partes[2])
            direccion = partes[0]
            antes1, antes2 = enc["c1"], enc["c2"]
            if direccion in DIRS_CON_PID:
                # Setear dirección ANTES de activar para que el hilo PID
                # la encuentre disponible desde su primer ciclo
                pid_estado["dir"]      = direccion
                pid_estado["vel_base"] = velocidad
                pid_estado["activo"]   = True
            enviar(ser, f"{direccion} {velocidad}")
            print(f"  {C.OK}[OK]{C.RESET} {direccion} al {velocidad}% → {ms} ms")
            time.sleep(ms / 1000.0)
            pid_estado["activo"] = False
            enviar(ser, "stop 0")
            delta1 = enc["c1"] - antes1
            delta2 = enc["c2"] - antes2
            print(f"  {C.OK}[AUTO]{C.RESET} Tiempo cumplido.")
            print(f"  {C.INFO}Pulsos durante el comando:{C.RESET}")
            print(f"    M1: {delta1:+d} pulsos  |  {delta1*M1_MM_POR_PULSO:+.1f} mm")
            print(f"    M2: {delta2:+d} pulsos  |  {delta2*M2_MM_POR_PULSO:+.1f} mm")

        elif esp_cmd == "stop 0":
            pid_estado["activo"] = False
            enviar(ser, esp_cmd)
            print(f"  {C.OK}[OK]{C.RESET} Stop.")

        elif esp_cmd == "reset 0":
            enviar(ser, esp_cmd)
            print(f"  {C.OK}[OK]{C.RESET} Encoders reseteados.")

    return "exit"

# =========================================================
# MAIN
# =========================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--mode", choices=["wasd","cmd"], default="wasd")
    args = ap.parse_args()

    puerto = args.port or buscar_puerto()
    if not puerto:
        print(f"{C.ERR}No se detectó ESP32. Usa --port /dev/ttyUSB0{C.RESET}")
        sys.exit(1)

    try:
        ser = serial.Serial(puerto, baudrate=args.baud, timeout=1)
        print(f"{C.OK}✔ Conectado: {puerto}{C.RESET}")
    except serial.SerialException as e:
        print(f"{C.ERR}✘ {e}{C.RESET}"); sys.exit(1)

    time.sleep(2)
    ser.reset_input_buffer()

    stop_ev     = threading.Event()
    stop_ev_pid = threading.Event()

    # Hilo lector de encoders
    threading.Thread(target=hilo_lector, args=(ser, stop_ev),
                     daemon=True).start()

    # Hilo PID — corre siempre, pero solo actúa cuando pid_estado["activo"]=True
    threading.Thread(target=hilo_controlador, args=(ser, stop_ev_pid),
                     daemon=True).start()

    modo = args.mode
    try:
        while True:
            resultado = modo_wasd(ser, stop_ev) if modo == "wasd" \
                        else modo_cmd(ser, stop_ev)
            if resultado == "exit": break
            modo = resultado
    except Exception as e:
        print(f"\n{C.ERR}Error: {e}{C.RESET}")
    finally:
        stop_ev.set()
        stop_ev_pid.set()
        try:
            pid_estado["activo"] = False
            enviar(ser, "stop 0")
            time.sleep(0.1)
        except Exception:
            pass
        ser.close()
        print(f"\n{C.INFO}Desconectado.{C.RESET}")

if __name__ == "__main__":
    main()
