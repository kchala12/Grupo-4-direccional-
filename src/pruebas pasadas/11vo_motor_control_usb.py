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
# CONFIGURACION PID
# El PID iguala la velocidad de ambas ruedas comparando
# sus pulsos por segundo. Si una rueda va más rápido que
# la otra, se reduce su PWM; si va más lenta, se aumenta.
#
# Kp: corrección proporcional al error actual.
#     Subir si el robot sigue yéndose chueco.
#     Bajar si la corrección oscila o es brusca.
#
# Ki: corrección acumulada en el tiempo.
#     Subir si hay un error constante que Kp no elimina.
#     Bajar si el robot oscila de un lado a otro.
#
# Kd: corrección basada en qué tan rápido cambia el error.
#     Subir para frenar correcciones bruscas.
#     Bajar si la respuesta se siente lenta.
# =========================================================

PID_KP = 0.8    # Ganancia proporcional ← ajustar
PID_KI = 0.1    # Ganancia integral     ← ajustar
PID_KD = 0.05   # Ganancia derivativa   ← ajustar

PID_INTERVALO_S  = 0.1    # Cada cuántos segundos se recalcula el PID
PID_CORRECCION_MAX = 20   # Máxima corrección de PWM permitida (0-100)

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

# Estado del PID compartido entre hilos.
# 'activo' indica si el hilo PID debe estar corriendo.
# 'vel_base' es la velocidad pedida por el usuario (0-100).
pid_estado = {
    "activo":   False,
    "vel_base": 50,
    "vel_m1":   50,    # velocidad actual enviada a M1 (con corrección)
    "vel_m2":   50,    # velocidad actual enviada a M2 (con corrección)
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
# HILO PID
# Corre mientras el robot está en movimiento rectilíneo
# (adelante o atras). Compara la velocidad real de cada
# rueda en pulsos/segundo y ajusta el PWM para igualarlas.
#
# No se activa en giros, porque en giros las ruedas deben
# girar a velocidades distintas a propósito.
# ---------------------------------------------------------
def hilo_pid(ser, stop_ev_pid):
    integral  = 0.0
    error_ant = 0.0
    pulsos_ant1 = enc["c1"]
    pulsos_ant2 = enc["c2"]

    while not stop_ev_pid.is_set():
        time.sleep(PID_INTERVALO_S)

        if not pid_estado["activo"]:
            # Resetear el estado del PID cuando no está activo
            # para que no acumule error entre movimientos
            integral  = 0.0
            error_ant = 0.0
            pulsos_ant1 = enc["c1"]
            pulsos_ant2 = enc["c2"]
            continue

        # Calcular velocidad real de cada rueda en pulsos/segundo
        p1_ahora = enc["c1"]
        p2_ahora = enc["c2"]
        vel1 = abs(p1_ahora - pulsos_ant1) / PID_INTERVALO_S
        vel2 = abs(p2_ahora - pulsos_ant2) / PID_INTERVALO_S
        pulsos_ant1 = p1_ahora
        pulsos_ant2 = p2_ahora

        # Las velocidades reales no son comparables directamente porque
        # cada encoder tiene diferente pulsos_por_vuelta. Se normalizan
        # dividiéndolas entre su propio pulsos_por_vuelta para obtener
        # vueltas/segundo, que sí es comparable entre las dos ruedas.
        vueltas1 = vel1 / M1_PULSOS_POR_VUELTA
        vueltas2 = vel2 / M2_PULSOS_POR_VUELTA

        # Error: diferencia de velocidad entre ruedas en vueltas/segundo.
        # Positivo = M1 va más rápido que M2.
        # Negativo = M2 va más rápido que M1.
        error = vueltas1 - vueltas2

        # Calcular los tres términos del PID
        integral  += error * PID_INTERVALO_S
        derivada   = (error - error_ant) / PID_INTERVALO_S
        error_ant  = error

        correccion = (PID_KP * error) + (PID_KI * integral) + (PID_KD * derivada)

        # Limitar la corrección para no hacer cambios bruscos
        correccion = max(-PID_CORRECCION_MAX, min(PID_CORRECCION_MAX, correccion))

        base = pid_estado["vel_base"]

        # Aplicar la corrección de forma opuesta a cada motor:
        # si M1 va más rápido (error positivo), se frena M1 y se acelera M2
        nueva_m1 = int(max(0, min(100, base - correccion)))
        nueva_m2 = int(max(0, min(100, base + correccion)))

        pid_estado["vel_m1"] = nueva_m1
        pid_estado["vel_m2"] = nueva_m2

        # Enviar las velocidades corregidas manteniendo la dirección actual
        dir_actual = pid_estado.get("dir", "adelante")
        enviar(ser, f"pid {nueva_m1} {nueva_m2} {dir_actual}")

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
                pid_estado["activo"]   = True
                pid_estado["vel_base"] = velocidad
                pid_estado["dir"]      = direccion
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
                pid_estado["activo"]   = True
                pid_estado["vel_base"] = velocidad
                pid_estado["dir"]      = direccion
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
                pid_estado["activo"]   = True
                pid_estado["vel_base"] = velocidad
                pid_estado["dir"]      = direccion
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
    threading.Thread(target=hilo_pid, args=(ser, stop_ev_pid),
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
