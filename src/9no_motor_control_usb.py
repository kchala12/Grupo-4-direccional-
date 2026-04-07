#!/usr/bin/env python3
"""
Control Robot Diferencial JGY370 — Raspberry Pi
Instalar: pip install pyserial
Uso: python3 motor_control_usb.py [--port /dev/ttyUSB0] [--mode cmd|wasd]
"""

import serial, serial.tools.list_ports
import threading, time, argparse, sys, tty, termios, math

# =========================================================
# CALIBRACION — ajustar estos valores si la distancia
# medida no coincide con la real.
# Para calibrar: envía "adelante 500 mm", mide la distancia
# real y calcula:
#   PPR_nuevo = PPR_actual × (distancia_real / 500)
# El radio de 30mm es una medida física, no cambiarlo.
# =========================================================

M1_PULSOS_POR_VUELTA = 660   # Motor izquierdo: PPR × reducción × 2
M2_PULSOS_POR_VUELTA = 660   # Motor derecho:   PPR × reducción × 2
                              # Valor base: 11 PPR × 30 reduccion × 2 = 660
                              # Aumentar si el robot recorre MAS de lo esperado
                              # Disminuir si el robot recorre MENOS de lo esperado

RADIO_MM = 30.0              # Radio de la rueda en mm (no cambiar)

# =========================================================
# FIN CALIBRACION
# =========================================================

# mm que corresponde a cada pulso del encoder
# Fórmula: circunferencia / pulsos_por_vuelta
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

# ---------------------------------------------------------
# HILO LECTOR
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

# Lee una tecla sin necesitar Enter
def leer_tecla():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        # Detectar teclas de flecha (secuencia ESC [ A/B/C/D)
        if ch == '\x1b':
            c2 = sys.stdin.read(1)
            if c2 == '[':
                c3 = sys.stdin.read(1)
                return {'A':'w','B':'s','C':'d','D':'a'}.get(c3,'')
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

TECLAS = {
    'w':'adelante','W':'adelante',
    's':'atras',   'S':'atras',
    'a':'izquierda','A':'izquierda',
    'd':'derecha', 'D':'derecha',
    ' ':'stop'
}

ICONOS = {
    "adelante":"▲ avanzando", "atras":"▼ retrocediendo",
    "derecha":"▶ girando der", "izquierda":"◀ girando izq",
    "stop":"■ parado"
}

def modo_wasd(ser, stop_ev):
    velocidad  = 50
    estado     = "stop"
    ultimo_cmd = ""

    def redibujar(ultima_tecla):
        d1 = enc["c1"] * M1_MM_POR_PULSO
        d2 = enc["c2"] * M2_MM_POR_PULSO
        print(C.CLEAR, end="")
        print(f"{C.BOLD}╔══════════════════════════════════════╗")
        print(f"║   WASD  |  Velocidad: {velocidad:3d}%           ║")
        print(f"╚══════════════════════════════════════╝{C.RESET}")
        print(f"  W/↑=Adelante  A/←=Izq  S/↓=Atrás  D/→=Der")
        print(f"  Espacio=Stop  +/-=Vel±10  0-9=Vel×10")
        print(f"  M=Modo comandos  Q=Salir")
        print(f"\n  Estado: {C.OK}{ICONOS.get(estado,estado)}{C.RESET}  "
              f"tecla:{repr(ultima_tecla)}")
        print(f"\n  {C.BOLD}Encoders:{C.RESET}")
        print(f"    M1 (izq): {enc['c1']:+7d} pulsos  |  {d1:+8.1f} mm")
        print(f"    M2 (der): {enc['c2']:+7d} pulsos  |  {d2:+8.1f} mm")
        sys.stdout.flush()

    redibujar("")
    while not stop_ev.is_set():
        tecla = leer_tecla()

        if tecla in ('q','Q','\x03'):         # Q o Ctrl+C → salir
            enviar(ser, "stop 0"); return "exit"
        if tecla in ('m','M'):                # M → cambiar a modo comandos
            enviar(ser, "stop 0"); return "cmd"

        if tecla in TECLAS:
            cmd = TECLAS[tecla]
            # Solo enviar si el comando cambió para no saturar el serial
            if cmd != ultimo_cmd:
                enviar(ser, f"{cmd} {velocidad}")
                ultimo_cmd = cmd
            estado = cmd

        elif tecla in ('+','='):
            velocidad = min(100, velocidad + 10)
            if estado != "stop":
                enviar(ser, f"{estado} {velocidad}")
        elif tecla in ('-','_'):
            velocidad = max(0, velocidad - 10)
            if estado != "stop":
                enviar(ser, f"{estado} {velocidad}")
        elif tecla.isdigit():
            velocidad = int(tecla) * 10
            if estado != "stop":
                enviar(ser, f"{estado} {velocidad}")

        redibujar(tecla)

    return "exit"

# =========================================================
# MODO COMANDOS — control por texto con distancia y tiempo
# =========================================================

DIRECCIONES = ("adelante", "atras", "derecha", "izquierda")

def validar(cmd):
    """
    Valida el comando escrito por el usuario.
    Retorna (ok, mensaje_error, comando_para_esp).
    El comando_para_esp usa __VEL__ donde va la velocidad.
    """
    p = cmd.strip().lower().split()
    if not p:
        return False, "Comando vacío.", ""

    if p[0] == "stop":
        return True, "", "stop 0"

    if p[0] == "reset":
        return True, "", "reset 0"

    if p[0] == "velocidad":
        if len(p) != 2:
            return False, "Uso: velocidad <0-100>", ""
        try:
            v = int(p[1])
            if not 0 <= v <= 100:
                return False, "Rango: 0-100.", ""
        except ValueError:
            return False, f"'{p[1]}' no es un número.", ""
        return True, "", "__VELOCIDAD__"   # manejado aparte en el bucle

    if p[0] in DIRECCIONES:
        if len(p) == 1:                    # sin argumento = indefinido
            return True, "", f"{p[0]} __VEL__"
        if len(p) == 3:
            try:
                v = int(p[1])
                if v <= 0:
                    return False, "El valor debe ser positivo.", ""
            except ValueError:
                return False, f"'{p[1]}' no es un número.", ""
            if p[2] not in ("mm", "ms"):
                return False, "Unidad debe ser 'mm' o 'ms'.", ""
            return True, "", f"{p[0]} __VEL__ {p[1]} {p[2]}"
        if len(p) == 2:
            return False, f"Falta la unidad. Ej: {p[0]} 500 mm", ""
        return False, f"Uso: {p[0]} [N mm|ms]", ""

    return False, f"'{p[0]}' no reconocido. Escribe 'help'.", ""

def imprimir_encoders():
    """Imprime una línea con pulsos y mm de cada encoder."""
    d1 = enc["c1"] * M1_MM_POR_PULSO
    d2 = enc["c2"] * M2_MM_POR_PULSO
    print(f"  {C.DIM}Encoders — "
          f"M1: {enc['c1']:+7d} pulsos  {d1:+8.1f} mm  |  "
          f"M2: {enc['c2']:+7d} pulsos  {d2:+8.1f} mm{C.RESET}")

def modo_cmd(ser, stop_ev):
    velocidad = 50

    print(C.CLEAR, end="")
    print(f"{C.BOLD}╔══════════════════════════════════════╗")
    print(f"║   MODO COMANDOS                      ║")
    print(f"╚══════════════════════════════════════╝{C.RESET}")
    print(f"""
  {C.BOLD}Movimiento:{C.RESET}
    adelante|atras|derecha|izquierda           indefinido
    adelante|atras|derecha|izquierda <N> mm    N milímetros (encoder)
    adelante|atras|derecha|izquierda <N> ms    N milisegundos
    stop                                       parar

  {C.BOLD}Configuración:{C.RESET}
    velocidad <0-100>     fijar velocidad global (actual: {velocidad}%)
    reset                 poner encoders a cero

  {C.BOLD}Navegación:{C.RESET}
    wasd    cambiar a modo WASD
    exit    salir

  {C.BOLD}Ejemplos:{C.RESET}
    {C.OK}velocidad 70{C.RESET}
    {C.OK}adelante 500 mm{C.RESET}   ← para solo al llegar a 500mm
    {C.OK}derecha 2000 ms{C.RESET}   ← gira durante 2 segundos
""")

    while not stop_ev.is_set():
        # Mostrar estado de encoders antes de cada prompt
        imprimir_encoders()

        try:
            cmd = input(f"{C.BOLD}cmd>{C.RESET} ").strip()
        except EOFError:
            return "exit"

        if not cmd:
            continue

        low = cmd.lower()

        if low in ("exit", "quit"):
            return "exit"
        if low == "wasd":
            enviar(ser, "stop 0"); return "wasd"
        if low == "help":
            # Redibujar la ayuda
            return modo_cmd(ser, stop_ev)

        ok, err, esp_cmd = validar(low)
        if not ok:
            print(f"  {C.ERR}[ERROR]{C.RESET} {err}")
            continue

        # Cambio de velocidad local (no envía nada a la ESP)
        if esp_cmd == "__VELOCIDAD__":
            velocidad = int(low.split()[1])
            print(f"  {C.OK}[OK]{C.RESET} Velocidad: {velocidad}%")
            continue

        # Sustituir el marcador __VEL__ con la velocidad actual
        esp_cmd = esp_cmd.replace("__VEL__", str(velocidad))
        partes  = esp_cmd.split()

        if len(partes) == 2:
            # Movimiento indefinido: enviar y listo
            enviar(ser, esp_cmd)
            print(f"  {C.OK}[OK]{C.RESET} {partes[0]} al {velocidad}% — indefinido")

        elif len(partes) == 4 and partes[3] == "mm":
            # Movimiento por distancia usando encoders
            dist_mm   = int(partes[2])
            direccion = partes[0]
            # Guardar pulsos en el momento de arrancar
            inicio1, inicio2 = enc["c1"], enc["c2"]
            # Calcular cuántos pulsos equivalen a la distancia pedida
            objetivo1 = dist_mm / M1_MM_POR_PULSO
            objetivo2 = dist_mm / M2_MM_POR_PULSO
            enviar(ser, f"{direccion} {velocidad}")
            print(f"  {C.OK}[OK]{C.RESET} {direccion} al {velocidad}% → {dist_mm} mm")
            # Esperar hasta que ambas ruedas recorran la distancia pedida
            while True:
                avance1 = abs(enc["c1"] - inicio1)
                avance2 = abs(enc["c2"] - inicio2)
                if avance1 >= objetivo1 and avance2 >= objetivo2:
                    enviar(ser, "stop 0")
                    print(f"  {C.OK}[AUTO]{C.RESET} Distancia alcanzada.")
                    imprimir_encoders()
                    break
                time.sleep(0.02)

        elif len(partes) == 4 and partes[3] == "ms":
            # Movimiento por tiempo
            ms        = int(partes[2])
            direccion = partes[0]
            enviar(ser, f"{direccion} {velocidad}")
            print(f"  {C.OK}[OK]{C.RESET} {direccion} al {velocidad}% → {ms} ms")
            time.sleep(ms / 1000.0)
            enviar(ser, "stop 0")
            print(f"  {C.OK}[AUTO]{C.RESET} Tiempo cumplido.")
            imprimir_encoders()

        elif esp_cmd in ("stop 0", "reset 0"):
            enviar(ser, esp_cmd)
            if "reset" in esp_cmd:
                print(f"  {C.OK}[OK]{C.RESET} Encoders reseteados.")
            else:
                print(f"  {C.OK}[OK]{C.RESET} Stop.")

    return "exit"

# =========================================================
# MAIN
# =========================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None,
                    help="Puerto serial. Ej: /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--mode", choices=["wasd","cmd"], default="wasd",
                    help="Modo inicial (default: wasd)")
    args = ap.parse_args()

    # Detectar puerto si no se indicó uno
    puerto = args.port or buscar_puerto()
    if not puerto:
        print(f"{C.ERR}No se detectó ESP32. Usa --port /dev/ttyUSB0{C.RESET}")
        sys.exit(1)

    # Abrir conexión serial
    try:
        ser = serial.Serial(puerto, baudrate=args.baud, timeout=1)
        print(f"{C.OK}✔ Conectado: {puerto}{C.RESET}")
    except serial.SerialException as e:
        print(f"{C.ERR}✘ {e}{C.RESET}")
        sys.exit(1)

    # Esperar a que la ESP32 arranque y limpiar buffer
    time.sleep(2)
    ser.reset_input_buffer()

    # Iniciar hilo lector de encoders
    stop_ev = threading.Event()
    threading.Thread(target=hilo_lector, args=(ser, stop_ev),
                     daemon=True).start()

    modo = args.mode
    try:
        while True:
            resultado = modo_wasd(ser, stop_ev) if modo == "wasd" \
                        else modo_cmd(ser, stop_ev)
            if resultado == "exit":
                break
            modo = resultado   # "wasd" o "cmd"
    except Exception as e:
        print(f"\n{C.ERR}Error: {e}{C.RESET}")
    finally:
        stop_ev.set()
        try:
            enviar(ser, "stop 0")
            time.sleep(0.1)
        except Exception:
            pass
        ser.close()
        print(f"\n{C.INFO}Desconectado.{C.RESET}")

if __name__ == "__main__":
    main()
