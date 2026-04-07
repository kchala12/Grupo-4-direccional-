#!/usr/bin/env python3
"""
Control Robot Diferencial JGY370 — Raspberry Pi
Envía comandos a la ESP32 e interpreta pulsos de encoders
Instalar: pip install pyserial
Uso: python3 motor_control_usb.py [--port /dev/ttyUSB0] [--mode cmd|wasd]
"""

import serial, serial.tools.list_ports
import threading, time, argparse, sys, tty, termios

# ═══════════════════════════════════════════════════════════
#  CALIBRACION DE ENCODERS
#  Si la distancia no coincide con la real, ajusta el radio
#  de la rueda correspondiente hasta que coincida.
#
#  Procedimiento:
#    1. Ejecuta: reset  luego  adelante 500 mm
#    2. Mide la distancia real con una regla
#    3. nuevo_radio = RADIO_ACTUAL * (distancia_real / 500)
# ═══════════════════════════════════════════════════════════

M1_PPR         = 11      # Pulsos por revolución del motor (antes de reductora)
M1_GEAR        = 30      # Relación de reducción
M1_WHEEL_MM    = 16.0    # Radio rueda izquierda en mm  ← AJUSTAR AQUÍ

M2_PPR         = 11
M2_GEAR        = 30
M2_WHEEL_MM    = 16.0    # Radio rueda derecha en mm   ← AJUSTAR AQUÍ

# ═══════════════════════════════════════════════════════════
#  FIN CALIBRACION
# ═══════════════════════════════════════════════════════════

import math
M1_MM_PER_PULSE = (2 * math.pi * M1_WHEEL_MM) / (M1_PPR * M1_GEAR * 2)
M2_MM_PER_PULSE = (2 * math.pi * M2_WHEEL_MM) / (M2_PPR * M2_GEAR * 2)

# ── Colores terminal ──────────────────────────────────────
class C:
    OK="\033[92m"; ERR="\033[91m"; INFO="\033[94m"
    WARN="\033[93m"; RESET="\033[0m"; BOLD="\033[1m"
    DIM="\033[2m"; CLEAR="\033[2J\033[H"

# ── Estado compartido de encoders ────────────────────────
enc = {"c1": 0, "c2": 0}   # pulsos acumulados

# ── Hilo lector: interpreta líneas "E c1 c2" de la ESP32 ─
def reader(ser, stop_ev):
    while not stop_ev.is_set():
        try:
            if ser.in_waiting:
                line = ser.readline().decode("utf-8", errors="replace").strip()
                if line.startswith("E "):
                    parts = line.split()
                    if len(parts) == 3:
                        enc["c1"] = int(parts[1])
                        enc["c2"] = int(parts[2])
        except Exception:
            pass
        time.sleep(0.01)

# ── Enviar comando a la ESP32 ─────────────────────────────
def send(ser, cmd):
    ser.write((cmd + "\n").encode())

# ── Detectar puerto ESP32 ─────────────────────────────────
def find_port():
    chips = ["CP210","CH340","CH341","FTDI","FT232","USB Serial","ESP32"]
    for p in serial.tools.list_ports.comports():
        d = (p.description or "").upper()
        m = (p.manufacturer or "").upper()
        if any(c in d or c in m for c in chips):
            return p.device
    pts = serial.tools.list_ports.comports()
    return pts[0].device if pts else None

# ─────────────────────────────────────────────────────────
#  MODO WASD — control en tiempo real
# ─────────────────────────────────────────────────────────
def get_key():
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

KEYS = {'w':'adelante','W':'adelante','s':'atras','S':'atras',
        'a':'izquierda','A':'izquierda','d':'derecha','D':'derecha',' ':'stop'}
STATE_ICON = {"adelante":"▲ avanzando","atras":"▼ retrocediendo",
              "derecha":"▶ girando der","izquierda":"◀ girando izq","stop":"■ parado"}

def mode_wasd(ser, stop_ev):
    speed = 50; state = "stop"; last_sent = ""; last_key = ""

    def redraw():
        d1 = enc["c1"] * M1_MM_PER_PULSE
        d2 = enc["c2"] * M2_MM_PER_PULSE
        print(C.CLEAR, end="")
        print(f"{C.BOLD}╔══════════════════════════════════════╗")
        print(f"║   WASD  |  Vel: {speed:3d}%                 ║")
        print(f"╚══════════════════════════════════════╝{C.RESET}")
        print(f"  W/↑ Adelante  A/← Izq  S/↓ Atrás  D/→ Der")
        print(f"  Esp=Stop  +/-=Vel  0-9=Vel%10  M=Comandos  Q=Salir")
        print(f"\n  {C.BOLD}Estado:{C.RESET} {C.OK}{STATE_ICON.get(state,state)}{C.RESET}   tecla:{repr(last_key)}")
        print(f"\n  {C.BOLD}Encoders:{C.RESET}")
        print(f"    M1 (izq): {enc['c1']:+7d} pulsos  |  {d1:+8.1f} mm")
        print(f"    M2 (der): {enc['c2']:+7d} pulsos  |  {d2:+8.1f} mm")
        sys.stdout.flush()

    redraw()
    while not stop_ev.is_set():
        k = get_key(); last_key = k
        if k in ('q','Q','\x03'):
            send(ser, "stop 0"); return "exit"
        if k in ('m','M'):
            send(ser, "stop 0"); return "cmd"
        if k in KEYS:
            cmd = KEYS[k]
            if cmd != last_sent:
                send(ser, f"{cmd} {speed}")
                last_sent = cmd
            state = cmd
        elif k in ('+','='):
            speed = min(100, speed+10); send(ser, f"{state} {speed}" if state!="stop" else "stop 0")
        elif k in ('-','_'):
            speed = max(0,  speed-10); send(ser, f"{state} {speed}" if state!="stop" else "stop 0")
        elif k.isdigit():
            speed = int(k)*10;         send(ser, f"{state} {speed}" if state!="stop" else "stop 0")
        redraw()
    return "exit"

# ─────────────────────────────────────────────────────────
#  MODO COMANDOS
# ─────────────────────────────────────────────────────────
MOVE = ("adelante","atras","derecha","izquierda")

def validate(cmd):
    """Retorna (ok, error, cmd_esp) donde cmd_esp es lo que se envía a la ESP."""
    p = cmd.strip().lower().split()
    if not p: return False, "Vacío.", ""

    # stop
    if p[0] == "stop": return True, "", "stop 0"

    # reset encoders
    if p[0] == "reset": return True, "", "reset 0"

    # velocidad <N>  — solo cambia la velocidad global en Python, no envía nada
    if p[0] == "velocidad" and len(p) == 2:
        try: int(p[1])
        except: return False, "Número inválido.", ""
        return True, "", ""   # manejado aparte

    # movimiento
    if p[0] in MOVE:
        if len(p) == 1:   return True, "", f"{p[0]} __VEL__"      # indefinido
        if len(p) == 3:
            try: v = int(p[1])
            except: return False, f"'{p[1]}' no es número.", ""
            if v <= 0: return False, "El valor debe ser positivo.", ""
            if p[2] not in ("mm","ms"): return False, "Unidad debe ser 'mm' o 'ms'.", ""
            return True, "", f"{p[0]} __VEL__ {v} {p[2]}"
        if len(p) == 2: return False, f"Falta unidad. Ej: {p[0]} 500 mm", ""
        return False, f"Uso: {p[0]} [N mm|ms]", ""

    return False, f"'{p[0]}' no reconocido.", ""

def mode_cmd(ser, stop_ev):
    speed = 50
    # Hilo para mostrar encoders mientras se espera input
    enc_stop = threading.Event()

    def enc_printer():
        last = (-1,-1)
        while not enc_stop.is_set():
            cur = (enc["c1"], enc["c2"])
            if cur != last:
                d1 = enc["c1"] * M1_MM_PER_PULSE
                d2 = enc["c2"] * M2_MM_PER_PULSE
                # Sobreescribir línea de encoders sin borrar toda la pantalla
                print(f"\r  {C.DIM}M1:{enc['c1']:+7d}p {d1:+7.1f}mm  "
                      f"M2:{enc['c2']:+7d}p {d2:+7.1f}mm{C.RESET}    ", end="", flush=True)
                last = cur
            time.sleep(0.15)

    print(C.CLEAR, end="")
    print(f"{C.BOLD}╔══════════════════════════════════════╗")
    print(f"║   MODO COMANDOS                      ║")
    print(f"╚══════════════════════════════════════╝{C.RESET}")
    print(f"""
  {C.BOLD}Movimiento:{C.RESET}
    adelante|atras|derecha|izquierda [N mm|ms]
    sin argumento = indefinido

  {C.BOLD}Config:{C.RESET}
    velocidad <0-100>   fijar velocidad global ({speed}% actual)
    reset               poner encoders a cero

  {C.BOLD}Ejemplos:{C.RESET}
    {C.OK}velocidad 70{C.RESET}
    {C.OK}adelante 500 mm{C.RESET}   ← avanza 500mm exactos (encoder)
    {C.OK}derecha 2000 ms{C.RESET}   ← gira 2 segundos
    {C.OK}atras{C.RESET}             ← indefinido

  {C.BOLD}wasd{C.RESET} = cambiar modo  |  {C.BOLD}exit{C.RESET} = salir
""")

    et = threading.Thread(target=enc_printer, daemon=True)
    et.start()

    try:
        while not stop_ev.is_set():
            # Imprimir prompt debajo de la línea de encoders
            print(f"\n{C.BOLD}cmd>{C.RESET} ", end="", flush=True)
            try:
                cmd = input().strip()
            except EOFError:
                enc_stop.set(); return "exit"

            if not cmd: continue
            low = cmd.lower()

            if low in ("exit","quit"): enc_stop.set(); return "exit"
            if low == "wasd":         enc_stop.set(); send(ser,"stop 0"); return "wasd"

            ok, err, esp_cmd = validate(low)
            if not ok:
                print(f"  {C.ERR}[ERROR]{C.RESET} {err}"); continue

            # Manejar velocidad local
            p = low.split()
            if p[0] == "velocidad":
                speed = max(0, min(100, int(p[1])))
                print(f"  {C.OK}[OK]{C.RESET} Velocidad: {speed}%"); continue

            if not esp_cmd: continue

            # Sustituir __VEL__ con velocidad actual
            esp_cmd = esp_cmd.replace("__VEL__", str(speed))

            # Movimiento por distancia: la Raspberry gestiona el stop
            parts = esp_cmd.split()
            if len(parts) == 4 and parts[3] == "mm":
                dist_mm = int(parts[2])
                direction = parts[0]
                send(ser, f"{direction} {speed}")
                print(f"  {C.OK}[OK]{C.RESET} {direction} {speed}% → {dist_mm}mm (esperando encoder...)")
                # Guardar pulsos iniciales y esperar
                start1, start2 = enc["c1"], enc["c2"]
                target1 = dist_mm / M1_MM_PER_PULSE
                target2 = dist_mm / M2_MM_PER_PULSE
                while True:
                    d1 = abs(enc["c1"] - start1)
                    d2 = abs(enc["c2"] - start2)
                    if d1 >= target1 and d2 >= target2:
                        send(ser, "stop 0")
                        print(f"\n  {C.OK}[AUTO]{C.RESET} Distancia alcanzada.")
                        break
                    time.sleep(0.02)

            elif len(parts) == 4 and parts[3] == "ms":
                ms = int(parts[2])
                send(ser, f"{parts[0]} {speed}")
                print(f"  {C.OK}[OK]{C.RESET} {parts[0]} {speed}% → {ms}ms")
                time.sleep(ms / 1000.0)
                send(ser, "stop 0")
                print(f"  {C.OK}[AUTO]{C.RESET} Tiempo cumplido.")

            else:
                send(ser, esp_cmd)
                print(f"  {C.OK}[OK]{C.RESET} {esp_cmd}")

    finally:
        enc_stop.set()

    return "exit"

# ─────────────────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--mode", choices=["wasd","cmd"], default="wasd")
    args = ap.parse_args()

    port = args.port or find_port()
    if not port:
        print(f"{C.ERR}No se detectó ESP32. Usa --port /dev/ttyUSB0{C.RESET}"); sys.exit(1)

    try:
        ser = serial.Serial(port=port, baudrate=args.baud, timeout=1)
        print(f"{C.OK}✔ Conectado: {port}{C.RESET}")
    except serial.SerialException as e:
        print(f"{C.ERR}✘ {e}{C.RESET}"); sys.exit(1)

    time.sleep(2); ser.reset_input_buffer()

    stop_ev = threading.Event()
    threading.Thread(target=reader, args=(ser, stop_ev), daemon=True).start()

    mode = args.mode
    try:
        while True:
            result = mode_wasd(ser, stop_ev) if mode=="wasd" else mode_cmd(ser, stop_ev)
            if result == "exit": break
            mode = result
    except Exception as e:
        print(f"\n{C.ERR}Error: {e}{C.RESET}")
    finally:
        stop_ev.set()
        try: send(ser, "stop 0"); time.sleep(0.1)
        except: pass
        ser.close()
        print(f"\n{C.INFO}Desconectado.{C.RESET}")

if __name__ == "__main__":
    main()
