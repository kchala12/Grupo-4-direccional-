#!/usr/bin/env python3
"""
main_robot.py
=============
Control interactivo del robot diferencial JGY370.

Comandos disponibles:
    f <valor> <ms|mm>   — Avanzar hacia adelante
    b <valor> <ms|mm>   — Retroceder hacia atrás
    l <valor> <ms|mm>   — Girar a la izquierda
    r <valor> <ms|mm>   — Girar a la derecha
    s                   — Detener inmediatamente
    q                   — Salir del programa

Ejemplos:
    f 100 ms     → avanza 100 milisegundos
    b 200 mm     → retrocede 200 milímetros
    l 500 ms     → gira a la izquierda 500 ms
    r 300 mm     → gira a la derecha 300 mm
    f 500 mm, b 200 ms, r 400 ms   → secuencia de comandos separados por coma

Comportamiento ante obstáculos:
    - Si la visión detecta un obstáculo DURANTE un movimiento, el robot se detiene.
    - El comando NO se descarta: el robot ESPERA a que el camino quede libre
      y luego REANUDA el movimiento restante automáticamente.
    - Si el obstáculo persiste más de TIMEOUT_OBSTACULO segundos, se cancela
      el comando actual y se pasa al siguiente.

Uso:
    python3 main_robot.py
    python3 main_robot.py --sin-vision     (sin cámara, para pruebas)
    python3 main_robot.py --puerto /dev/ttyUSB0
"""

import sys
import time
import threading
import argparse

from robot_motor_lib import RobotMotor

# ── Importar visión de forma opcional ─────────────────────────────────────────
try:
    from vision_obstaculos import VisionObstaculos
    VISION_DISPONIBLE = True
except ImportError:
    VISION_DISPONIBLE = False
    print("[Advertencia] vision_obstaculos.py no encontrado. "
          "Ejecutando sin detección de obstáculos.")


# =============================================================================
# CONFIGURACIÓN
# =============================================================================

# Segundos máximos esperando a que despeje el obstáculo antes de cancelar.
TIMEOUT_OBSTACULO = 30.0

# Velocidad PWM por defecto (0-100)
VELOCIDAD_DEFAULT = 55

# Separador de comandos en cadena (ej: "f 100 ms, b 200 mm")
SEPARADOR_COMANDOS = ","

# Mapeo de alias de dirección → método de RobotMotor
DIRECCION_METODO = {
    "f": "adelante",
    "b": "atras",
    "l": "izquierda",
    "r": "derecha",
}

AYUDA = """
╔══════════════════════════════════════════════════════╗
║           CONTROL INTERACTIVO DEL ROBOT              ║
╠══════════════════════════════════════════════════════╣
║  f <val> <ms|mm>  →  Avanzar adelante                ║
║  b <val> <ms|mm>  →  Retroceder                      ║
║  l <val> <ms|mm>  →  Girar izquierda                 ║
║  r <val> <ms|mm>  →  Girar derecha                   ║
║  s                →  Parar motores ahora              ║
║  q                →  Salir                            ║
║  ?                →  Mostrar esta ayuda               ║
║                                                       ║
║  Puedes encadenar comandos separados por coma:        ║
║    f 500 mm, r 300 ms, b 200 mm                       ║
╚══════════════════════════════════════════════════════╝
"""


# =============================================================================
# PARSER DE COMANDOS
# =============================================================================

class ErrorComando(Exception):
    pass


def parsear_comando(texto: str) -> dict:
    """
    Convierte una cadena de texto en un dict de comando.

    Retorna:
        {"tipo": "mover",  "dir": "adelante", "valor": 500, "unidad": "mm"}
        {"tipo": "stop"}
        {"tipo": "salir"}
        {"tipo": "ayuda"}
    """
    partes = texto.strip().lower().split()
    if not partes:
        raise ErrorComando("Comando vacío.")

    alias = partes[0]

    if alias in ("q", "quit", "salir", "exit"):
        return {"tipo": "salir"}

    if alias in ("s", "stop", "parar", "detener"):
        return {"tipo": "stop"}

    if alias in ("?", "help", "ayuda", "h"):
        return {"tipo": "ayuda"}

    if alias not in DIRECCION_METODO:
        raise ErrorComando(
            f"Dirección desconocida: '{alias}'. "
            f"Usa: {', '.join(DIRECCION_METODO.keys())}"
        )

    if len(partes) < 3:
        raise ErrorComando(
            f"Formato: {alias} <valor> <ms|mm>  "
            f"Ejemplo: {alias} 300 ms"
        )

    try:
        valor = float(partes[1])
    except ValueError:
        raise ErrorComando(f"Valor inválido: '{partes[1]}'. Debe ser un número.")

    if valor <= 0:
        raise ErrorComando("El valor debe ser mayor que 0.")

    unidad = partes[2]
    if unidad not in ("ms", "mm"):
        raise ErrorComando(f"Unidad inválida: '{unidad}'. Usa 'ms' o 'mm'.")

    return {
        "tipo":   "mover",
        "dir":    DIRECCION_METODO[alias],
        "alias":  alias,
        "valor":  valor,
        "unidad": unidad,
    }


def parsear_linea(linea: str) -> list:
    """
    Parsea una línea que puede contener varios comandos separados por coma.
    Retorna una lista de dicts de comando.
    """
    segmentos = linea.split(SEPARADOR_COMANDOS)
    comandos = []
    for seg in segmentos:
        seg = seg.strip()
        if seg:
            comandos.append(parsear_comando(seg))
    return comandos


# =============================================================================
# EJECUTOR DE COMANDOS
# =============================================================================

class EjecutorComandos:
    """
    Ejecuta comandos de movimiento con soporte de pausa/reanudación
    ante obstáculos detectados por la visión.
    """

    def __init__(self, robot: RobotMotor, timeout_obstaculo: float = TIMEOUT_OBSTACULO):
        self.robot              = robot
        self.timeout_obstaculo  = timeout_obstaculo
        self._cancelar          = threading.Event()

    def cancelar(self):
        """Señala cancelación del comando en curso."""
        self._cancelar.set()

    def ejecutar(self, cmd: dict) -> bool:
        """
        Ejecuta un comando de movimiento.

        Retorna True si completó, False si fue cancelado/interrumpido.
        """
        self._cancelar.clear()

        if cmd["tipo"] == "stop":
            self.robot.detener()
            print("[Robot] Detenido.")
            return True

        if cmd["tipo"] != "mover":
            return True

        dir_nombre = cmd["dir"]
        valor      = cmd["valor"]
        unidad     = cmd["unidad"]

        print(f"[Robot] Ejecutando: {dir_nombre} {valor} {unidad}")

        if unidad == "ms":
            return self._ejecutar_ms(dir_nombre, valor)
        else:
            return self._ejecutar_mm(dir_nombre, valor)

    def _ejecutar_ms(self, direccion: str, ms_total: float) -> bool:
        """
        Mueve durante ms_total milisegundos, pausando si hay obstáculo
        y reanudando cuando el camino quede libre.
        """
        ms_restantes = ms_total

        while ms_restantes > 0:
            if self._cancelar.is_set():
                self.robot.detener()
                return False

            # Esperar si hay obstáculo
            if not self._esperar_despeje():
                print("[Robot] Tiempo de espera agotado. Cancelando comando.")
                return False

            # Ejecutar el tramo restante
            inicio = time.time()
            exito  = self.robot.mover_ms(
                int(ms_restantes), direccion=direccion,
                velocidad=self.robot.velocidad_default
            )
            transcurrido_ms = (time.time() - inicio) * 1000

            if self._cancelar.is_set():
                return False

            if exito:
                # Completó el tramo sin obstáculo
                return True
            else:
                # Se detuvo por obstáculo: calcular lo que falta
                ms_restantes -= transcurrido_ms
                ms_restantes  = max(0, ms_restantes)
                print(f"[Robot] Obstáculo. Reanudando en {ms_restantes:.0f} ms restantes...")

        return True

    def _ejecutar_mm(self, direccion: str, mm_total: float) -> bool:
        """
        Mueve durante mm_total milímetros, pausando si hay obstáculo
        y reanudando cuando el camino quede libre.
        """
        # Para giros no hay encoder significativo: fallback a tiempo
        if direccion in ("izquierda", "derecha"):
            print("[Aviso] Giros por mm no calibrados; usando tiempo estimado.")
            # Estimación: ~1 mm ≈ 4 ms a velocidad 55 (ajustar según robot)
            ms_est = mm_total * 4
            return self._ejecutar_ms(direccion, ms_est)

        mm_restantes = mm_total

        while mm_restantes > 0:
            if self._cancelar.is_set():
                self.robot.detener()
                return False

            if not self._esperar_despeje():
                print("[Robot] Tiempo de espera agotado. Cancelando comando.")
                return False

            # Leer encoders antes del tramo
            d1_antes, d2_antes = self.robot.distancia_mm

            exito = self.robot.mover_mm(
                mm_restantes, direccion=direccion,
                velocidad=self.robot.velocidad_default
            )

            d1_despues, d2_despues = self.robot.distancia_mm
            recorrido = max(
                abs(d1_despues - d1_antes),
                abs(d2_despues - d2_antes)
            )

            if self._cancelar.is_set():
                return False

            if exito:
                return True
            else:
                mm_restantes -= recorrido
                mm_restantes  = max(0, mm_restantes)
                print(f"[Robot] Obstáculo. Reanudando en {mm_restantes:.1f} mm restantes...")

        return True

    def _esperar_despeje(self) -> bool:
        """
        Bloquea mientras parada_emergencia esté activa.
        Retorna True cuando el camino despeja, False si se agota el timeout.
        """
        if not self.robot.parada_emergencia.is_set():
            return True  # No hay obstáculo, continuar

        print("[Robot] Esperando a que despeje el camino...")
        t_inicio = time.time()

        while self.robot.parada_emergencia.is_set():
            if self._cancelar.is_set():
                return False
            if time.time() - t_inicio > self.timeout_obstaculo:
                return False
            time.sleep(0.1)

        print("[Robot] Camino despejado. Reanudando...")
        time.sleep(0.3)  # Pequeña pausa de seguridad antes de arrancar
        return True


# =============================================================================
# BUCLE PRINCIPAL
# =============================================================================

def bucle_comandos(robot: RobotMotor):
    """Bucle interactivo que lee comandos del usuario y los ejecuta."""
    ejecutor = EjecutorComandos(robot)
    print(AYUDA)
    print("Robot listo. Escribe un comando:\n")

    while True:
        try:
            linea = input(">> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n[Sistema] Interrupción del usuario.")
            break

        if not linea:
            continue

        try:
            comandos = parsear_linea(linea)
        except ErrorComando as e:
            print(f"[Error] {e}")
            continue

        salir = False
        for cmd in comandos:
            if cmd["tipo"] == "ayuda":
                print(AYUDA)
                break

            if cmd["tipo"] == "salir":
                salir = True
                break

            if cmd["tipo"] == "stop":
                ejecutor.cancelar()
                robot.detener()
                print("[Robot] Detenido.")
                break

            # Ejecutar movimiento
            completado = ejecutor.ejecutar(cmd)
            if not completado:
                print("[Robot] Comando cancelado.")
                break

        if salir:
            break

    print("[Sistema] Saliendo del bucle de comandos.")


# =============================================================================
# ENTRY POINT
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Control interactivo del robot diferencial JGY370"
    )
    parser.add_argument(
        "--sin-vision", action="store_true",
        help="Ejecutar sin cámara ni detección de obstáculos"
    )
    parser.add_argument(
        "--puerto", default=None,
        help="Puerto serie (ej. /dev/ttyUSB0). Se detecta automáticamente si no se indica."
    )
    parser.add_argument(
        "--velocidad", type=int, default=VELOCIDAD_DEFAULT,
        help=f"Velocidad PWM por defecto 0-100 (default: {VELOCIDAD_DEFAULT})"
    )
    args = parser.parse_args()

    # ── 1. Conectar robot ──────────────────────────────────────────────────────
    robot = RobotMotor(port=args.puerto, velocidad_default=args.velocidad)
    try:
        robot.conectar()
        print(f"[Sistema] Robot conectado (velocidad base: {args.velocidad}).")
    except Exception as e:
        print(f"[Error] No se pudo conectar al robot: {e}")
        sys.exit(1)

    # ── 2. Iniciar visión (opcional) ───────────────────────────────────────────
    vision = None
    usar_vision = (not args.sin_vision) and VISION_DISPONIBLE

    if usar_vision:
        try:
            vision = VisionObstaculos(robot=robot, mostrar_ventana=False)
            vision.iniciar()
            print("[Sistema] Visión iniciada.")
        except Exception as e:
            print(f"[Aviso] No se pudo iniciar la visión: {e}")
            print("[Aviso] Continuando sin detección de obstáculos.")
            vision = None
    else:
        print("[Sistema] Modo sin visión.")

    # ── 3. Ejecutar bucle de comandos ──────────────────────────────────────────
    try:
        bucle_comandos(robot)
    finally:
        robot.detener()
        if vision:
            vision.detener()
        robot.desconectar()
        print("[Sistema] Sistema apagado correctamente.")


if __name__ == "__main__":
    main()
