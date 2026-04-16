#!/usr/bin/env python3
"""
robot_motor_lib.py
==================
Librería de control del robot diferencial JGY370.
Encapsula la comunicación serial con la ESP32, la lectura
de encoders y el controlador PID por motor.

Uso desde programa principal:
    from robot_motor_lib import RobotMotor

    robot = RobotMotor()          # detecta puerto automáticamente
    robot.conectar()
    robot.adelante(velocidad=60)
    robot.mover_mm(300)           # avanza 300 mm y para solo
    robot.detener()
    robot.desconectar()
"""

import serial
import serial.tools.list_ports
import threading
import time
import math

# =========================================================
# CALIBRACIÓN — ajustar con valores reales del robot
# =========================================================
M1_PULSOS_POR_VUELTA = 297     # Rueda izquierda
M2_PULSOS_POR_VUELTA = 2682    # Rueda derecha
RADIO_MM             = 30.0    # Radio de rueda en mm

# =========================================================
# PARÁMETROS DEL CONTROLADOR PID
# =========================================================
CTRL_KP             = 1.0    # Ganancia proporcional
CTRL_INTERVALO_S    = 0.1    # Período de recálculo (segundos)
CTRL_CORRECCION_MAX = 20     # Límite de corrección de PWM

# Derivados de calibración
M1_MM_POR_PULSO = (2 * math.pi * RADIO_MM) / M1_PULSOS_POR_VUELTA
M2_MM_POR_PULSO = (2 * math.pi * RADIO_MM) / M2_PULSOS_POR_VUELTA

# Direcciones en las que el PID actúa (movimiento recto)
DIRS_CON_PID = ("adelante", "atras")


class RobotMotor:
    """
    Interfaz de alto nivel para controlar el robot.

    Parámetros
    ----------
    port : str | None
        Puerto serie (ej. '/dev/ttyUSB0'). Si es None, se detecta solo.
    baud : int
        Velocidad de comunicación (debe coincidir con la ESP32).
    velocidad_default : int
        Velocidad PWM usada cuando no se especifica (0-100).
    """

    def __init__(self, port=None, baud=115200, velocidad_default=50):
        self.port               = port
        self.baud               = baud
        self.velocidad_default  = velocidad_default
        self._ser               = None

        # Pulsos de encoder leídos de la ESP32
        self._enc = {"c1": 0, "c2": 0}

        # Estado del controlador PID
        self._pid = {
            "activo":   False,
            "vel_base": velocidad_default,
            "vel_m1":   velocidad_default,
            "vel_m2":   velocidad_default,
            "dir":      "",
        }

        # Eventos de parada para los hilos internos
        self._stop_lector = threading.Event()
        self._stop_pid    = threading.Event()

        # Bandera pública para parada de emergencia (ej. desde visión)
        self.parada_emergencia = threading.Event()

    # ----------------------------------------------------------
    # CONEXIÓN
    # ----------------------------------------------------------

    def conectar(self):
        """
        Abre el puerto serie y lanza los hilos de fondo.
        Lanza serial.SerialException si el puerto no existe.
        """
        puerto = self.port or self._buscar_puerto()
        if not puerto:
            raise serial.SerialException(
                "No se detectó ESP32. Usa port='/dev/ttyUSB0'."
            )

        self._ser = serial.Serial(puerto, baudrate=self.baud, timeout=1)
        time.sleep(2)                     # esperar reset del ESP32
        self._ser.reset_input_buffer()

        self._stop_lector.clear()
        self._stop_pid.clear()

        threading.Thread(
            target=self._hilo_lector, daemon=True
        ).start()
        threading.Thread(
            target=self._hilo_controlador, daemon=True
        ).start()

        return self   # permite encadenar: robot = RobotMotor().conectar()

    def desconectar(self):
        """Detiene motores, para los hilos y cierra el puerto."""
        self._pid["activo"] = False
        self._enviar("stop 0")
        time.sleep(0.1)

        self._stop_lector.set()
        self._stop_pid.set()

        if self._ser and self._ser.is_open:
            self._ser.close()

    # Soporte para `with RobotMotor() as robot:`
    def __enter__(self):
        return self.conectar()

    def __exit__(self, *args):
        self.desconectar()

    # ----------------------------------------------------------
    # PROPIEDADES DE LECTURA
    # ----------------------------------------------------------

    @property
    def pulsos(self):
        """Devuelve (pulsos_M1, pulsos_M2) acumulados desde el reset."""
        return self._enc["c1"], self._enc["c2"]

    @property
    def distancia_mm(self):
        """Devuelve (dist_M1_mm, dist_M2_mm) recorridas."""
        c1, c2 = self.pulsos
        return c1 * M1_MM_POR_PULSO, c2 * M2_MM_POR_PULSO

    # ----------------------------------------------------------
    # CONTROL DE MOVIMIENTO
    # ----------------------------------------------------------

    def adelante(self, velocidad=None):
        """Mueve el robot hacia adelante de forma indefinida."""
        self._mover_indefinido("adelante", velocidad)

    def atras(self, velocidad=None):
        """Mueve el robot hacia atrás de forma indefinida."""
        self._mover_indefinido("atras", velocidad)

    def derecha(self, velocidad=None):
        """Gira el robot a la derecha."""
        self._mover_indefinido("derecha", velocidad)

    def izquierda(self, velocidad=None):
        """Gira el robot a la izquierda."""
        self._mover_indefinido("izquierda", velocidad)

    def detener(self):
        """Para los motores inmediatamente."""
        self._pid["activo"] = False
        self._enviar("stop 0")

    def reset_encoders(self):
        """Pone los contadores de encoder a cero."""
        self._enviar("reset 0")

    def set_velocidad(self, velocidad):
        """Cambia la velocidad base (0-100)."""
        self.velocidad_default   = velocidad
        self._pid["vel_base"]    = velocidad

    # ----------------------------------------------------------
    # MOVIMIENTO CON DISTANCIA O TIEMPO
    # ----------------------------------------------------------

    def mover_mm(self, distancia_mm, direccion="adelante", velocidad=None):
        """
        Mueve el robot una distancia en milímetros y para automáticamente.
        Bloquea hasta que la distancia se alcanza.

        Parámetros
        ----------
        distancia_mm : float  — distancia a recorrer
        direccion    : str    — 'adelante' o 'atras'
        velocidad    : int    — PWM 0-100 (usa default si None)
        """
        vel       = velocidad or self.velocidad_default
        antes1, antes2 = self._enc["c1"], self._enc["c2"]
        objetivo1 = distancia_mm / M1_MM_POR_PULSO
        objetivo2 = distancia_mm / M2_MM_POR_PULSO

        if direccion in DIRS_CON_PID:
            self._pid["dir"]      = direccion
            self._pid["vel_base"] = vel
            self._pid["activo"]   = True

        self._enviar(f"{direccion} {vel}")

        while True:
            # Parada de emergencia externa (ej. desde visión)
            if self.parada_emergencia.is_set():
                self.detener()
                return False

            if (abs(self._enc["c1"] - antes1) >= objetivo1 and
                    abs(self._enc["c2"] - antes2) >= objetivo2):
                self._pid["activo"] = False
                self.detener()
                return True

            time.sleep(0.02)

    def mover_ms(self, milisegundos, direccion="adelante", velocidad=None):
        """
        Mueve el robot durante un tiempo en milisegundos y para automáticamente.

        Retorna False si la parada de emergencia se activó antes de terminar.
        """
        vel = velocidad or self.velocidad_default

        if direccion in DIRS_CON_PID:
            self._pid["dir"]      = direccion
            self._pid["vel_base"] = vel
            self._pid["activo"]   = True

        self._enviar(f"{direccion} {vel}")

        inicio = time.time()
        while (time.time() - inicio) < (milisegundos / 1000.0):
            if self.parada_emergencia.is_set():
                self.detener()
                return False
            time.sleep(0.02)

        self._pid["activo"] = False
        self.detener()
        return True

    # ----------------------------------------------------------
    # API INTERNA
    # ----------------------------------------------------------

    def _enviar(self, cmd):
        if self._ser and self._ser.is_open:
            self._ser.write((cmd + "\n").encode("utf-8"))

    def _buscar_puerto(self):
        chips = ["CP210", "CH340", "CH341", "FTDI", "FT232",
                 "USB Serial", "ESP32"]
        for p in serial.tools.list_ports.comports():
            d = (p.description   or "").upper()
            m = (p.manufacturer  or "").upper()
            if any(c in d or c in m for c in chips):
                return p.device
        pts = serial.tools.list_ports.comports()
        return pts[0].device if pts else None

    # ----------------------------------------------------------
    # HILOS DE FONDO
    # ----------------------------------------------------------

    def _hilo_lector(self):
        """Lee las tramas 'E c1 c2' enviadas por la ESP32 cada 100ms."""
        while not self._stop_lector.is_set():
            try:
                if self._ser.in_waiting:
                    linea = (self._ser.readline()
                             .decode("utf-8", errors="replace")
                             .strip())
                    if linea.startswith("E "):
                        p = linea.split()
                        if len(p) == 3:
                            self._enc["c1"] = int(p[1])
                            self._enc["c2"] = int(p[2])
            except Exception:
                pass
            time.sleep(0.01)

    def _hilo_controlador(self):
        """
        Controlador PID independiente por motor.
        Solo actúa cuando pid['activo'] es True y la dirección
        es una de DIRS_CON_PID (movimiento recto).
        """
        tiempo_inicio  = 0.0
        pulsos_inicio1 = 0
        pulsos_inicio2 = 0
        tasa_esp1      = 0.0
        tasa_esp2      = 0.0

        while not self._stop_pid.is_set():
            time.sleep(CTRL_INTERVALO_S)

            if not self._pid["activo"]:
                tiempo_inicio = 0.0
                tasa_esp1     = 0.0
                tasa_esp2     = 0.0
                continue

            if tiempo_inicio == 0.0:
                tiempo_inicio  = time.time()
                pulsos_inicio1 = self._enc["c1"]
                pulsos_inicio2 = self._enc["c2"]
                continue

            t_el    = time.time() - tiempo_inicio
            reales1 = abs(self._enc["c1"] - pulsos_inicio1)
            reales2 = abs(self._enc["c2"] - pulsos_inicio2)

            if tasa_esp1 == 0.0 and reales1 > 0:
                tasa_esp1 = reales1 / t_el
            if tasa_esp2 == 0.0 and reales2 > 0:
                tasa_esp2 = reales2 / t_el

            if tasa_esp1 == 0.0 or tasa_esp2 == 0.0:
                continue

            esp1   = tasa_esp1 * t_el
            esp2   = tasa_esp2 * t_el
            err1   = reales1 - esp1
            err2   = reales2 - esp2
            corr1  = max(-CTRL_CORRECCION_MAX,
                         min(CTRL_CORRECCION_MAX, CTRL_KP * err1))
            corr2  = max(-CTRL_CORRECCION_MAX,
                         min(CTRL_CORRECCION_MAX, CTRL_KP * err2))
            base   = self._pid["vel_base"]
            vm1    = int(max(0, min(100, base - corr1)))
            vm2    = int(max(0, min(100, base - corr2)))

            self._pid["vel_m1"] = vm1
            self._pid["vel_m2"] = vm2

            direccion = self._pid.get("dir", "")
            if direccion:
                self._enviar(f"pid {vm1} {vm2} {direccion}")
