#!/usr/bin/env python3
"""
vision_obstaculos.py
====================
Detección de obstáculos con cámara IMX219 (Raspberry Pi) y OpenCV.
Detiene los motores automáticamente cuando detecta un obstáculo.

Dependencias:
    pip install opencv-python numpy

Uso independiente:
    python3 vision_obstaculos.py

Uso como módulo desde el programa principal:
    from vision_obstaculos import VisionObstaculos
    from robot_motor_lib   import RobotMotor

    robot  = RobotMotor().conectar()
    vision = VisionObstaculos(robot=robot)
    vision.iniciar()           # lanza hilo de visión en segundo plano

    robot.adelante(60)
    # → vision para el robot automáticamente si detecta obstáculo

    vision.detener()
    robot.desconectar()
"""

import cv2
import numpy as np
import threading
import time


# =========================================================
# PARÁMETROS DE DETECCIÓN — ajustar según entorno
# =========================================================

# Fracción del ancho de imagen que define la zona central de análisis.
# 0.5 = analizar el 50% central de la imagen.
ZONA_CENTRAL_FRACCION = 0.5

# Fracción del alto de imagen analizada (mitad inferior).
# Ayuda a ignorar el cielo u objetos lejanos.
ZONA_VERTICAL_FRACCION = 0.5

# Porcentaje de pixeles "de primer plano" en la zona de interés
# que dispara la parada. Rango recomendado: 0.05 – 0.20.
UMBRAL_DETECCION = 0.08

# Distancia mínima entre contornos (píxeles) para considerarlos
# un obstáculo real y no ruido.
AREA_MINIMA_CONTORNO = 3000

# Segundos que el robot espera detenido antes de intentar moverse
# de nuevo (si la parada la disparó la visión).
TIEMPO_ESPERA_POST_STOP = 2.0

# Parámetros internos del detector de fondo (BackgroundSubtractor).
# history: cuántos frames se usan para construir el modelo de fondo.
# varThreshold: sensibilidad — mayor valor = menos sensible.
HIST_FONDO   = 300
VAR_UMBRAL   = 40


class VisionObstaculos:
    """
    Abre la cámara IMX219, detecta obstáculos en tiempo real
    y activa robot.parada_emergencia cuando detecta uno.

    Parámetros
    ----------
    robot : RobotMotor | None
        Instancia de RobotMotor. Si se pasa None, solo detecta
        sin controlar motores (útil para probar la visión sola).
    indice_camara : int
        Índice de la cámara. 0 para la primera cámara disponible.
        Con libcamera/GStreamer puede necesitar una pipeline string.
    resolucion : tuple
        (ancho, alto) de captura. Menor = más rápido.
    fps_objetivo : int
        Frames por segundo máximos del hilo de visión.
    mostrar_ventana : bool
        Muestra ventana de debug con los contornos detectados.
        Requiere entorno gráfico (X11 / VNC).
    usar_gstreamer : bool
        Si True, usa GStreamer para abrir la IMX219 con libcamera.
        Recomendado en Raspberry Pi con cámara oficial.
    """

    def __init__(
        self,
        robot=None,
        indice_camara=0,
        resolucion=(640, 480),
        fps_objetivo=15,
        mostrar_ventana=False,
        usar_gstreamer=True,
    ):
        self.robot           = robot
        self.resolucion      = resolucion
        self.fps_objetivo    = fps_objetivo
        self.mostrar_ventana = mostrar_ventana
        self.usar_gstreamer  = usar_gstreamer
        self._indice_camara  = indice_camara

        self._cap            = None
        self._hilo           = None
        self._corriendo      = threading.Event()

        # Último frame y máscara de detección (para lectura externa)
        self.frame_actual    = None
        self.mascara_actual  = None

        # Contador de obstáculos detectados (útil para logging)
        self.obstaculos_detectados = 0

        # Callback opcional: se llama cada vez que se detecta un obstáculo.
        # Firma: callback(frame, mascara)
        self.on_obstaculo = None

        # Modelo de fondo adaptativo
        self._substractor   = cv2.createBackgroundSubtractorMOG2(
            history=HIST_FONDO,
            varThreshold=VAR_UMBRAL,
            detectShadows=False,
        )

    # ----------------------------------------------------------
    # CICLO DE VIDA
    # ----------------------------------------------------------

    def iniciar(self):
        """Abre la cámara y lanza el hilo de visión en segundo plano."""
        self._cap = self._abrir_camara()
        if self._cap is None or not self._cap.isOpened():
            raise RuntimeError(
                "No se pudo abrir la cámara. "
                "Verifica que la IMX219 esté conectada y habilitada."
            )

        self._corriendo.set()
        self._hilo = threading.Thread(
            target=self._bucle_vision, daemon=True
        )
        self._hilo.start()
        print("[Vision] Iniciada.")
        return self

    def detener(self):
        """Para el hilo de visión y libera la cámara."""
        self._corriendo.clear()
        if self._hilo:
            self._hilo.join(timeout=3)
        if self._cap:
            self._cap.release()
        if self.mostrar_ventana:
            cv2.destroyAllWindows()
        print("[Vision] Detenida.")

    def __enter__(self):
        return self.iniciar()

    def __exit__(self, *args):
        self.detener()

    # ----------------------------------------------------------
    # BUCLE PRINCIPAL
    # ----------------------------------------------------------

    def _bucle_vision(self):
        intervalo = 1.0 / self.fps_objetivo

        while self._corriendo.is_set():
            t0 = time.time()

            ok, frame = self._cap.read()
            if not ok or frame is None:
                time.sleep(0.1)
                continue

            self.frame_actual = frame.copy()
            hay_obstaculo, mascara = self._detectar(frame)
            self.mascara_actual = mascara

            if hay_obstaculo:
                self.obstaculos_detectados += 1
                print(f"[Vision] ¡Obstáculo detectado! (#{self.obstaculos_detectados})")

                # Parar motores
                if self.robot is not None:
                    self.robot.parada_emergencia.set()
                    self.robot.detener()

                # Ejecutar callback externo si existe
                if callable(self.on_obstaculo):
                    self.on_obstaculo(frame, mascara)

                # Esperar antes de limpiar la señal de emergencia
                time.sleep(TIEMPO_ESPERA_POST_STOP)
                if self.robot is not None:
                    self.robot.parada_emergencia.clear()

            if self.mostrar_ventana:
                self._mostrar_debug(frame, mascara, hay_obstaculo)

            # Mantener el FPS objetivo
            elapsed = time.time() - t0
            tiempo_restante = intervalo - elapsed
            if tiempo_restante > 0:
                time.sleep(tiempo_restante)

    # ----------------------------------------------------------
    # DETECCIÓN
    # ----------------------------------------------------------

    def _detectar(self, frame):
        """
        Aplica la pipeline de detección y devuelve (bool, mascara).
        Pipeline:
          1. Recortar zona de interés (ROI)
          2. Desenfoque gaussiano para reducir ruido
          3. Substractor de fondo adaptativo
          4. Operaciones morfológicas para limpiar la máscara
          5. Búsqueda de contornos significativos
        """
        alto, ancho = frame.shape[:2]

        # ── 1. Región de interés ────────────────────────────────
        # Zona central horizontal + mitad inferior vertical
        x_ini = int(ancho * (1 - ZONA_CENTRAL_FRACCION) / 2)
        x_fin = ancho - x_ini
        y_ini = int(alto * (1 - ZONA_VERTICAL_FRACCION))
        roi   = frame[y_ini:alto, x_ini:x_fin]

        # ── 2. Preprocesado ─────────────────────────────────────
        borroso = cv2.GaussianBlur(roi, (7, 7), 0)

        # ── 3. Substracción de fondo ────────────────────────────
        mascara = self._substractor.apply(borroso)

        # ── 4. Morfología ───────────────────────────────────────
        kernel  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        mascara = cv2.morphologyEx(mascara, cv2.MORPH_OPEN,  kernel)
        mascara = cv2.morphologyEx(mascara, cv2.MORPH_CLOSE, kernel)
        mascara = cv2.dilate(mascara, kernel, iterations=2)

        # ── 5. Contornos ─────────────────────────────────────────
        contornos, _ = cv2.findContours(
            mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        contornos_grandes = [
            c for c in contornos
            if cv2.contourArea(c) >= AREA_MINIMA_CONTORNO
        ]

        # También verificar por porcentaje de píxeles activos
        pixeles_activos  = np.count_nonzero(mascara)
        total_roi        = roi.shape[0] * roi.shape[1]
        fraccion_activos = pixeles_activos / max(total_roi, 1)

        hay_obstaculo = (
            len(contornos_grandes) > 0
            or fraccion_activos > UMBRAL_DETECCION
        )

        return hay_obstaculo, mascara

    # ----------------------------------------------------------
    # DEBUG VISUAL
    # ----------------------------------------------------------

    def _mostrar_debug(self, frame, mascara, hay_obstaculo):
        """Dibuja la ROI, contornos y estado sobre el frame."""
        alto, ancho = frame.shape[:2]
        x_ini = int(ancho * (1 - ZONA_CENTRAL_FRACCION) / 2)
        x_fin = ancho - x_ini
        y_ini = int(alto * (1 - ZONA_VERTICAL_FRACCION))

        debug = frame.copy()

        # Rectángulo de la ROI
        color_roi = (0, 0, 255) if hay_obstaculo else (0, 255, 0)
        cv2.rectangle(debug, (x_ini, y_ini), (x_fin, alto), color_roi, 2)

        # Texto de estado
        estado = "OBSTACULO" if hay_obstaculo else "libre"
        cv2.putText(
            debug, estado, (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX, 1.0, color_roi, 2
        )

        cv2.imshow("Vision - Frame",  debug)
        cv2.imshow("Vision - Mascara", mascara)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            self._corriendo.clear()

    # ----------------------------------------------------------
    # APERTURA DE CÁMARA
    # ----------------------------------------------------------

    def _abrir_camara(self):
        """
        Intenta abrir la IMX219 con GStreamer (recomendado en RPi)
        y cae de vuelta al índice numérico si falla.
        """
        ancho, alto = self.resolucion

        if self.usar_gstreamer:
            # Pipeline para Raspberry Pi con libcamera + GStreamer
            pipeline = (
                f"libcamerasrc ! "
                f"video/x-raw,width={ancho},height={alto},framerate={self.fps_objetivo}/1 ! "
                f"videoconvert ! "
                f"video/x-raw,format=BGR ! "
                f"appsink drop=1"
            )
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if cap.isOpened():
                print(f"[Vision] Cámara abierta con GStreamer ({ancho}x{alto}).")
                return cap
            print("[Vision] GStreamer no disponible, intentando apertura directa...")

        # Fallback: apertura directa por índice
        cap = cv2.VideoCapture(self._indice_camara)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,  ancho)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, alto)
            cap.set(cv2.CAP_PROP_FPS,          self.fps_objetivo)
            print(f"[Vision] Cámara abierta por índice {self._indice_camara}.")
            return cap

        return None


# =========================================================
# EJECUCIÓN DIRECTA — prueba sin robot
# =========================================================

if __name__ == "__main__":
    print("=== Prueba de detección de obstáculos (sin robot) ===")
    print("Presiona 'q' en la ventana para salir.\n")

    def _callback(frame, mascara):
        print("  → Callback: obstáculo confirmado por visión.")

    vision = VisionObstaculos(
        robot=None,
        mostrar_ventana=True,   # Requiere entorno gráfico
        usar_gstreamer=True,
    )
    vision.on_obstaculo = _callback

    try:
        vision.iniciar()
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        vision.detener()
        print(f"\nTotal obstáculos detectados: {vision.obstaculos_detectados}")
