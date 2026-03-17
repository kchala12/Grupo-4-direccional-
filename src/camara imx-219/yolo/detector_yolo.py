"""
Detector de obstaculos con YOLOv8 preentrenado
------------------------------------------------
Instalacion:
    pip install ultralytics opencv-python

Uso:
    python detector_yolo.py

Controles:
    q  → salir
    r  → resetear alertas
"""

import cv2
from ultralytics import YOLO

# -----------------------------------------------------------
# CONFIGURACION
# -----------------------------------------------------------

# Modelo: yolov8n = nano (mas rapido, menos pesado pa Raspberry)
#         yolov8s = small (mas preciso)
MODELO = "yolov8n.pt"

# Clases que nos interesan
# AHORA: solo carro (obstaculo) con el modelo preentrenado
# DESPUES: cuando entrenes tu modelo cambia esto a ["via", "carro"]
CLASES_INTERES = {
    "car":        "carro",
    "truck":      "carro",
    "motorcycle": "carro",
    "bicycle":    "carro",
}

# Confianza minima para detectar (0.0 - 1.0)
CONFIANZA = 0.45

# Zona de peligro: si el objeto ocupa mas de este % del ancho → ALERTA
UMBRAL_PELIGRO = 0.30

# Colores BGR
COLOR_SEGURO  = (0, 230, 160)   # verde
COLOR_PELIGRO = (0, 60, 255)    # rojo
COLOR_INFO    = (255, 255, 255) # blanco
COLOR_OVERLAY = (20, 20, 30)    # fondo overlay

# -----------------------------------------------------------

def calcular_zona(x1, x2, frame_w):
    """Determina si el objeto esta en zona izquierda, centro o derecha."""
    cx = (x1 + x2) / 2
    tercio = frame_w / 3
    if cx < tercio:
        return "IZQUIERDA"
    elif cx < tercio * 2:
        return "CENTRO"
    else:
        return "DERECHA"

def es_peligroso(x1, x2, frame_w):
    """True si el objeto ocupa mas del umbral del ancho del frame."""
    ancho_obj = x2 - x1
    return (ancho_obj / frame_w) > UMBRAL_PELIGRO

def dibujar_caja(frame, x1, y1, x2, y2, etiqueta, peligro):
    color = COLOR_PELIGRO if peligro else COLOR_SEGURO
    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

    # Fondo del texto
    (tw, th), _ = cv2.getTextSize(etiqueta, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
    cv2.rectangle(frame, (x1, y1 - th - 8), (x1 + tw + 6, y1), color, -1)
    cv2.putText(frame, etiqueta, (x1 + 3, y1 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 2)

def dibujar_overlay(frame, alertas, total):
    h, w = frame.shape[:2]

    # Barra superior
    cv2.rectangle(frame, (0, 0), (w, 38), COLOR_OVERLAY, -1)
    cv2.putText(frame, f"Objetos detectados: {total}",
                (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_INFO, 2)
    cv2.putText(frame, "q=salir  r=reset",
                (w - 175, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (130, 130, 130), 1)

    # Alertas de peligro
    if alertas:
        msg = "ALERTA: " + " | ".join(alertas)
        cv2.rectangle(frame, (0, h - 40), (w, h), (0, 0, 180), -1)
        cv2.putText(frame, msg, (10, h - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLOR_INFO, 2)

def detectar_linea(frame):
    """
    Detecta la linea/via por color usando OpenCV.
    Por defecto busca linea NEGRA (cinta negra en piso claro).
    Cambia los valores HSV si tu linea es de otro color:
      - Linea blanca:  lower=(0,0,200)   upper=(180,30,255)
      - Linea amarilla: lower=(20,100,100) upper=(30,255,255)
      - Linea roja:    lower=(0,120,70)   upper=(10,255,255)
    """
    h, w = frame.shape[:2]
    roi  = frame[h//2:, :]  # solo mitad inferior (donde esta la via)

    hsv  = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # Linea negra
    lower_negro    = (0,   0,   0)
    upper_negro    = (180, 255, 80)

    # Linea amarilla
    lower_amarillo = (18, 80, 80)
    upper_amarillo = (35, 255, 255)

    mascara_negro    = cv2.inRange(hsv, lower_negro,    upper_negro)
    mascara_amarillo = cv2.inRange(hsv, lower_amarillo, upper_amarillo)
    mascara          = cv2.bitwise_or(mascara_negro, mascara_amarillo)
    mascara = cv2.GaussianBlur(mascara, (5, 5), 0)

    contornos, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    linea_detectada = False
    posicion_linea  = "DESCONOCIDA"

    if contornos:
        c = max(contornos, key=cv2.contourArea)
        if cv2.contourArea(c) > 500:
            M = cv2.moments(c)
            if M["m00"] != 0:
                cx_linea = int(M["m10"] / M["m00"])
                linea_detectada = True

                # Dibujar contorno en el ROI
                cv2.drawContours(frame[h//2:], [c], -1, (0, 200, 255), 2)

                # Centro de la linea
                cy_linea = int(M["m01"] / M["m00"]) + h//2
                cv2.circle(frame, (cx_linea, cy_linea), 6, (0, 200, 255), -1)

                # Posicion respecto al centro
                if cx_linea < w // 3:
                    posicion_linea = "IZQUIERDA"
                elif cx_linea < (w * 2) // 3:
                    posicion_linea = "CENTRO"
                else:
                    posicion_linea = "DERECHA"

                cv2.putText(frame, f"linea [{posicion_linea}]",
                            (cx_linea - 60, cy_linea - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)

    return linea_detectada, posicion_linea


def main():
    print("Cargando modelo YOLOv8...")
    modelo = YOLO(MODELO)
    print("Modelo listo.")

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("No se pudo abrir la camara.")
        return

    print("Camara activa. Presiona 'q' para salir.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error leyendo frame.")
            break

        h, w = frame.shape[:2]

        # --- Deteccion de linea/via (OpenCV) ---
        linea_ok, pos_linea = detectar_linea(frame)

        # --- Deteccion de carros/obstaculos (YOLO) ---
        resultados = modelo(frame, conf=CONFIANZA, verbose=False)[0]

        alertas = []
        total   = 0

        for det in resultados.boxes:
            clase_id   = int(det.cls[0])
            clase_name = modelo.names[clase_id]

            if clase_name not in CLASES_INTERES:
                continue

            total += 1
            etiqueta_es = CLASES_INTERES[clase_name]
            conf        = float(det.conf[0])

            x1, y1, x2, y2 = map(int, det.xyxy[0])

            zona    = calcular_zona(x1, x2, w)
            peligro = es_peligroso(x1, x2, w)

            etiqueta = f"{etiqueta_es} {conf:.0%} [{zona}]"
            dibujar_caja(frame, x1, y1, x2, y2, etiqueta, peligro)

            if peligro:
                alertas.append(f"{etiqueta_es} en {zona}")

        # Info de linea en overlay
        if not linea_ok:
            alertas.append("VIA NO DETECTADA")

        dibujar_overlay(frame, alertas, total)

        # Estado de linea arriba a la derecha
        color_linea = COLOR_SEGURO if linea_ok else COLOR_PELIGRO
        texto_linea = f"via: {pos_linea}" if linea_ok else "via: NO DETECTADA"
        cv2.putText(frame, texto_linea, (w - 240, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_linea, 2)

        # --- Aqui va la logica del carrito en el futuro ---
        # if "carro" in [a.split()[0] for a in alertas]:
        #     detener_carro()
        # elif linea_ok:
        #     seguir_linea(pos_linea)
        # else:
        #     buscar_linea()

        cv2.imshow("Detector YOLO - Carrito", frame)

        tecla = cv2.waitKey(1) & 0xFF
        if tecla == ord("q"):
            print("Saliendo...")
            break
        elif tecla == ord("r"):
            print("Alertas reseteadas.")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()