"""
Lector de QR para Raspberry Pi (Bookworm + IMX219)
----------------------------------------------------
Instalacion:
    sudo apt install -y python3-picamera2 python3-opencv libzbar0
    pip install pyzbar

Uso:
    python lector_qr_rasberry.py

Controles:
    q  → salir
    r  → resetear historial
"""

import cv2
import webbrowser
from picamera2 import Picamera2
from pyzbar import pyzbar


def leer_qr():
    # Iniciar camara con Picamera2
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"format": "RGB888", "size": (640, 480)}
    )
    picam2.configure(config)
    picam2.start()

    print("Camara activa. Apunta a un QR. Presiona 'q' para salir.")

    codigos_leidos = set()

    while True:
        # Capturar frame como array numpy
        frame = picam2.capture_array()

        # Convertir RGB a BGR para OpenCV
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # Decodificar QR en escala de grises (mas preciso)
        gris = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        codigos = pyzbar.decode(gris)

        for codigo in codigos:
            datos = codigo.data.decode("utf-8")
            tipo  = codigo.type

            # Dibujar rectangulo alrededor del QR
            x, y, w, h = codigo.rect
            cv2.rectangle(frame_bgr, (x, y), (x + w, y + h), (0, 230, 160), 2)

            # Mostrar texto encima del QR
            cv2.putText(
                frame_bgr, datos[:50],
                (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (0, 230, 160), 2
            )

            # Actuar solo si es un codigo nuevo
            if datos not in codigos_leidos:
                codigos_leidos.add(datos)
                print(f"\n[{tipo}] Contenido: {datos}")

                # Si es URL abrir en navegador
                if datos.startswith("http://") or datos.startswith("https://"):
                    print(f"Abriendo enlace: {datos}")
                    webbrowser.open(datos)
                else:
                    print("Dato leido (no es URL).")

                # --- Aqui va la logica del carrito en el futuro ---
                # if datos.startswith("RUTA:"):
                #     direccion = datos.split(":")[1]
                #     mover_carro(direccion)

        # Overlay de estado
        estado = f"QRs leidos: {len(codigos_leidos)}"
        cv2.putText(
            frame_bgr, estado,
            (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7, (255, 255, 255), 2
        )

        instruccion = "Presiona 'q' para salir  |  'r' para resetear"
        cv2.putText(
            frame_bgr, instruccion,
            (10, frame_bgr.shape[0] - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45, (160, 160, 160), 1
        )

        cv2.imshow("Lector QR - Raspberry Pi", frame_bgr)

        tecla = cv2.waitKey(1) & 0xFF
        if tecla == ord("q"):
            print("Saliendo...")
            break
        elif tecla == ord("r"):
            codigos_leidos.clear()
            print("Historial reseteado.")

    picam2.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    leer_qr()
