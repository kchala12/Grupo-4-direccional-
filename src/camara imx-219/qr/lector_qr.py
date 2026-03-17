"""
Lector de QR con OpenCV + pyzbar
----------------------------------
Instalacion:
    pip install opencv-python pyzbar

En Raspberry Pi tambien instala:
    sudo apt install libzbar0
"""

import cv2
import webbrowser
from pyzbar import pyzbar


def leer_qr():
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("No se pudo abrir la camara.")
        return

    print("Camara activa. Apunta a un QR. Presiona 'q' para salir.")

    codigos_leidos = set()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error leyendo el frame.")
            break

        codigos = pyzbar.decode(frame)

        for codigo in codigos:
            datos = codigo.data.decode("utf-8")
            tipo  = codigo.type

            # Dibujar rectangulo alrededor del QR
            puntos = codigo.polygon
            if len(puntos) == 4:
                pts = [(p.x, p.y) for p in puntos]
                for i in range(4):
                    cv2.line(frame, pts[i], pts[(i + 1) % 4], (0, 230, 160), 2)

            # Mostrar texto encima del QR
            x, y, w, h = codigo.rect
            cv2.putText(
                frame, datos[:50],
                (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (0, 230, 160), 2
            )

            # Actuar solo si es un codigo nuevo
            if datos not in codigos_leidos:
                codigos_leidos.add(datos)
                print(f"\n[{tipo}] Contenido: {datos}")

                # Si es una URL, abrirla en el navegador
                if datos.startswith("http://") or datos.startswith("https://"):
                    print(f"Abriendo enlace: {datos}")
                    webbrowser.open(datos)
                else:
                    print("No es una URL. Dato leido arriba.")

        # Overlay de estado
        estado = f"QRs leidos: {len(codigos_leidos)}"
        cv2.putText(
            frame, estado,
            (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7, (255, 255, 255), 2
        )

        instruccion = "Presiona 'q' para salir  |  'r' para resetear"
        cv2.putText(
            frame, instruccion,
            (10, frame.shape[0] - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45, (160, 160, 160), 1
        )

        cv2.imshow("Lector QR", frame)

        tecla = cv2.waitKey(1) & 0xFF
        if tecla == ord("q"):
            print("Saliendo...")
            break
        elif tecla == ord("r"):
            codigos_leidos.clear()
            print("Historial de codigos reseteado.")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    leer_qr()
