"""
Detector QR + YOLOv8  —  solo OpenCV (sin pyzbar)
===================================================
Instalacion:
    pip install opencv-python ultralytics

Uso:
    # Deteccion en tiempo real con pesos YOLOv8 por defecto
    python qr_yolo_detector.py

    # Entrenar con imagenes propias y luego detectar
    python qr_yolo_detector.py --train --data mi_dataset.yaml --epochs 50

    # Usar pesos propios ya entrenados
    python qr_yolo_detector.py --weights runs/train/mi_modelo/weights/best.pt

Estructura del dataset (formato Ultralytics / YOLO):
    mi_dataset/
        images/
            train/   *.jpg
            val/     *.jpg
        labels/
            train/   *.txt   (<clase> <cx> <cy> <w> <h>  normalizados)
            val/     *.txt

    mi_dataset.yaml:
        path: ./mi_dataset
        train: images/train
        val:   images/val
        nc: 2
        names:
          0: qr_code
          1: barcode
"""

import argparse
import webbrowser
import cv2
from ultralytics import YOLO


# ──────────────────────────────────────────────
# Colores BGR
# ──────────────────────────────────────────────
COLOR_QR   = (0, 230, 160)    # verde   → contorno QR
COLOR_YOLO = (0, 165, 255)    # naranja → cajas YOLO
COLOR_HUD  = (255, 255, 255)  # blanco  → HUD
COLOR_HINT = (160, 160, 160)  # gris    → instrucciones


# ══════════════════════════════════════════════
#  DETECCION EN TIEMPO REAL
# ══════════════════════════════════════════════
def detectar(model: YOLO, conf: float = 0.45) -> None:
    """
    Corre en paralelo:
      • cv2.QRCodeDetector   → lectura QR pura OpenCV (sin pyzbar)
      • YOLOv8 (Ultralytics) → deteccion de objetos
    """
    qr_det = cv2.QRCodeDetector()
    cap    = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("No se pudo abrir la camara.")
        return

    print("Camara activa.  q = salir  |  r = resetear historial")
    codigos_leidos: set[str] = set()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error leyendo frame.")
            break

        # ── 1. Deteccion QR con OpenCV ────────────────────────────────
        datos_qr, puntos, _ = qr_det.detectAndDecode(frame)

        if datos_qr and puntos is not None:
            pts = puntos[0].astype(int)
            for i in range(4):
                cv2.line(frame, tuple(pts[i]), tuple(pts[(i + 1) % 4]),
                         COLOR_QR, 2)

            x0 = pts[:, 0].min()
            y0 = pts[:, 1].min()
            cv2.putText(frame, datos_qr[:50],
                        (x0, y0 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, COLOR_QR, 2)

            if datos_qr not in codigos_leidos:
                codigos_leidos.add(datos_qr)
                print(f"\n[QR] {datos_qr}")
                if datos_qr.startswith("http://") or datos_qr.startswith("https://"):
                    print(f"  -> Abriendo: {datos_qr}")
                    webbrowser.open(datos_qr)

        # ── 2. Inferencia YOLOv8 ──────────────────────────────────────
        resultados = model(frame, conf=conf, verbose=False)[0]

        for box in resultados.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cls_id   = int(box.cls[0])
            score    = float(box.conf[0])
            etiqueta = f"{model.names[cls_id]} {score:.0%}"

            cv2.rectangle(frame, (x1, y1), (x2, y2), COLOR_YOLO, 2)
            cv2.putText(frame, etiqueta,
                        (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_YOLO, 2)

        # ── 3. HUD ────────────────────────────────────────────────────
        cv2.putText(frame, f"QRs leidos: {len(codigos_leidos)}",
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_HUD, 2)
        cv2.putText(frame, "q = salir  |  r = resetear",
                    (10, frame.shape[0] - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, COLOR_HINT, 1)

        cv2.imshow("QR + YOLO", frame)

        tecla = cv2.waitKey(1) & 0xFF
        if tecla == ord("q"):
            break
        elif tecla == ord("r"):
            codigos_leidos.clear()
            print("Historial reseteado.")

    cap.release()
    cv2.destroyAllWindows()


# ══════════════════════════════════════════════
#  ENTRENAMIENTO CON ULTRALYTICS
# ══════════════════════════════════════════════
def entrenar(
    base_weights: str = "yolov8n.pt",
    data_yaml:    str = "mi_dataset.yaml",
    epochs:       int = 50,
    img_size:     int = 640,
    batch:        int = 16,
    proyecto:     str = "runs/train",
    nombre:       str = "mi_modelo",
) -> YOLO:
    """
    Entrena YOLOv8 con Ultralytics usando tus propias imagenes.
    Parte de pesos preentrenados (transfer learning) para converger mas rapido.
    Retorna el modelo con los mejores pesos listos para inferencia.
    """
    print(f"\n{'='*55}")
    print(f"  YOLOv8 — Entrenamiento con Ultralytics")
    print(f"  Pesos base : {base_weights}")
    print(f"  Dataset    : {data_yaml}")
    print(f"  Epocas     : {epochs}  |  img_size: {img_size}  |  batch: {batch}")
    print(f"{'='*55}\n")

    model = YOLO(base_weights)

    model.train(
        data      = data_yaml,
        epochs    = epochs,
        imgsz     = img_size,
        batch     = batch,
        project   = proyecto,
        name      = nombre,
        pretrained= True,
        exist_ok  = True,
    )

    metricas = model.val()
    print(f"\nmAP50: {metricas.box.map50:.3f}  |  mAP50-95: {metricas.box.map:.3f}")
    print(f"Pesos guardados en: {proyecto}/{nombre}/weights/best.pt")

    return model


# ══════════════════════════════════════════════
#  ENTRY POINT
# ══════════════════════════════════════════════
def main() -> None:
    parser = argparse.ArgumentParser(description="Detector QR + YOLOv8")

    parser.add_argument("--train",   action="store_true",
                        help="Entrenar antes de abrir la camara")
    parser.add_argument("--data",    default="mi_dataset.yaml",
                        help="Ruta al .yaml del dataset")
    parser.add_argument("--epochs",  type=int,   default=50)
    parser.add_argument("--batch",   type=int,   default=16)
    parser.add_argument("--weights", default="yolov8n.pt",
                        help="yolov8n/s/m/l/x.pt  o  ruta a best.pt propio")
    parser.add_argument("--conf",    type=float, default=0.45,
                        help="Umbral de confianza (default 0.45)")

    args = parser.parse_args()

    if args.train:
        model = entrenar(
            base_weights = args.weights,
            data_yaml    = args.data,
            epochs       = args.epochs,
            batch        = args.batch,
        )
    else:
        print(f"Cargando modelo: {args.weights}")
        model = YOLO(args.weights)

    detectar(model, conf=args.conf)


if __name__ == "__main__":
    main()
