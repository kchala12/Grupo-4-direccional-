"""
prueba.py  —  Detector QR + YOLOv8  —  Picamera2 (IMX219) + Streaming Web (Flask)
===================================================================================
Camara     : Raspberry Pi Camera v2 (sensor IMX219) — cable plano CSI
Sistema    : Raspberry Pi 5 con Raspberry Pi OS Bookworm

Instalacion:
    sudo apt install -y python3-picamera2       # ya viene en Bookworm
    pip install opencv-python ultralytics flask

Verificar que la camara esta activa:
    rpicam-hello --timeout 2000

Uso:
    # Deteccion en tiempo real con streaming web
    python prueba.py

    # Con pesos propios ya entrenados
    python prueba.py --weights runs/train/mi_modelo/weights/best.pt

    # Entrenar y luego iniciar el stream
    python prueba.py --train --data mi_dataset.yaml --epochs 50

Acceso desde el navegador (misma red):
    http://192.168.0.103:5000
"""

import argparse
import threading
import time
import cv2
import numpy as np
from flask import Flask, Response, render_template_string
from ultralytics import YOLO
from picamera2 import Picamera2


# ──────────────────────────────────────────────
# Colores BGR
# ──────────────────────────────────────────────
COLOR_QR   = (0, 230, 160)
COLOR_YOLO = (0, 165, 255)
COLOR_HUD  = (255, 255, 255)
COLOR_HINT = (160, 160, 160)


# ──────────────────────────────────────────────
# Estado global compartido entre hilos
# ──────────────────────────────────────────────
ultimo_frame: bytes | None = None
frame_lock = threading.Lock()
codigos_leidos: set[str] = set()
codigos_lock = threading.Lock()


# ══════════════════════════════════════════════
#  HTML DE LA PAGINA WEB
# ══════════════════════════════════════════════
PAGINA_HTML = """
<!DOCTYPE html>
<html lang="es">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>QR + YOLO — Raspberry Pi</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      background: #0f0f0f;
      color: #e0e0e0;
      font-family: 'Segoe UI', sans-serif;
      display: flex;
      flex-direction: column;
      align-items: center;
      min-height: 100vh;
      padding: 20px;
      gap: 16px;
    }
    h1 {
      font-size: 1.4rem;
      color: #00e6a0;
      letter-spacing: 2px;
      margin-top: 8px;
    }
    #stream-container {
      position: relative;
      border: 2px solid #00e6a0;
      border-radius: 8px;
      overflow: hidden;
      box-shadow: 0 0 24px #00e6a044;
      max-width: 860px;
      width: 100%;
    }
    #stream-container img {
      width: 100%;
      display: block;
    }
    #panel {
      background: #1a1a1a;
      border: 1px solid #333;
      border-radius: 8px;
      padding: 16px 20px;
      width: 100%;
      max-width: 860px;
    }
    #panel h2 {
      font-size: 0.95rem;
      color: #00e6a0;
      margin-bottom: 10px;
      letter-spacing: 1px;
    }
    #lista-qr {
      list-style: none;
      display: flex;
      flex-direction: column;
      gap: 6px;
      max-height: 200px;
      overflow-y: auto;
    }
    #lista-qr li {
      background: #252525;
      border-left: 3px solid #00e6a0;
      padding: 6px 10px;
      border-radius: 4px;
      font-size: 0.85rem;
      word-break: break-all;
    }
    #lista-qr li a {
      color: #58c8ff;
      text-decoration: none;
    }
    #lista-qr li a:hover { text-decoration: underline; }
    #empty-msg {
      color: #555;
      font-size: 0.85rem;
    }
    footer {
      font-size: 0.75rem;
      color: #444;
      margin-top: auto;
      padding-bottom: 8px;
    }
  </style>
</head>
<body>
  <h1>📷 QR + YOLO DETECTOR</h1>

  <div id="stream-container">
    <img src="/video_feed" alt="Stream de camara">
  </div>

  <div id="panel">
    <h2>🔍 CÓDIGOS QR DETECTADOS</h2>
    <ul id="lista-qr">
      <li id="empty-msg">Esperando códigos QR...</li>
    </ul>
  </div>

  <footer>192.168.0.103 · QR + YOLOv8 · Streaming MJPEG</footer>

  <script>
    // Actualiza la lista de QRs cada 2 segundos
    async function actualizarQRs() {
      try {
        const res  = await fetch('/qr_list');
        const data = await res.json();
        const ul   = document.getElementById('lista-qr');
        ul.innerHTML = '';
        if (data.length === 0) {
          ul.innerHTML = '<li id="empty-msg">Esperando códigos QR...</li>';
        } else {
          data.forEach(codigo => {
            const li = document.createElement('li');
            if (codigo.startsWith('http://') || codigo.startsWith('https://')) {
              li.innerHTML = `<a href="${codigo}" target="_blank">${codigo}</a>`;
            } else {
              li.textContent = codigo;
            }
            ul.appendChild(li);
          });
        }
      } catch (_) {}
    }
    setInterval(actualizarQRs, 2000);
    actualizarQRs();
  </script>
</body>
</html>
"""


# ══════════════════════════════════════════════
#  FLASK APP
# ══════════════════════════════════════════════
app = Flask(__name__)


@app.route('/')
def index():
    return render_template_string(PAGINA_HTML)


@app.route('/video_feed')
def video_feed():
    """Endpoint MJPEG: el navegador lo carga como <img src=...>"""
    return Response(
        _generar_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )


@app.route('/qr_list')
def qr_list():
    import json
    with codigos_lock:
        lista = list(codigos_leidos)
    return app.response_class(
        response=json.dumps(lista, ensure_ascii=False),
        mimetype='application/json'
    )


def _generar_frames():
    """Generador que entrega el ultimo frame procesado como MJPEG."""
    global ultimo_frame
    while True:
        with frame_lock:
            frame_actual = ultimo_frame
        if frame_actual is None:
            time.sleep(0.05)
            continue
        yield (
            b'--frame\r\n'
            b'Content-Type: image/jpeg\r\n\r\n'
            + frame_actual +
            b'\r\n'
        )
        time.sleep(0.03)   # ~30 fps max al cliente


# ══════════════════════════════════════════════
#  HILO DE CAPTURA + DETECCION
# ══════════════════════════════════════════════
def hilo_camara(model: YOLO, conf: float = 0.45) -> None:
    """
    Corre en un hilo separado:
      • Captura frames con Picamera2 (sensor IMX219 / Pi Camera v2)
      • Aplica QRCodeDetector y YOLOv8
      • Guarda el frame anotado en 'ultimo_frame' para el stream web
    """
    global ultimo_frame

    # ── Iniciar Picamera2 ─────────────────────────────────────────────
    picam = Picamera2()
    config = picam.create_preview_configuration(
        main={"size": (640, 480), "format": "RGB888"}
    )
    picam.configure(config)
    picam.start()
    time.sleep(1)  # dar tiempo a que el sensor se estabilice

    qr_det = cv2.QRCodeDetector()
    print("[CAMARA] Picamera2 (IMX219) iniciada — 640x480 RGB888")

    while True:
        # capture_array() devuelve numpy array en RGB → convertir a BGR para OpenCV
        frame_rgb = picam.capture_array()
        frame     = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

        # ── 1. Deteccion QR ───────────────────────────────────────────
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

            with codigos_lock:
                if datos_qr not in codigos_leidos:
                    codigos_leidos.add(datos_qr)
                    print(f"[QR] {datos_qr}")

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
        with codigos_lock:
            n_qr = len(codigos_leidos)

        cv2.putText(frame, f"QRs leidos: {n_qr}",
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, COLOR_HUD, 2)
        cv2.putText(frame, "IMX219 · Streaming activo",
                    (10, frame.shape[0] - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, COLOR_HINT, 1)

        # ── 4. Codificar y guardar frame ──────────────────────────────
        ok, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
        if ok:
            with frame_lock:
                ultimo_frame = buffer.tobytes()

    picam.stop()


# ══════════════════════════════════════════════
#  ENTRENAMIENTO
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
    parser = argparse.ArgumentParser(description="Detector QR + YOLOv8 con streaming web")

    parser.add_argument("--train",   action="store_true",
                        help="Entrenar antes de iniciar el stream")
    parser.add_argument("--data",    default="mi_dataset.yaml")
    parser.add_argument("--epochs",  type=int,   default=50)
    parser.add_argument("--batch",   type=int,   default=16)
    parser.add_argument("--weights", default="yolov8n.pt",
                        help="yolov8n/s/m/l/x.pt  o  ruta a best.pt propio")
    parser.add_argument("--conf",    type=float, default=0.45)
    parser.add_argument("--port",    type=int,   default=5000,
                        help="Puerto del servidor web (default: 5000)")
    parser.add_argument("--host",    default="0.0.0.0",
                        help="Host del servidor (default: 0.0.0.0 = toda la red)")

    args = parser.parse_args()

    # ── Entrenamiento opcional ────────────────────────────────────────
    if args.train:
        model = entrenar(
            base_weights = args.weights,
            data_yaml    = args.data,
            epochs       = args.epochs,
            batch        = args.batch,
        )
    else:
        print(f"[INFO] Cargando modelo: {args.weights}")
        model = YOLO(args.weights)

    # ── Hilo de camara ────────────────────────────────────────────────
    t = threading.Thread(
        target=hilo_camara,
        args=(model, args.conf),
        daemon=True
    )
    t.start()

    # ── Servidor Flask ────────────────────────────────────────────────
    print(f"\n{'='*55}")
    print(f"  Servidor web iniciado")
    print(f"  Abre en tu navegador: http://192.168.0.103:{args.port}")
    print(f"{'='*55}\n")

    app.run(host=args.host, port=args.port, threaded=True)


if __name__ == "__main__":
    main()