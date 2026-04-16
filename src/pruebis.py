"""
qr_yolo_stream.py  —  Detector pista completo — OpenCV puro + Flask
=====================================================================
Detecta (todo con OpenCV, sin YOLO):
  • Línea NEGRA      → cinta negra derecha      → contorno azul en pantalla
  • Línea AMARILLA   → cinta amarilla izquierda  → contorno amarillo en pantalla
  • Línea CENTRAL    → promedio entre ambas       → línea verde punteada
  • QR codes         → tienen borde azul físico   → contorno azul + texto
  • Carros           → fichas naranjas con número negro grabado → contorno naranja + número negro

Cámara:
  • Raspberry Pi 5   → Picamera2 (IMX219, cable CSI)
  • PC / laptop      → webcam USB (detección automática)

Instalación:
    sudo apt install -y python3-picamera2    # solo Raspberry Pi
    pip install opencv-python flask

Uso:
    python qr_yolo_stream.py

Acceso:
    http://127.0.0.1:5000          (mismo PC)
    http://192.168.0.103:5000      (desde la red)
"""

import argparse
import threading
import time
import json
import cv2
import numpy as np
from flask import Flask, Response, render_template_string

# ── Picamera2 solo en Raspberry Pi ───────────────────────────────────
try:
    from picamera2 import Picamera2
    PICAMERA_DISPONIBLE = True
except ImportError:
    PICAMERA_DISPONIBLE = False


# ══════════════════════════════════════════════
#  COLORES BGR
# ══════════════════════════════════════════════
C_LINEA_NEGRA    = (0,   200,   0)   # verde      → contorno línea negra
C_LINEA_AMARILLA = (0,   210, 210)   # amarillo   → contorno línea amarilla
C_CENTRO         = (200,   0, 200)   # morado     → línea central
C_QR             = (255,   0,   0)   # azul       → contorno QR
C_CARRO          = (0,   165, 255)   # naranja    → contorno carro
C_NUMERO         = (0,     0,   0)   # negro      → número encima del carro
C_HUD            = (255, 255, 255)   # blanco     → HUD cámara


# ══════════════════════════════════════════════
#  ESTADO GLOBAL
# ══════════════════════════════════════════════
ultimo_frame: bytes | None = None
frame_lock   = threading.Lock()
codigos_leidos: set[str] = set()
codigos_lock = threading.Lock()


# ══════════════════════════════════════════════
#  1. DETECCIÓN DE LÍNEAS (negro / amarillo)
# ══════════════════════════════════════════════
def _centroide(mask, offset_y=0):
    """Devuelve (cx, cy, contorno) del contorno más grande, o (None, None, None)."""
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None, None, None
    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < 500:
        return None, None, None
    M = cv2.moments(c)
    if M["m00"] == 0:
        return None, None, None
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"]) + offset_y
    return cx, cy, c


def detectar_y_dibujar_lineas(frame: np.ndarray):
    """
    Detecta línea negra y amarilla en la mitad inferior del frame.
    Dibuja contornos, centroides y línea guía central.
    """
    h, w   = frame.shape[:2]
    oy     = h // 2
    roi    = frame[oy:, :]
    hsv    = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))

    # Máscara negra
    mask_n = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 80, 80]))
    mask_n = cv2.morphologyEx(mask_n, cv2.MORPH_CLOSE, kernel)

    # Máscara amarilla
    mask_a = cv2.inRange(hsv, np.array([18, 80, 80]), np.array([35, 255, 255]))
    mask_a = cv2.morphologyEx(mask_a, cv2.MORPH_CLOSE, kernel)

    cx_n, cy_n, cnt_n = _centroide(mask_n, oy)
    cx_a, cy_a, cnt_a = _centroide(mask_a, oy)

    if cnt_n is not None:
        cv2.drawContours(frame, [cnt_n + np.array([0, oy])], -1, C_LINEA_NEGRA, 2)
        cv2.circle(frame, (cx_n, cy_n), 7, C_LINEA_NEGRA, -1)
        cv2.putText(frame, "NEGRO", (cx_n + 10, cy_n),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, C_LINEA_NEGRA, 2)

    if cnt_a is not None:
        cv2.drawContours(frame, [cnt_a + np.array([0, oy])], -1, C_LINEA_AMARILLA, 2)
        cv2.circle(frame, (cx_a, cy_a), 7, C_LINEA_AMARILLA, -1)
        cv2.putText(frame, "AMARILLO", (cx_a + 10, cy_a),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, C_LINEA_AMARILLA, 2)

    error = None
    if cx_n is not None and cx_a is not None:
        cx_c = (cx_n + cx_a) // 2
        cy_c = (cy_n + cy_a) // 2
        for y in range(cy_c, h, 12):
            cv2.circle(frame, (cx_c, y), 2, C_CENTRO, -1)
        cv2.circle(frame, (cx_c, cy_c), 10, C_CENTRO, -1)
        cv2.putText(frame, f"CENTRO x={cx_c}", (cx_c + 12, cy_c - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, C_CENTRO, 2)
        error = cx_c - w // 2
        signo = "→" if error > 0 else "←"
        cv2.putText(frame, f"error: {error:+d}px {signo}", (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, C_CENTRO, 2)
    elif cx_n is not None:
        cv2.putText(frame, "Solo NEGRO", (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, C_LINEA_NEGRA, 2)
    elif cx_a is not None:
        cv2.putText(frame, "Solo AMARILLO", (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, C_LINEA_AMARILLA, 2)
    else:
        cv2.putText(frame, "Sin lineas", (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 200), 2)

    cv2.line(frame, (w // 2, oy), (w // 2, h), (70, 70, 70), 1)
    return frame, error


# ══════════════════════════════════════════════
#  2. DETECCIÓN DE CARROS (fichas naranjas + número negro)
# ══════════════════════════════════════════════
def detectar_y_dibujar_carros(frame: np.ndarray) -> np.ndarray:
    """
    Detecta fichas naranjas (carros).
    Dibuja:
      • Contorno naranja grueso
      • Número negro grande centrado dentro de la ficha
      • Etiqueta 'CARRO N' encima con fondo negro
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Naranja en HSV (dos rangos)
    mask1 = cv2.inRange(hsv, np.array([0,   140, 100]), np.array([18,  255, 255]))
    mask2 = cv2.inRange(hsv, np.array([160, 140, 100]), np.array([180, 255, 255]))
    mask  = cv2.bitwise_or(mask1, mask2)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)

    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    carros   = sorted(
        [(c, cv2.contourArea(c)) for c in cnts if cv2.contourArea(c) > 800],
        key=lambda x: x[1], reverse=True
    )

    for idx, (cnt, _) in enumerate(carros, start=1):
        x, y, bw, bh = cv2.boundingRect(cnt)

        # Contorno naranja
        cv2.drawContours(frame, [cnt], -1, C_CARRO, 3)
        cv2.rectangle(frame, (x, y), (x + bw, y + bh), C_CARRO, 2)

        # Etiqueta superior con fondo negro
        label = f"CARRO {idx}"
        font  = cv2.FONT_HERSHEY_SIMPLEX
        (tw, th), bl = cv2.getTextSize(label, font, 0.6, 2)
        tx = x + bw // 2 - tw // 2
        ty = y - 8
        cv2.rectangle(frame, (tx - 3, ty - th - 3), (tx + tw + 3, ty + bl),
                      (0, 0, 0), -1)
        cv2.putText(frame, label, (tx, ty), font, 0.6, C_CARRO, 2)

        # Número negro grande centrado dentro de la ficha
        num = str(idx)
        (nw, nh), _ = cv2.getTextSize(num, font, 1.4, 4)
        nx = x + bw // 2 - nw // 2
        ny = y + bh // 2 + nh // 2
        # Sombra blanca para que resalte sobre la ficha naranja
        cv2.putText(frame, num, (nx + 2, ny + 2), font, 1.4, (255, 255, 255), 5)
        # Número en negro encima
        cv2.putText(frame, num, (nx, ny), font, 1.4, C_NUMERO, 4)

    return frame


# ══════════════════════════════════════════════
#  3. DETECCIÓN DE QR (borde azul físico)
# ══════════════════════════════════════════════
def detectar_y_dibujar_qr(frame: np.ndarray, qr_det) -> np.ndarray:
    """
    Detecta QR codes (tienen borde azul físico).
    Dibuja contorno azul + texto del QR.
    """
    datos_qr, puntos, _ = qr_det.detectAndDecode(frame)

    if datos_qr and puntos is not None:
        pts = puntos[0].astype(int)
        for i in range(4):
            cv2.line(frame, tuple(pts[i]), tuple(pts[(i + 1) % 4]), C_QR, 3)

        x0 = pts[:, 0].min()
        y0 = pts[:, 1].min()
        label = datos_qr[:40]
        (tw, th), bl = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
        cv2.rectangle(frame, (x0, y0 - th - 8), (x0 + tw + 4, y0), (0, 0, 0), -1)
        cv2.putText(frame, label, (x0 + 2, y0 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, C_QR, 2)

        with codigos_lock:
            if datos_qr not in codigos_leidos:
                codigos_leidos.add(datos_qr)
                print(f"[QR] {datos_qr}")

    return frame


# ══════════════════════════════════════════════
#  HTML
# ══════════════════════════════════════════════
PAGINA_HTML = """
<!DOCTYPE html>
<html lang="es">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Detector Pista — Raspberry Pi</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      background: #0f0f0f; color: #e0e0e0;
      font-family: 'Segoe UI', sans-serif;
      display: flex; flex-direction: column; align-items: center;
      min-height: 100vh; padding: 20px; gap: 16px;
    }
    h1 { font-size: 1.4rem; color: #00e6a0; letter-spacing: 2px; margin-top: 8px; }
    #stream-container {
      border: 2px solid #00e6a0; border-radius: 8px;
      overflow: hidden; box-shadow: 0 0 24px #00e6a044;
      max-width: 860px; width: 100%;
    }
    #stream-container img { width: 100%; display: block; }
    .panel {
      background: #1a1a1a; border: 1px solid #333;
      border-radius: 8px; padding: 14px 20px;
      width: 100%; max-width: 860px;
    }
    .panel h2 { font-size: 0.9rem; color: #00e6a0; margin-bottom: 10px; letter-spacing: 1px; }
    #leyenda { display: flex; gap: 18px; flex-wrap: wrap; }
    .item { display: flex; align-items: center; gap: 8px; font-size: 0.82rem; }
    .dot  { width: 13px; height: 13px; border-radius: 50%; flex-shrink: 0; }
    #lista-qr {
      list-style: none; display: flex; flex-direction: column;
      gap: 6px; max-height: 180px; overflow-y: auto;
    }
    #lista-qr li {
      background: #252525; border-left: 3px solid #0000ff;
      padding: 6px 10px; border-radius: 4px;
      font-size: 0.83rem; word-break: break-all;
    }
    #lista-qr li a { color: #58c8ff; text-decoration: none; }
    #lista-qr li a:hover { text-decoration: underline; }
    #empty-msg { color: #555; font-size: 0.83rem; background: none !important; border: none !important; }
    footer { font-size: 0.75rem; color: #444; margin-top: auto; padding-bottom: 8px; }
  </style>
</head>
<body>
  <h1>🛣️ DETECTOR DE PISTA</h1>

  <div id="stream-container">
    <img src="/video_feed" alt="Stream de cámara">
  </div>

  <div class="panel">
    <h2>🎨 LEYENDA</h2>
    <div id="leyenda">
      <div class="item"><div class="dot" style="background:#00c800"></div>Línea negra</div>
      <div class="item"><div class="dot" style="background:#00d2d2"></div>Línea amarilla</div>
      <div class="item"><div class="dot" style="background:#c800c8"></div>Centro / guía</div>
      <div class="item"><div class="dot" style="background:#0000ff"></div>QR (borde azul)</div>
      <div class="item"><div class="dot" style="background:#ffa500"></div>Carro (ficha naranja)</div>
    </div>
  </div>

  <div class="panel">
    <h2>🔍 CÓDIGOS QR DETECTADOS</h2>
    <ul id="lista-qr">
      <li id="empty-msg">Esperando códigos QR...</li>
    </ul>
  </div>

  <footer>192.168.0.103 · Detector pista OpenCV · Streaming MJPEG</footer>

  <script>
    async function actualizarQRs() {
      try {
        const data = await (await fetch('/qr_list')).json();
        const ul = document.getElementById('lista-qr');
        ul.innerHTML = '';
        if (data.length === 0) {
          ul.innerHTML = '<li id="empty-msg">Esperando códigos QR...</li>';
        } else {
          data.forEach(c => {
            const li = document.createElement('li');
            li.innerHTML = (c.startsWith('http://') || c.startsWith('https://'))
              ? `<a href="${c}" target="_blank">${c}</a>` : c;
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
#  FLASK
# ══════════════════════════════════════════════
app = Flask(__name__)


@app.route('/')
def index():
    return render_template_string(PAGINA_HTML)


@app.route('/video_feed')
def video_feed():
    return Response(_generar_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/qr_list')
def qr_list():
    with codigos_lock:
        lista = list(codigos_leidos)
    return app.response_class(
        response=json.dumps(lista, ensure_ascii=False),
        mimetype='application/json'
    )


def _generar_frames():
    while True:
        with frame_lock:
            f = ultimo_frame
        if f is None:
            time.sleep(0.05)
            continue
        yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + f + b'\r\n'
        time.sleep(0.03)


# ══════════════════════════════════════════════
#  HILO DE CAPTURA
# ══════════════════════════════════════════════
def hilo_camara() -> None:
    global ultimo_frame

    qr_det = cv2.QRCodeDetector()

    if PICAMERA_DISPONIBLE:
        picam  = Picamera2()
        config = picam.create_preview_configuration(
            main={"size": (640, 480), "format": "RGB888"})
        picam.configure(config)
        picam.start()
        time.sleep(1)
        cap       = None
        label_cam = "IMX219 · Raspberry Pi"
        print("[CAMARA] Picamera2 (IMX219) — 640x480")
    else:
        picam = None
        cap   = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("[ERROR] No se encontró cámara.")
            return
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        label_cam = "Webcam PC · Modo desarrollo"
        print("[CAMARA] Webcam PC — 640x480")

    while True:
        if picam is not None:
            frame = cv2.cvtColor(picam.capture_array(), cv2.COLOR_RGB2BGR)
        else:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.1)
                continue

        # ── Las 3 detecciones ─────────────────────────────────────────
        frame, _ = detectar_y_dibujar_lineas(frame)   # líneas pista
        frame    = detectar_y_dibujar_carros(frame)    # fichas naranjas
        frame    = detectar_y_dibujar_qr(frame, qr_det)  # QR borde azul

        # HUD
        cv2.putText(frame, label_cam,
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, C_HUD, 2)

        ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            with frame_lock:
                ultimo_frame = buf.tobytes()

    if picam:
        picam.stop()
    if cap:
        cap.release()


# ══════════════════════════════════════════════
#  ENTRY POINT
# ══════════════════════════════════════════════
def main() -> None:
    parser = argparse.ArgumentParser(description="Detector pista completo")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--host", default="0.0.0.0")
    args = parser.parse_args()

    threading.Thread(target=hilo_camara, daemon=True).start()

    print(f"\n{'='*55}")
    print(f"  Servidor web iniciado")
    print(f"  Local  : http://127.0.0.1:{args.port}")
    print(f"  Red    : http://192.168.0.103:{args.port}")
    print(f"{'='*55}\n")

    app.run(host=args.host, port=args.port, threaded=True)


if __name__ == "__main__":
    main()