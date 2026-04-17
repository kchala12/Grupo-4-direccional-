"""
camara_detector.py  —  Librería detector pista v2 — OpenCV puro + Flask
=====================================================================
Uso como librería:
    import camara_detector
    threading.Thread(target=camara_detector.iniciar, daemon=True).start()

    # Leer estado desde cualquier hilo:
    camara_detector.estado["error_linea"]
    camara_detector.estado["obstaculo_detectado"]

Acceso web (streaming + estado):
    http://IP:5000
"""

import threading
import time
import json
import cv2
import numpy as np
from flask import Flask, Response, render_template_string

try:
    from picamera2 import Picamera2
    PICAMERA_DISPONIBLE = True
except ImportError:
    PICAMERA_DISPONIBLE = False


# ══════════════════════════════════════════════
#  COLORES BGR
# ══════════════════════════════════════════════
C_LINEA_ROJA     = (0,   0,   220)
C_LINEA_AMARILLA = (0,   210, 210)
C_CENTRO         = (200,   0, 200)
C_QR             = (255,   0,   0)
C_OBSTACULO      = (0,   0,   255)
C_ALERTA         = (0,   0,   255)
C_HUD            = (255, 255, 255)


# ══════════════════════════════════════════════
#  PARÁMETROS DE DETECCIÓN DE OBSTÁCULOS
# ══════════════════════════════════════════════
ZONA_OBSTACULO_ANCHO = 0.6
ZONA_OBSTACULO_ALTO  = 0.5
AREA_MIN_OBSTACULO   = 1500
UMBRAL_BLOQUEO       = 0.30


# ══════════════════════════════════════════════
#  ESTADO GLOBAL (accesible desde el principal)
# ══════════════════════════════════════════════
ultimo_frame: bytes | None = None
frame_lock   = threading.Lock()

codigos_leidos: set[str] = set()
codigos_lock = threading.Lock()

estado = {
    "obstaculo_detectado": False,
    "error_linea": None,
}
estado_lock = threading.Lock()


# ══════════════════════════════════════════════
#  1. DETECCIÓN DE LÍNEAS
# ══════════════════════════════════════════════
def _centroide(mask, offset_y=0):
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
    h, w   = frame.shape[:2]
    oy     = h // 2
    roi    = frame[oy:, :]
    hsv    = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))

    mask_r1 = cv2.inRange(hsv, np.array([0,   120,  70]), np.array([10,  255, 255]))
    mask_r2 = cv2.inRange(hsv, np.array([170, 120,  70]), np.array([180, 255, 255]))
    mask_r  = cv2.bitwise_or(mask_r1, mask_r2)
    mask_r  = cv2.morphologyEx(mask_r, cv2.MORPH_CLOSE, kernel)

    mask_a = cv2.inRange(hsv, np.array([18, 80, 80]), np.array([35, 255, 255]))
    mask_a = cv2.morphologyEx(mask_a, cv2.MORPH_CLOSE, kernel)

    cx_r, cy_r, cnt_r = _centroide(mask_r, oy)
    cx_a, cy_a, cnt_a = _centroide(mask_a, oy)

    if cnt_r is not None:
        cv2.drawContours(frame, [cnt_r + np.array([0, oy])], -1, C_LINEA_ROJA, 2)
        cv2.circle(frame, (cx_r, cy_r), 7, C_LINEA_ROJA, -1)
        cv2.putText(frame, "ROJA", (cx_r + 10, cy_r),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, C_LINEA_ROJA, 2)

    if cnt_a is not None:
        cv2.drawContours(frame, [cnt_a + np.array([0, oy])], -1, C_LINEA_AMARILLA, 2)
        cv2.circle(frame, (cx_a, cy_a), 7, C_LINEA_AMARILLA, -1)
        cv2.putText(frame, "AMARILLO", (cx_a + 10, cy_a),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, C_LINEA_AMARILLA, 2)

    error = None
    if cx_r is not None and cx_a is not None:
        cx_c = (cx_r + cx_a) // 2
        cy_c = (cy_r + cy_a) // 2
        for y in range(cy_c, h, 12):
            cv2.circle(frame, (cx_c, y), 2, C_CENTRO, -1)
        cv2.circle(frame, (cx_c, cy_c), 10, C_CENTRO, -1)
        cv2.putText(frame, f"CENTRO x={cx_c}", (cx_c + 12, cy_c - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, C_CENTRO, 2)
        error = cx_c - w // 2
        signo = "→" if error > 0 else "←"
        cv2.putText(frame, f"error: {error:+d}px {signo}", (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, C_CENTRO, 2)
    elif cx_r is not None:
        cv2.putText(frame, "Solo ROJA", (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, C_LINEA_ROJA, 2)
    elif cx_a is not None:
        cv2.putText(frame, "Solo AMARILLO", (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, C_LINEA_AMARILLA, 2)
    else:
        cv2.putText(frame, "Sin lineas", (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 200), 2)

    cv2.line(frame, (w // 2, oy), (w // 2, h), (70, 70, 70), 1)
    return frame, error


# ══════════════════════════════════════════════
#  2. DETECCIÓN DE OBSTÁCULOS
# ══════════════════════════════════════════════
def detectar_y_dibujar_obstaculos(frame: np.ndarray) -> tuple[np.ndarray, bool]:
    h, w = frame.shape[:2]

    zona_y2    = int(h * ZONA_OBSTACULO_ALTO)
    zona_x1    = int(w * (1 - ZONA_OBSTACULO_ANCHO) / 2)
    zona_x2    = int(w * (1 + ZONA_OBSTACULO_ANCHO) / 2)
    ancho_zona = zona_x2 - zona_x1

    roi = frame[0:zona_y2, zona_x1:zona_x2]

    cv2.rectangle(frame, (zona_x1, 0), (zona_x2, zona_y2), (80, 80, 80), 1)
    cv2.putText(frame, "ZONA FRONTAL", (zona_x1 + 4, 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (80, 80, 80), 1)

    gris  = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur  = cv2.GaussianBlur(gris, (5, 5), 0)
    edges = cv2.Canny(blur, 30, 90)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    thresh = cv2.dilate(edges, kernel, iterations=2)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    cnts, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    obstaculo_bloqueando = False
    obstaculos_validos   = []

    for cnt in cnts:
        area = cv2.contourArea(cnt)
        if area < AREA_MIN_OBSTACULO:
            continue
        x, y, bw, bh = cv2.boundingRect(cnt)
        if y <= 3:
            continue
        obstaculos_validos.append((cnt, x, y, bw, bh, area))
        if bw >= ancho_zona * UMBRAL_BLOQUEO:
            obstaculo_bloqueando = True

    for cnt, x, y, bw, bh, area in obstaculos_validos:
        ax, ay = x + zona_x1, y
        color = C_OBSTACULO if bw >= ancho_zona * UMBRAL_BLOQUEO else (0, 200, 200)
        cv2.drawContours(frame, [cnt + np.array([zona_x1, 0])], -1, color, 2)
        cv2.rectangle(frame, (ax, ay), (ax + bw, ay + bh), color, 2)
        label = f"OBJ {int(area/100)}u"
        cv2.putText(frame, label, (ax, ay - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    if obstaculo_bloqueando:
        cv2.rectangle(frame, (0, 0), (w, h), C_ALERTA, 4)
        cv2.putText(frame, "!! OBSTACULO — STOP !!",
                    (w // 2 - 160, zona_y2 + 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, C_ALERTA, 3)

    return frame, obstaculo_bloqueando


# ══════════════════════════════════════════════
#  3. DETECCIÓN DE QR
# ══════════════════════════════════════════════
def detectar_y_dibujar_qr(frame: np.ndarray, qr_det) -> np.ndarray:
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
            codigos_leidos.add(datos_qr)
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
  <title>Detector Pista v2 — Raspberry Pi</title>
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
      max-width: 860px; width: 100%; position: relative;
    }
    #stream-container img { width: 100%; display: block; }
    #alerta-overlay {
      display: none; position: absolute; top: 0; left: 0;
      width: 100%; background: #ff000088;
      color: white; text-align: center;
      font-size: 1.2rem; font-weight: bold; padding: 6px;
      letter-spacing: 2px;
    }
    .panel {
      background: #1a1a1a; border: 1px solid #333;
      border-radius: 8px; padding: 14px 20px;
      width: 100%; max-width: 860px;
    }
    .panel h2 { font-size: 0.9rem; color: #00e6a0; margin-bottom: 10px; letter-spacing: 1px; }
    #leyenda { display: flex; gap: 18px; flex-wrap: wrap; }
    .item { display: flex; align-items: center; gap: 8px; font-size: 0.82rem; }
    .dot  { width: 13px; height: 13px; border-radius: 50%; flex-shrink: 0; }
    #estado-box { display: flex; gap: 24px; align-items: center; flex-wrap: wrap; }
    .estado-item { font-size: 0.9rem; }
    .badge { display: inline-block; padding: 3px 10px; border-radius: 4px; font-weight: bold; font-size: 0.85rem; }
    .badge.ok   { background: #1a4d2e; color: #00e676; }
    .badge.stop { background: #4d1a1a; color: #ff5252; }
    #lista-qr { list-style: none; display: flex; flex-direction: column; gap: 6px; max-height: 180px; overflow-y: auto; }
    #lista-qr li { background: #252525; border-left: 3px solid #0000ff; padding: 6px 10px; border-radius: 4px; font-size: 0.83rem; word-break: break-all; }
    #lista-qr li a { color: #58c8ff; text-decoration: none; }
    #empty-msg { color: #555; font-size: 0.83rem; background: none !important; border: none !important; }
    footer { font-size: 0.75rem; color: #444; margin-top: auto; padding-bottom: 8px; }
  </style>
</head>
<body>
  <h1>🛣️ DETECTOR DE PISTA v2</h1>
  <div id="stream-container">
    <img src="/video_feed" alt="Stream de cámara">
    <div id="alerta-overlay">⚠ OBSTÁCULO DETECTADO — STOP</div>
  </div>
  <div class="panel">
    <h2>📊 ESTADO</h2>
    <div id="estado-box">
      <div class="estado-item">Obstáculo: <span id="badge-obs" class="badge ok">LIBRE</span></div>
      <div class="estado-item">Error línea: <span id="error-linea">—</span></div>
    </div>
  </div>
  <div class="panel">
    <h2>🎨 LEYENDA</h2>
    <div id="leyenda">
      <div class="item"><div class="dot" style="background:#dc0000"></div>Línea roja</div>
      <div class="item"><div class="dot" style="background:#00d2d2"></div>Línea amarilla</div>
      <div class="item"><div class="dot" style="background:#c800c8"></div>Centro / guía</div>
      <div class="item"><div class="dot" style="background:#0000ff"></div>QR</div>
      <div class="item"><div class="dot" style="background:#ff0000"></div>Obstáculo STOP</div>
      <div class="item"><div class="dot" style="background:#00c8c8"></div>Obstáculo aviso</div>
    </div>
  </div>
  <div class="panel">
    <h2>🔍 CÓDIGOS QR DETECTADOS</h2>
    <ul id="lista-qr"><li id="empty-msg">Esperando códigos QR...</li></ul>
  </div>
  <footer>Detector pista v2 · OpenCV · Streaming MJPEG</footer>
  <script>
    async function actualizarEstado() {
      try {
        const data = await (await fetch('/estado')).json();
        const badge = document.getElementById('badge-obs');
        const overlay = document.getElementById('alerta-overlay');
        if (data.obstaculo_detectado) {
          badge.textContent = 'STOP'; badge.className = 'badge stop';
          overlay.style.display = 'block';
        } else {
          badge.textContent = 'LIBRE'; badge.className = 'badge ok';
          overlay.style.display = 'none';
        }
        const el = document.getElementById('error-linea');
        if (data.error_linea !== null) {
          const signo = data.error_linea > 0 ? '→' : '←';
          el.textContent = `${data.error_linea > 0 ? '+' : ''}${data.error_linea} px ${signo}`;
        } else {
          el.textContent = '— (sin líneas)';
        }
      } catch (_) {}
    }
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
    setInterval(actualizarEstado, 300);
    setInterval(actualizarQRs, 2000);
    actualizarEstado(); actualizarQRs();
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

@app.route('/estado')
def estado_api():
    with estado_lock:
        s = dict(estado)
    return app.response_class(
        response=json.dumps(s),
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
def _hilo_camara() -> None:
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
            frame = picam.capture_array()   # BGR directo, sin conversión
        else:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.1)
                continue

        frame, error   = detectar_y_dibujar_lineas(frame)
        frame, hay_obs = detectar_y_dibujar_obstaculos(frame)
        frame          = detectar_y_dibujar_qr(frame, qr_det)

        with estado_lock:
            estado["obstaculo_detectado"] = hay_obs
            estado["error_linea"]         = error

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
#  PUNTO DE ENTRADA COMO LIBRERÍA
# ══════════════════════════════════════════════
def iniciar(host="0.0.0.0", port=5000):
    """Llama esto desde el código principal para arrancar cámara + Flask."""
    import logging
    # Silenciar los logs de acceso de Werkzeug para no ensuciar la terminal
    log = logging.getLogger("werkzeug")
    log.setLevel(logging.ERROR)

    threading.Thread(target=_hilo_camara, daemon=True).start()
    # Dar tiempo a que el hilo de cámara capture el primer frame
    time.sleep(0.5)
    print(f"[CAMARA] Streaming en http://{host}:{port}")
    # use_reloader=False es OBLIGATORIO cuando Flask corre dentro de un hilo
    # (el reloader lanza un proceso hijo que rompe todo el programa)
    app.run(host=host, port=port, threaded=True, use_reloader=False)


# ══════════════════════════════════════════════
#  STANDALONE (python camara_detector.py)
# ══════════════════════════════════════════════
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--host", default="0.0.0.0")
    args = parser.parse_args()
    iniciar(host=args.host, port=args.port)
