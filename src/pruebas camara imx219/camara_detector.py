"""
camara_detector.py  —  Detector pista v2 — OpenCV puro + Flask
=====================================================================
Detecta (todo con OpenCV, sin YOLO):
  • Línea ROJA        → cinta roja derecha         → contorno rojo en pantalla
  • Línea AMARILLA    → cinta amarilla izquierda    → contorno amarillo en pantalla
  • Línea CENTRAL     → promedio entre ambas        → línea morada punteada
  • QR codes          → tienen borde azul físico    → contorno azul + texto
  • Obstáculos        → cualquier objeto de tamaño
                        significativo en zona frontal → contorno rojo + alerta

Estado global exportable:
  • OBSTACULO_DETECTADO  (bool)  → True cuando hay algo bloqueando el paso
  • ERROR_LINEA          (int|None) → desplazamiento lateral respecto al centro

Cámara:
  • Raspberry Pi  → Picamera2 (IMX219, cable CSI)
  • PC / laptop   → webcam USB (detección automática)

Uso:
    python camara_detector.py
Acceso:
    http://127.0.0.1:5000
    http://192.168.0.103:5000
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
C_LINEA_ROJA     = (0,   0,   220)   # rojo       → contorno línea roja
C_LINEA_AMARILLA = (0,   210, 210)   # amarillo   → contorno línea amarilla
C_CENTRO         = (200,   0, 200)   # morado     → línea central
C_QR             = (255,   0,   0)   # azul       → contorno QR
C_OBSTACULO      = (0,   0,   255)   # rojo vivo  → contorno obstáculo
C_ALERTA         = (0,   0,   255)   # rojo vivo  → texto alerta
C_HUD            = (255, 255, 255)   # blanco     → HUD cámara


# ══════════════════════════════════════════════
#  PARÁMETROS DE DETECCIÓN DE OBSTÁCULOS
# ══════════════════════════════════════════════
# Fracción del ancho del frame que define la "zona de peligro" central
ZONA_OBSTACULO_ANCHO   = 0.6   # 60 % central del frame
# Fracción del alto del frame: solo miramos la mitad superior (zona frontal)
ZONA_OBSTACULO_ALTO    = 0.5   # mitad superior
# Área mínima (px²) de un contorno para considerarlo obstáculo real
AREA_MIN_OBSTACULO     = 3000
# Porcentaje del ancho de la zona que debe ocupar el obstáculo para frenar
UMBRAL_BLOQUEO         = 0.30  # 30 % del ancho de zona → STOP


# ══════════════════════════════════════════════
#  ESTADO GLOBAL
# ══════════════════════════════════════════════
ultimo_frame: bytes | None = None
frame_lock   = threading.Lock()

codigos_leidos: set[str] = set()
codigos_lock = threading.Lock()

# Estado exportable para integración futura con motores
estado = {
    "obstaculo_detectado": False,   # True → mandar STOP a motores
    "error_linea": None,            # desplazamiento lateral en px
}
estado_lock = threading.Lock()


# ══════════════════════════════════════════════
#  1. DETECCIÓN DE LÍNEAS (rojo / amarillo)
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
    Detecta línea ROJA y AMARILLA en la mitad inferior del frame.
    Dibuja contornos, centroides y línea guía central.
    Retorna (frame, error_lateral).
    """
    h, w   = frame.shape[:2]
    oy     = h // 2
    roi    = frame[oy:, :]
    hsv    = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))

    # Máscara ROJA (el rojo en HSV ocupa dos rangos)
    mask_r1 = cv2.inRange(hsv, np.array([0,   120,  70]), np.array([10,  255, 255]))
    mask_r2 = cv2.inRange(hsv, np.array([170, 120,  70]), np.array([180, 255, 255]))
    mask_r  = cv2.bitwise_or(mask_r1, mask_r2)
    mask_r  = cv2.morphologyEx(mask_r, cv2.MORPH_CLOSE, kernel)

    # Máscara AMARILLA
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
#  2. DETECCIÓN DE OBSTÁCULOS (cualquier objeto)
# ══════════════════════════════════════════════
def detectar_y_dibujar_obstaculos(frame: np.ndarray) -> tuple[np.ndarray, bool]:
    """
    Detecta cualquier obstáculo físico en la zona frontal del robot,
    independientemente del color.

    Estrategia:
      1. Recortar solo la zona frontal (mitad superior, franja central).
      2. Convertir a escala de grises y aplicar umbral adaptativo para
         separar objetos del fondo de pista.
      3. Filtrar contornos por área mínima y posición horizontal.
      4. Si el obstáculo supera el umbral de bloqueo → alerta STOP.

    Retorna (frame_anotado, obstaculo_bloqueando).
    """
    h, w = frame.shape[:2]

    # ── Definir zona de análisis ────────────────────────────────────────
    zona_y2 = int(h * ZONA_OBSTACULO_ALTO)          # límite inferior de la zona
    zona_x1 = int(w * (1 - ZONA_OBSTACULO_ANCHO) / 2)
    zona_x2 = int(w * (1 + ZONA_OBSTACULO_ANCHO) / 2)
    ancho_zona = zona_x2 - zona_x1

    roi = frame[0:zona_y2, zona_x1:zona_x2]

    # Dibujar zona de vigilancia en el frame
    cv2.rectangle(frame, (zona_x1, 0), (zona_x2, zona_y2), (80, 80, 80), 1)
    cv2.putText(frame, "ZONA FRONTAL", (zona_x1 + 4, 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (80, 80, 80), 1)

    # ── Preprocesado ────────────────────────────────────────────────────
    gris   = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur   = cv2.GaussianBlur(gris, (7, 7), 0)

    # Umbral adaptativo: extrae cualquier objeto con contraste respecto al fondo
    thresh = cv2.adaptiveThreshold(
        blur, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        blockSize=31,
        C=8
    )

    # Morfología para limpiar ruido y unir partes del mismo objeto
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN,  kernel)

    # ── Buscar contornos ────────────────────────────────────────────────
    cnts, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    obstaculo_bloqueando = False
    obstaculos_validos   = []

    for cnt in cnts:
        area = cv2.contourArea(cnt)
        if area < AREA_MIN_OBSTACULO:
            continue

        x, y, bw, bh = cv2.boundingRect(cnt)

        # Ignorar contornos que toquen el borde superior (probable fondo lejano)
        if y == 0:
            continue

        obstaculos_validos.append((cnt, x, y, bw, bh, area))

        # ¿Bloquea el paso? → el objeto ocupa buena parte del ancho de la zona
        if bw >= ancho_zona * UMBRAL_BLOQUEO:
            obstaculo_bloqueando = True

    # ── Dibujar en el frame original (offset por la ROI) ────────────────
    for cnt, x, y, bw, bh, area in obstaculos_validos:
        # Coordenadas absolutas
        ax, ay = x + zona_x1, y
        color = C_OBSTACULO if bw >= ancho_zona * UMBRAL_BLOQUEO else (0, 200, 200)

        cv2.drawContours(
            frame,
            [cnt + np.array([zona_x1, 0])],
            -1, color, 2
        )
        cv2.rectangle(frame, (ax, ay), (ax + bw, ay + bh), color, 2)

        label = f"OBJ {int(area/100)}u"
        cv2.putText(frame, label, (ax, ay - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    # ── Alerta visual si hay bloqueo ────────────────────────────────────
    if obstaculo_bloqueando:
        cv2.rectangle(frame, (0, 0), (w, h), C_ALERTA, 4)
        cv2.putText(frame, "!! OBSTACULO — STOP !!",
                    (w // 2 - 160, zona_y2 + 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, C_ALERTA, 3)

    return frame, obstaculo_bloqueando


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
      max-width: 860px; width: 100%;
      position: relative;
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
    #estado-box {
      display: flex; gap: 24px; align-items: center; flex-wrap: wrap;
    }
    .estado-item { font-size: 0.9rem; }
    .badge {
      display: inline-block; padding: 3px 10px; border-radius: 4px;
      font-weight: bold; font-size: 0.85rem;
    }
    .badge.ok   { background: #1a4d2e; color: #00e676; }
    .badge.stop { background: #4d1a1a; color: #ff5252; }
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
  <h1>🛣️ DETECTOR DE PISTA v2</h1>

  <div id="stream-container">
    <img src="/video_feed" alt="Stream de cámara">
    <div id="alerta-overlay">⚠ OBSTÁCULO DETECTADO — STOP</div>
  </div>

  <div class="panel">
    <h2>📊 ESTADO</h2>
    <div id="estado-box">
      <div class="estado-item">
        Obstáculo: <span id="badge-obs" class="badge ok">LIBRE</span>
      </div>
      <div class="estado-item">
        Error línea: <span id="error-linea">—</span>
      </div>
    </div>
  </div>

  <div class="panel">
    <h2>🎨 LEYENDA</h2>
    <div id="leyenda">
      <div class="item"><div class="dot" style="background:#dc0000"></div>Línea roja</div>
      <div class="item"><div class="dot" style="background:#00d2d2"></div>Línea amarilla</div>
      <div class="item"><div class="dot" style="background:#c800c8"></div>Centro / guía</div>
      <div class="item"><div class="dot" style="background:#0000ff"></div>QR (borde azul)</div>
      <div class="item"><div class="dot" style="background:#ff0000"></div>Obstáculo (STOP)</div>
      <div class="item"><div class="dot" style="background:#00c8c8"></div>Obstáculo (aviso)</div>
    </div>
  </div>

  <div class="panel">
    <h2>🔍 CÓDIGOS QR DETECTADOS</h2>
    <ul id="lista-qr">
      <li id="empty-msg">Esperando códigos QR...</li>
    </ul>
  </div>

  <footer>Detector pista v2 · OpenCV · Streaming MJPEG</footer>

  <script>
    async function actualizarEstado() {
      try {
        const data = await (await fetch('/estado')).json();

        // Obstáculo
        const badge = document.getElementById('badge-obs');
        const overlay = document.getElementById('alerta-overlay');
        if (data.obstaculo_detectado) {
          badge.textContent = 'STOP';
          badge.className = 'badge stop';
          overlay.style.display = 'block';
        } else {
          badge.textContent = 'LIBRE';
          badge.className = 'badge ok';
          overlay.style.display = 'none';
        }

        // Error línea
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
    actualizarEstado();
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


@app.route('/estado')
def estado_api():
    """
    Endpoint JSON para integración futura con el controlador de motores.
    Ejemplo de respuesta:
      { "obstaculo_detectado": true, "error_linea": -42 }
    """
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
        frame, error       = detectar_y_dibujar_lineas(frame)
        frame, hay_obs     = detectar_y_dibujar_obstaculos(frame)
        frame              = detectar_y_dibujar_qr(frame, qr_det)

        # Actualizar estado global
        with estado_lock:
            estado["obstaculo_detectado"] = hay_obs
            estado["error_linea"]         = error

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
    parser = argparse.ArgumentParser(description="Detector pista v2")
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--host", default="0.0.0.0")
    args = parser.parse_args()

    threading.Thread(target=hilo_camara, daemon=True).start()

    print(f"\n{'='*55}")
    print(f"  Detector Pista v2 iniciado")
    print(f"  Local  : http://127.0.0.1:{args.port}")
    print(f"  Estado : http://127.0.0.1:{args.port}/estado")
    print(f"{'='*55}\n")

    app.run(host=args.host, port=args.port, threaded=True)


if __name__ == "__main__":
    main()
