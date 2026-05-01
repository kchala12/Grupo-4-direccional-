"""
camara_detector.py — Librería detector de pista
================================================
Uso como librería:
    import camara_detector
    camara_detector.iniciar()   # arranca cámara y servidor HTTP en hilos daemon

    # Leer estado desde cualquier hilo:
    camara_detector.estado["error_linea"]
    camara_detector.estado["obstaculo_detectado"]

Streaming:
    http://IP:5000/       → página HTML
    http://IP:5000/video  → stream MJPEG
    http://IP:5000/estado → JSON con estado
"""

import threading
import time
import json
import cv2
import numpy as np
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn

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
#  PARÁMETROS
# ══════════════════════════════════════════════
ZONA_OBSTACULO_ANCHO  = 0.6
ZONA_OBSTACULO_DESDE  = 0.10   # desde dónde empieza (fracción del alto, 0=arriba)
ZONA_OBSTACULO_HASTA  = 0.35   # hasta dónde llega (fracción del alto)
AREA_MIN_OBSTACULO   = 4000
UMBRAL_BLOQUEO       = 0.50

# ══════════════════════════════════════════════
#  AJUSTE DE ZONA DE DETECCIÓN DE LÍNEAS
#  ROI_INICIO: desde dónde empieza (0.0=arriba, 1.0=abajo del frame)
#  ROI_ALTO:   qué tan grande es la zona (fracción del frame)
# ══════════════════════════════════════════════
ROI_INICIO = 0.60
ROI_ALTO   = 0.40

# ══════════════════════════════════════════════
#  PUNTO DE REFERENCIA PARA LÍNEA ÚNICA
#  Fracción del ancho donde debe estar cada línea
#  cuando solo se ve una de las dos.
#  REF_SOLO_AMARILLA: fracción desde izquierda (línea interna)
#  REF_SOLO_ROJA:     fracción desde izquierda (línea externa)
# ══════════════════════════════════════════════
REF_SOLO_AMARILLA = 0.25
REF_SOLO_ROJA     = 0.75

# ══════════════════════════════════════════════
#  ESTADO GLOBAL
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
#  DETECCIÓN DE LÍNEAS
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


def detectar_y_dibujar_lineas(frame):
    h, w   = frame.shape[:2]

    # ROI ajustable desde las variables ROI_INICIO y ROI_ALTO
    oy     = int(h * ROI_INICIO)
    oy_fin = min(h, oy + int(h * ROI_ALTO))
    roi    = frame[oy:oy_fin, :]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))

    mask_r1 = cv2.inRange(hsv, np.array([0,   120,  70]), np.array([10,  255, 255]))
    mask_r2 = cv2.inRange(hsv, np.array([170, 120,  70]), np.array([180, 255, 255]))
    mask_r  = cv2.morphologyEx(cv2.bitwise_or(mask_r1, mask_r2), cv2.MORPH_CLOSE, kernel)
    mask_a  = cv2.morphologyEx(
        cv2.inRange(hsv, np.array([18, 100, 100]), np.array([35, 255, 255])),
        cv2.MORPH_CLOSE, kernel)

    # Contornos para visualización
    cx_r, cy_r, cnt_r = _centroide(mask_r, oy)
    cx_a, cy_a, cnt_a = _centroide(mask_a, oy)

    if cnt_r is not None:
        cv2.drawContours(frame, [cnt_r + np.array([0, oy])], -1, C_LINEA_ROJA, 2)
        cv2.circle(frame, (cx_r, cy_r), 7, C_LINEA_ROJA, -1)
        cv2.putText(frame, "ROJA", (cx_r+10, cy_r), cv2.FONT_HERSHEY_SIMPLEX, 0.5, C_LINEA_ROJA, 2)

    if cnt_a is not None:
        cv2.drawContours(frame, [cnt_a + np.array([0, oy])], -1, C_LINEA_AMARILLA, 2)
        cv2.circle(frame, (cx_a, cy_a), 7, C_LINEA_AMARILLA, -1)
        cv2.putText(frame, "AMARILLO", (cx_a+10, cy_a), cv2.FONT_HERSHEY_SIMPLEX, 0.5, C_LINEA_AMARILLA, 2)

    # Calcular X de cada línea promediando franjas horizontales dentro del ROI
    roi_h = roi.shape[0]
    FRANJAS_P = (0.10, 0.25, 0.40, 0.55)  # porcentajes dentro del ROI
    franjas   = [int(roi_h * p) for p in FRANJAS_P]

    def _x_promedio_franjas(mask, franjas):
        xs = []
        for fy in franjas:
            if fy >= mask.shape[0]:
                continue
            fila    = mask[fy, :]
            pixeles = np.where(fila > 0)[0]
            if len(pixeles) > 0:
                xs.append(int(np.mean(pixeles)))
        return int(np.mean(xs)) if xs else None

    fx_r = _x_promedio_franjas(mask_r, franjas)
    fx_a = _x_promedio_franjas(mask_a, franjas)

    # Fallback: si las franjas no encuentran píxeles usar centroide global
    if fx_r is None and cx_r is not None:
        fx_r = cx_r
    if fx_a is None and cx_a is not None:
        fx_a = cx_a

    # Líneas de referencia: límite superior e inferior del área de medición
    ref_sup = oy + franjas[0]
    ref_inf = oy + franjas[-1]
    cv2.line(frame, (0, ref_sup), (w, ref_sup), (50, 50, 50), 1)
    cv2.line(frame, (0, ref_inf), (w, ref_inf), (50, 50, 50), 1)
    cv2.putText(frame, "ref sup", (4, ref_sup - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (50, 50, 50), 1)
    cv2.putText(frame, "ref inf", (4, ref_inf - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (50, 50, 50), 1)

    error = None
    if fx_r is not None and fx_a is not None:
        # Ambas líneas visibles — usar centro entre ellas
        cx_franja = (fx_r + fx_a) // 2
        cv2.circle(frame, (fx_r,      ref_inf), 6,  C_LINEA_ROJA,     -1)
        cv2.circle(frame, (fx_a,      ref_inf), 6,  C_LINEA_AMARILLA, -1)
        cv2.circle(frame, (cx_franja, ref_inf), 10, C_CENTRO,         -1)
        cv2.line(frame, (w//2, ref_inf), (cx_franja, ref_inf), C_CENTRO, 2)
        error = cx_franja - w // 2
        signo = "→" if error > 0 else "←"
        cv2.putText(frame, f"error: {error:+d}px {signo}", (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, C_CENTRO, 2)

    elif fx_a is not None:
        # Solo amarilla — mantenerla en REF_SOLO_AMARILLA
        ref_x = int(w * REF_SOLO_AMARILLA)
        cv2.circle(frame, (fx_a,  ref_inf), 6,  C_LINEA_AMARILLA, -1)
        cv2.circle(frame, (ref_x, ref_inf), 8,  (80, 80, 80),      2)   # objetivo
        cv2.line(frame, (ref_x, ref_inf), (fx_a, ref_inf), C_LINEA_AMARILLA, 2)
        error = fx_a - ref_x
        signo = "→" if error > 0 else "←"
        cv2.putText(frame, f"Solo AMARILLO  err:{error:+d}px {signo}", (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, C_LINEA_AMARILLA, 2)

    elif fx_r is not None:
        # Solo roja — mantenerla en REF_SOLO_ROJA
        ref_x = int(w * REF_SOLO_ROJA)
        cv2.circle(frame, (fx_r,  ref_inf), 6,  C_LINEA_ROJA, -1)
        cv2.circle(frame, (ref_x, ref_inf), 8,  (80, 80, 80),  2)       # objetivo
        cv2.line(frame, (ref_x, ref_inf), (fx_r, ref_inf), C_LINEA_ROJA, 2)
        error = fx_r - ref_x
        signo = "→" if error > 0 else "←"
        cv2.putText(frame, f"Solo ROJA  err:{error:+d}px {signo}", (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, C_LINEA_ROJA, 2)

    else:
        cv2.putText(frame, "Sin lineas", (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 200), 2)

    cv2.line(frame, (w//2, oy), (w//2, oy_fin), (70, 70, 70), 1)
    cv2.rectangle(frame, (0, oy), (w, oy_fin), (40, 40, 40), 1)
    return frame, error


# ══════════════════════════════════════════════
#  FONDO DE REFERENCIA (diferencia de fondo)
#  Se captura automáticamente al arrancar.
#  FONDO_FRAMES: cuántos frames promediar para el fondo
#  FONDO_UMBRAL: diferencia mínima para considerar obstáculo (0-255)
#  FONDO_AREA_MIN: área mínima del objeto detectado en px²
# ══════════════════════════════════════════════
FONDO_FRAMES   = 30
FONDO_UMBRAL   = 35
FONDO_AREA_MIN = 2000

_fondo_acum   = None   # acumulador de frames para el fondo
_fondo_ref    = None   # fondo de referencia final (float32)
_fondo_count  = 0      # frames capturados hasta ahora
_fondo_lock   = threading.Lock()


def resetear_fondo():
    """Llama esto para recapturar el fondo (útil si cambia la posición inicial)."""
    global _fondo_acum, _fondo_ref, _fondo_count
    with _fondo_lock:
        _fondo_acum  = None
        _fondo_ref   = None
        _fondo_count = 0
    print("[CAMARA] Recapturando fondo de referencia...")


# ══════════════════════════════════════════════
#  DETECCIÓN DE OBSTÁCULOS (diferencia de fondo)
# ══════════════════════════════════════════════
def detectar_y_dibujar_obstaculos(frame):
    global _fondo_acum, _fondo_ref, _fondo_count

    h, w    = frame.shape[:2]
    zona_y1 = int(h * ZONA_OBSTACULO_DESDE)
    zona_y2 = int(h * ZONA_OBSTACULO_HASTA)
    zona_x1 = int(w * (1 - ZONA_OBSTACULO_ANCHO) / 2)
    zona_x2 = int(w * (1 + ZONA_OBSTACULO_ANCHO) / 2)

    roi = frame[zona_y1:zona_y2, zona_x1:zona_x2]
    cv2.rectangle(frame, (zona_x1, zona_y1), (zona_x2, zona_y2), (80, 80, 80), 1)
    cv2.putText(frame, "ZONA FRONTAL", (zona_x1+4, zona_y1+15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (80, 80, 80), 1)

    gris_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    gris_f32 = gris_roi.astype(np.float32)

    with _fondo_lock:
        # Fase de captura de fondo
        if _fondo_count < FONDO_FRAMES:
            if _fondo_acum is None:
                _fondo_acum = gris_f32.copy()
            else:
                _fondo_acum += gris_f32
            _fondo_count += 1

            if _fondo_count == FONDO_FRAMES:
                _fondo_ref = _fondo_acum / FONDO_FRAMES
                print("[CAMARA] Fondo de referencia listo.")

            pct = int(_fondo_count / FONDO_FRAMES * 100)
            cv2.putText(frame, f"Calibrando fondo... {pct}%",
                        (zona_x1 + 4, zona_y1 + 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 200), 2)
            return frame, False

        fondo = _fondo_ref.copy()

    # Diferencia contra el fondo
    diff  = cv2.absdiff(gris_f32, fondo)
    diff8 = diff.astype(np.uint8)

    # Excluir líneas roja y amarilla
    hsv_roi      = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask_excl_r1 = cv2.inRange(hsv_roi, np.array([0,   60,  50]), np.array([10,  255, 255]))
    mask_excl_r2 = cv2.inRange(hsv_roi, np.array([160,  60,  50]), np.array([180, 255, 255]))
    mask_excl_a  = cv2.inRange(hsv_roi, np.array([18,   80,  80]), np.array([35,  255, 255]))
    mask_excl    = cv2.bitwise_or(cv2.bitwise_or(mask_excl_r1, mask_excl_r2), mask_excl_a)
    diff8        = cv2.bitwise_and(diff8, diff8, mask=cv2.bitwise_not(mask_excl))

    _, thresh = cv2.threshold(diff8, FONDO_UMBRAL, 255, cv2.THRESH_BINARY)
    kernel    = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    thresh    = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    thresh    = cv2.dilate(thresh, kernel, iterations=1)

    cnts, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    bloqueando = False
    ancho_zona = zona_x2 - zona_x1
    for cnt in cnts:
        area = cv2.contourArea(cnt)
        if area < FONDO_AREA_MIN:            continue
        x, y, bw, bh = cv2.boundingRect(cnt)
        if y <= 3:
            continue
        ax, ay = x + zona_x1, y + zona_y1
        color  = C_OBSTACULO if bw >= ancho_zona * UMBRAL_BLOQUEO else (0, 200, 200)
        if bw >= ancho_zona * UMBRAL_BLOQUEO:
            bloqueando = True
        cv2.drawContours(frame, [cnt + np.array([zona_x1, zona_y1])], -1, color, 2)
        cv2.rectangle(frame, (ax, ay), (ax+bw, ay+bh), color, 2)
        cv2.putText(frame, f"OBJ {int(area/100)}u", (ax, max(ay-6, zona_y1+10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    if bloqueando:
        cv2.rectangle(frame, (0, 0), (w, h), C_ALERTA, 4)
        cv2.putText(frame, "!! OBSTACULO — STOP !!",
                    (w//2-160, zona_y2+30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, C_ALERTA, 3)

    return frame, bloqueando


# ══════════════════════════════════════════════
#  DETECCIÓN DE QR
# ══════════════════════════════════════════════
def detectar_y_dibujar_qr(frame, qr_det):
    datos_qr, puntos, _ = qr_det.detectAndDecode(frame)
    if puntos is not None and cv2.contourArea(puntos[0]) <= 0:
        puntos = None
        datos_qr = ""
    if datos_qr and puntos is not None:
        pts = puntos[0].astype(int)
        for i in range(4):
            cv2.line(frame, tuple(pts[i]), tuple(pts[(i+1)%4]), C_QR, 3)
        x0, y0 = pts[:,0].min(), pts[:,1].min()
        label = datos_qr[:40]
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
        cv2.rectangle(frame, (x0, y0-th-8), (x0+tw+4, y0), (0,0,0), -1)
        cv2.putText(frame, label, (x0+2, y0-4), cv2.FONT_HERSHEY_SIMPLEX, 0.55, C_QR, 2)
        with codigos_lock:
            codigos_leidos.add(datos_qr)
    return frame


# ══════════════════════════════════════════════
#  HILO DE CAPTURA
# ══════════════════════════════════════════════
def _hilo_camara():
    global ultimo_frame
    qr_det = cv2.QRCodeDetector()

    if PICAMERA_DISPONIBLE:
        picam  = Picamera2()
        config = picam.create_preview_configuration(
            main={"size": (640, 480), "format": "RGB888"})
        picam.configure(config)
        picam.start()
        time.sleep(1)
        cap, label_cam = None, "IMX219 · Raspberry Pi"
        print("[CAMARA] Picamera2 (IMX219) — 640x480")
    else:
        picam = None
        cap   = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        label_cam = "Webcam · Desarrollo"
        print("[CAMARA] Webcam — 640x480")

    while True:
        if picam is not None:
            frame = picam.capture_array()
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

        cv2.putText(frame, label_cam, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, C_HUD, 2)

        ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            with frame_lock:
                ultimo_frame = buf.tobytes()


# ══════════════════════════════════════════════
#  SERVIDOR HTTP (reemplaza Flask)
# ══════════════════════════════════════════════
PAGINA_HTML = """<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="UTF-8">
<title>Detector Pista v2</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#0f0f0f;color:#e0e0e0;font-family:'Segoe UI',sans-serif;
     display:flex;flex-direction:column;align-items:center;padding:20px;gap:16px}
h1{font-size:1.4rem;color:#00e6a0;letter-spacing:2px;margin-top:8px}
#sc{border:2px solid #00e6a0;border-radius:8px;overflow:hidden;
    max-width:860px;width:100%;position:relative}
#sc img{width:100%;display:block}
#ao{display:none;position:absolute;top:0;left:0;width:100%;
    background:#ff000088;color:white;text-align:center;
    font-size:1.2rem;font-weight:bold;padding:6px;letter-spacing:2px}
.panel{background:#1a1a1a;border:1px solid #333;border-radius:8px;
       padding:14px 20px;width:100%;max-width:860px}
.panel h2{font-size:.9rem;color:#00e6a0;margin-bottom:10px}
#eb{display:flex;gap:24px;align-items:center;flex-wrap:wrap}
.badge{display:inline-block;padding:3px 10px;border-radius:4px;font-weight:bold;font-size:.85rem}
.ok{background:#1a4d2e;color:#00e676}
.stop{background:#4d1a1a;color:#ff5252}
footer{font-size:.75rem;color:#444;margin-top:auto;padding-bottom:8px}
</style>
</head>
<body>
<h1>&#128739; DETECTOR DE PISTA v2</h1>
<div id="sc">
  <img src="/video" alt="Stream">
  <div id="ao">&#9888; OBSTACULO &#8212; STOP</div>
</div>
<div class="panel">
  <h2>&#128202; ESTADO</h2>
  <div id="eb">
    <div>Obstaculo: <span id="bo" class="badge ok">LIBRE</span></div>
    <div>Error linea: <span id="el">&#8212;</span></div>
  </div>
</div>
<footer>Detector pista v2 · OpenCV</footer>
<script>
async function upd(){
  try{
    const d=await(await fetch('/estado')).json();
    const b=document.getElementById('bo');
    const o=document.getElementById('ao');
    if(d.obstaculo_detectado){b.textContent='STOP';b.className='badge stop';o.style.display='block';}
    else{b.textContent='LIBRE';b.className='badge ok';o.style.display='none';}
    const e=document.getElementById('el');
    if(d.error_linea!==null){
      const s=d.error_linea>0?'rarr;':'&larr;';
      e.textContent=(d.error_linea>0?'+':'')+d.error_linea+' px '+s;
    }else{e.textContent='sin lineas';}
  }catch(_){}
}
setInterval(upd,300);upd();
</script>
</body>
</html>"""


class _Handler(BaseHTTPRequestHandler):

    def log_message(self, format, *args):
        pass  # sin logs en terminal

    def do_GET(self):
        if self.path in ('/', '/index.html'):
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(PAGINA_HTML.encode('utf-8'))

        elif self.path == '/video':
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            try:
                while True:
                    with frame_lock:
                        f = ultimo_frame
                    if f:
                        self.wfile.write(
                            b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + f + b'\r\n')
                    time.sleep(0.03)
            except Exception:
                pass

        elif self.path == '/estado':
            with estado_lock:
                data = json.dumps(estado).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(data)

        else:
            self.send_response(404)
            self.end_headers()


class _ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


def iniciar(host="0.0.0.0", port=5000):
    """Arranca cámara y servidor HTTP en hilos daemon. No bloquea."""
    threading.Thread(target=_hilo_camara, daemon=True).start()

    def _servidor():
        srv = _ThreadedHTTPServer((host, port), _Handler)
        srv.serve_forever()

    threading.Thread(target=_servidor, daemon=True).start()
    print(f"[CAMARA] http://{host}:{port}")


# ══════════════════════════════════════════════
#  STANDALONE
# ══════════════════════════════════════════════
if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--port", type=int, default=5000)
    args = ap.parse_args()
    iniciar(host=args.host, port=args.port)
    print("Ctrl+C para salir")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
