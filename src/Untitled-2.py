import time
import cv2
import numpy as np
from flask import Flask, Response, render_template_string, jsonify
from pyzbar.pyzbar import decode as qr_decode
import threading

try:
    from picamera2 import Picamera2
    USE_PICAMERA2 = True
except ImportError:
    USE_PICAMERA2 = False

app = Flask(__name__)

cam = None
cam_lock = threading.Lock()
latest_frame = {"data": None, "lock": threading.Lock()}

def init_camera():
    global cam
    if USE_PICAMERA2:
        cam = Picamera2()
        cam.configure(cam.create_video_configuration(
            main={"size": (640, 480), "format": "RGB888"},
            controls={"FrameDurationLimits": (33333, 33333)}
        ))
        cam.start(); time.sleep(1)
    else:
        cam = cv2.VideoCapture(0)
        for p, v in [(cv2.CAP_PROP_FRAME_WIDTH, 640),
                     (cv2.CAP_PROP_FRAME_HEIGHT, 480),
                     (cv2.CAP_PROP_FPS, 30),
                     (cv2.CAP_PROP_BUFFERSIZE, 1)]:
            cam.set(p, v)

def capture_loop():
    while True:
        with cam_lock:
            frame = cam.capture_array() if USE_PICAMERA2 else (lambda r,f: f if r else None)(*cam.read())
        if frame is not None:
            with latest_frame["lock"]:
                latest_frame["data"] = frame

def get_frame():
    with latest_frame["lock"]:
        f = latest_frame["data"]
        return f.copy() if f is not None else None

shared = {"qr_list": [], "cx_yellow": None, "cx_black": None, "error_px": None}

K3 = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
K5 = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
K7 = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))

RANGES = {
    "yellow": (np.array([18,  80,  80]), np.array([35,  255, 255])),
    "black":  (np.array([0,   0,   0]), np.array([180, 60,  80])),   # más estricto
    "bright": (np.array([0,   0,  140]), np.array([180, 40,  255])),
    "blue":   (np.array([90,  80,  80]), np.array([130, 255, 255])),
}

COL = {
    "y": (0, 220, 255), "b": (255, 100, 50),
    "c": (0, 255, 80),  "q": (0, 255, 120),
    "g": (80, 80, 80),  "w": (0, 140, 255),
}

QR_EVERY = 8
frame_n  = 0


def build_masks(roi):
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    my  = cv2.inRange(hsv, *RANGES["yellow"])
    mb  = cv2.inRange(hsv, *RANGES["black"])
    # quitar brillos del negro
    cv2.bitwise_and(mb, cv2.bitwise_not(
        cv2.inRange(hsv, *RANGES["bright"])), dst=mb)
    for m in (my, mb):
        cv2.morphologyEx(m, cv2.MORPH_OPEN,  K3, dst=m)
        cv2.morphologyEx(m, cv2.MORPH_CLOSE, K5, dst=m)
    return my, mb


def centroid(mask, oy, min_a):
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts: return None
    # tomar los 3 más grandes y unirlos — evita perder segmentos de línea
    tops = sorted(cnts, key=cv2.contourArea, reverse=True)[:3]
    total = sum(cv2.contourArea(c) for c in tops)
    if total < min_a: return None
    # centroide ponderado
    cx_acc = cy_acc = w_acc = 0
    for c in tops:
        M = cv2.moments(c)
        if not M["m00"]: continue
        a = M["m00"]
        cx_acc += int(M["m10"]/M["m00"]) * a
        cy_acc += int(M["m01"]/M["m00"]) * a
        w_acc  += a
    if not w_acc: return None
    return int(cx_acc/w_acc), int(cy_acc/w_acc) + oy, tops


def draw_all(frame, cnts, oy, col):
    for cnt in cnts:
        c = cnt + np.array([0, oy])
        ov = frame.copy()
        cv2.drawContours(ov, [c], -1, col, -1)
        cv2.addWeighted(ov, 0.25, frame, 0.75, 0, dst=frame)
        cv2.drawContours(frame, [c], -1, col, 2)


# ── Zona lejana con Canny — detecta líneas delgadas y curvas ──
def far_centroids(frame, FY, FH):
    roi  = frame[FY:FY+FH, :]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    hsv  = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # Amarillo lejano — por color
    my = cv2.inRange(hsv, *RANGES["yellow"])
    cv2.morphologyEx(my, cv2.MORPH_CLOSE, K5, dst=my)

    # Negro lejano — Canny sobre canal L (mejor contraste que gris)
    lab  = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
    L    = lab[:,:,0]
    blur = cv2.GaussianBlur(L, (5,5), 0)
    edge = cv2.Canny(blur, 30, 90)
    # quitar zonas amarillas del edge
    edge = cv2.bitwise_and(edge, cv2.bitwise_not(my))
    # quitar zonas muy brillantes
    bright = cv2.inRange(hsv, *RANGES["bright"])
    edge   = cv2.bitwise_and(edge, cv2.bitwise_not(bright))
    cv2.morphologyEx(edge, cv2.MORPH_CLOSE, K3, dst=edge)

    def cx_from_mask(mask):
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts: return None
        tops = sorted(cnts, key=cv2.contourArea, reverse=True)[:3]
        if sum(cv2.contourArea(c) for c in tops) < 100: return None
        cx_a = cy_a = wa = 0
        for c in tops:
            M = cv2.moments(c)
            if not M["m00"]: continue
            a = M["m00"]
            cx_a += int(M["m10"]/M["m00"]) * a
            cy_a += int(M["m01"]/M["m00"]) * a
            wa   += a
        return (int(cx_a/wa), int(cy_a/wa) + FY) if wa else None

    return cx_from_mask(my), cx_from_mask(edge), my, edge


def detect_lines(frame):
    h, w = frame.shape[:2]
    ref  = w // 2
    FY, FH, NY = int(h*.20), int(h*.35), int(h*.55)

    # ── Lejano ────────────────────────────────────────────────
    cyf, cbf, my_f, mb_f = far_centroids(frame, FY, FH)

    # dibujar máscaras lejanas semitransparentes
    for mask, col in ((my_f, COL["y"]), (mb_f, COL["b"])):
        colored = np.zeros_like(frame[FY:FY+FH, :])
        colored[mask > 0] = col
        cv2.addWeighted(colored, 0.3, frame[FY:FY+FH, :], 1.0, 0, dst=frame[FY:FY+FH, :])

    for pt, col in ((cyf, COL["y"]), (cbf, COL["b"])):
        if pt: cv2.circle(frame, pt, 5, col, -1)

    cv2.line(frame, (0, FY), (w, FY), COL["g"], 1, cv2.LINE_AA)
    cv2.putText(frame, "LEJOS", (4, FY-4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, COL["g"], 1)

    # ── Cercano ───────────────────────────────────────────────
    mny, mnb = build_masks(frame[NY:, :])
    rn = [centroid(m, NY, 300) for m in (mny, mnb)]
    cx_y = cx_b = None

    for r, col, lbl, key in zip(rn,
                                  (COL["y"], COL["b"]),
                                  ("AMARILLO", "NEGRO"),
                                  ("y", "b")):
        if r:
            draw_all(frame, r[2], NY, col)
            cv2.circle(frame, r[:2], 8, col, -1)
            cv2.putText(frame, lbl, (r[0]+12, r[1]+5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
            if key == "y": cx_y = r[0]
            else:          cx_b = r[0]

    # Proyección lejos→cerca
    for fp, rp, col in ((cyf, rn[0], COL["y"]), (cbf, rn[1], COL["b"])):
        if fp and rp:
            cv2.line(frame, fp, rp[:2], col, 1, cv2.LINE_AA)

    shared["cx_yellow"], shared["cx_black"] = cx_y, cx_b

    # ── Centro y error ────────────────────────────────────────
    cy_mid = int(h*.82)
    if cx_y is not None and cx_b is not None:
        mid = (cx_y + cx_b) // 2
        err = mid - ref
        shared["error_px"] = err
        cv2.line(frame, (cx_y, cy_mid), (cx_b, cy_mid), (60,60,60), 1)
        cv2.circle(frame, (mid, cy_mid), 9, COL["c"], -1)
        cv2.line(frame, (mid, cy_mid), (ref, cy_mid), (0,100,255), 2, cv2.LINE_AA)
        cv2.putText(frame, f"CENTRO e={err:+d}px", (mid+12, cy_mid-6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, COL["c"], 2)
        cv2.putText(frame, f"error: {err:+d}px", (10, 36),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, COL["c"], 2)
    else:
        shared["error_px"] = None
        if cx_y is not None or cx_b is not None:
            cv2.putText(frame, "UNA SOLA LINEA", (10, 36),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, COL["w"], 2)

    for y in range(FY, h, 14):
        cv2.line(frame, (ref, y), (ref, min(y+8,h)), (180,180,180), 1)

    return frame


def detect_qr(frame):
    h, w = frame.shape[:2]
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mb   = cv2.inRange(hsv, *RANGES["blue"])
    cv2.morphologyEx(mb, cv2.MORPH_CLOSE, K7, dst=mb)

    found = []
    for cnt in cv2.findContours(mb, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
        if cv2.contourArea(cnt) < 1200: continue
        x, y, bw, bh = cv2.boundingRect(cnt)
        x1,y1,x2,y2 = max(0,x-12),max(0,y-12),min(w,x+bw+12),min(h,y+bh+12)
        for qr in qr_decode(cv2.cvtColor(frame[y1:y2,x1:x2], cv2.COLOR_BGR2GRAY)):
            t = qr.data.decode()
            found.append(t)
            cv2.rectangle(frame, (x1,y1),(x2,y2), COL["q"], 2)
            cv2.putText(frame, t, (x1,y1-8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, COL["q"], 2)

    if not found:
        for qr in qr_decode(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)):
            t = qr.data.decode()
            found.append(t)
            cv2.polylines(frame, [np.array(qr.polygon,np.int32).reshape(-1,1,2)], True, COL["q"], 2)
            cv2.putText(frame, t, (qr.rect.left,qr.rect.top-8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, COL["q"], 2)

    if found:
        shared["qr_list"] = list(dict.fromkeys(found))
    return frame


def process_frame(frame):
    global frame_n
    frame_n += 1
    frame = detect_lines(frame)
    if frame_n % QR_EVERY == 0:
        frame = detect_qr(frame)
    if not USE_PICAMERA2:
        cv2.putText(frame, "dev", (10, frame.shape[0]-8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100,100,100), 1)
    return frame

def generate_frames():
    while True:
        frame = get_frame()
        if frame is None: time.sleep(0.01); continue
        ret, buf = cv2.imencode(".jpg", process_frame(frame),
                                [cv2.IMWRITE_JPEG_QUALITY, 72, cv2.IMWRITE_JPEG_OPTIMIZE, 1])
        if ret:
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + buf.tobytes() + b"\r\n"


HTML = """<!DOCTYPE html><html lang="es"><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Detector Líneas + QR</title><style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#0d0d0d;color:#eee;font-family:'Segoe UI',sans-serif;display:flex;flex-direction:column;align-items:center;padding:20px 12px;gap:16px}
h1{font-size:1.25rem;color:#ccc;letter-spacing:2px}
#sb{border:2px solid #1a1a1a;border-radius:10px;overflow:hidden}
#sb img{display:block;width:100%;max-width:680px}
.p{width:100%;max-width:680px;background:#111;border:1px solid #222;border-radius:10px;padding:14px 18px}
.pt{font-size:.75rem;font-weight:700;letter-spacing:2px;color:#0af;margin-bottom:10px}
.lg{display:flex;flex-wrap:wrap;gap:12px;font-size:.82rem;color:#aaa}
.lg span{display:flex;align-items:center;gap:6px}
.d{width:10px;height:10px;border-radius:50%;display:inline-block}
.st{display:grid;grid-template-columns:repeat(3,1fr);gap:10px}
.s{background:#0d0d0d;border:1px solid #222;border-radius:8px;padding:10px 14px;text-align:center}
.s label{display:block;font-size:.7rem;color:#555;letter-spacing:1px;margin-bottom:4px}
.s value{font-size:1.1rem;font-weight:700}
#ql{display:flex;flex-direction:column;gap:6px;min-height:32px}
.qi{background:#0d0d0d;border:1px solid #1e3;border-radius:6px;padding:6px 12px;font-size:.85rem;color:#0f9;font-family:monospace}
.qe{color:#444;font-size:.82rem;font-style:italic}
</style></head><body>
<h1>📷 DETECTOR DE LÍNEAS + QR</h1>
<div id="sb"><img src="/video_feed"></div>
<div class="p"><div class="pt">🎨 LEYENDA</div>
<div class="lg">
  <span><i class="d" style="background:#ff6432"></i>Negra</span>
  <span><i class="d" style="background:#00dcff"></i>Amarilla</span>
  <span><i class="d" style="background:#00ff50"></i>Centro</span>
  <span><i class="d" style="background:#00ff78"></i>QR</span>
</div></div>
<div class="p"><div class="pt">📊 ESTADO</div>
<div class="st">
  <div class="s"><label>AMARILLA X</label><value id="sy">—</value></div>
  <div class="s"><label>NEGRA X</label><value id="sb2">—</value></div>
  <div class="s"><label>ERROR</label><value id="se">—</value></div>
</div></div>
<div class="p"><div class="pt">🔲 QR DETECTADOS</div>
<div id="ql"><p class="qe">Esperando QR...</p></div></div>
<script>
setInterval(()=>{
  fetch('/status').then(r=>r.json()).then(d=>{
    document.getElementById('sy').textContent  = d.yellow!=null?d.yellow+'px':'—';
    document.getElementById('sb2').textContent = d.black!=null?d.black+'px':'—';
    document.getElementById('se').textContent  = d.error!=null?(d.error>=0?'+':'')+d.error+'px':'—';
    document.getElementById('ql').innerHTML    = d.qr_list?.length
      ?d.qr_list.map(q=>`<div class="qi">🔲 ${q}</div>`).join('')
      :'<p class="qe">Esperando QR...</p>';
  });
},400);
</script></body></html>"""


@app.route("/")
def index(): return render_template_string(HTML)

@app.route("/video_feed")
def video_feed():
    return Response(generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/status")
def status():
    return jsonify({"yellow": shared["cx_yellow"], "black": shared["cx_black"],
                    "error": shared["error_px"],   "qr_list": shared["qr_list"]})

if __name__ == "__main__":
    init_camera()
    threading.Thread(target=capture_loop, daemon=True).start()
    print("[INFO] http://0.0.0.0:5000")
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)