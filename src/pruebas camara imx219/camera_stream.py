import io
import time
import cv2
import numpy as np
from flask import Flask, Response, render_template_string, jsonify, request
from pyzbar.pyzbar import decode as qr_decode
import threading
import os

try:
    from picamera2 import Picamera2
    USE_PICAMERA2 = True
except ImportError:
    USE_PICAMERA2 = False

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

app = Flask(__name__)

modes = {"stream": True, "qr": False, "yolo": False, "custom": False}

yolo_model = None
custom_model = None
CUSTOM_MODEL_PATH = "custom_model.pt"

def load_models():
    global yolo_model, custom_model
    if not YOLO_AVAILABLE:
        return
    print("[INFO] Cargando YOLOv8n...")
    yolo_model = YOLO("yolov8n.pt")

    print("[INFO] YOLOv8n listo.")
    if os.path.exists(CUSTOM_MODEL_PATH):
        custom_model = YOLO(CUSTOM_MODEL_PATH)
        print("[INFO] Modelo personalizado listo.")

threading.Thread(target=load_models, daemon=True).start()

cam = None
cam_lock = threading.Lock()

def init_camera():
    global cam
    if USE_PICAMERA2:
        cam = Picamera2()
        config = cam.create_video_configuration(main={"size": (640, 480), "format": "RGB888"})
        cam.configure(config)
        cam.start()
        time.sleep(1)
    else:
        cam = cv2.VideoCapture(0)

def get_frame():
    with cam_lock:
        if USE_PICAMERA2:
            frame = cam.capture_array()
            
        else:
            ret, frame = cam.read()
            if not ret:
                return None
    return frame

last_qr = "—"

def process_frame(frame):
    global last_qr
    if modes["qr"]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        for qr in qr_decode(gray):
            pts = np.array(qr.polygon, np.int32).reshape((-1, 1, 2))
            cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
            last_qr = qr.data.decode("utf-8")
            cv2.putText(frame, last_qr, (qr.rect.left, qr.rect.top - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    if modes["yolo"] and yolo_model:
        for r in yolo_model(frame, conf=0.4, imgsz=320, verbose=False):
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                name = yolo_model.names[int(box.cls[0])]
                conf = float(box.conf[0])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 100, 0), 2)
                cv2.putText(frame, f"{name} {conf:.0%}", (x1, y1-8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 100, 0), 2)
    if modes["custom"] and custom_model:
        for r in custom_model(frame, conf=0.4, verbose=False):
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                name = custom_model.names[int(box.cls[0])]
                conf = float(box.conf[0])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 200, 255), 2)
                cv2.putText(frame, f"[CUSTOM] {name} {conf:.0%}", (x1, y1-8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 2)
    return frame

def generate_frames():
    while True:
        frame = get_frame()
        if frame is None:
            time.sleep(0.05)
            continue
        frame = process_frame(frame)
        ret, buffer = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ret:
            continue
        yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + buffer.tobytes() + b"\r\n")

HTML_PAGE = """
<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Raspberry Cam</title>
<style>
* { margin:0; padding:0; box-sizing:border-box; }
body { background:#0d0d0d; color:#eee; font-family:'Segoe UI',sans-serif; display:flex; flex-direction:column; align-items:center; padding:24px 16px; gap:20px; }
h1 { font-size:1.3rem; color:#bbb; letter-spacing:2px; }
#stream-box { position:relative; border:2px solid #222; border-radius:10px; overflow:hidden; }
#stream-box img { display:block; width:100%; max-width:640px; }
.controls { display:flex; flex-wrap:wrap; gap:12px; justify-content:center; }
.btn { padding:10px 22px; border-radius:8px; border:1.5px solid #333; cursor:pointer; font-size:0.95rem; font-weight:600; transition:all .2s; background:#1e1e1e; color:#888; }
.btn.active { color:#0d0d0d; border-color:transparent; }
.btn#btn-qr.active { background:#22c55e; }
.btn#btn-yolo.active { background:#3b82f6; }
.btn#btn-custom.active { background:#f59e0b; }
#qr-result { background:#111; border:1px solid #333; border-radius:8px; padding:12px 20px; font-size:0.9rem; color:#aaa; min-width:300px; text-align:center; }
#qr-result span { color:#22c55e; font-weight:600; }
.badge { position:absolute; top:10px; left:10px; background:rgba(0,0,0,0.6); border-radius:6px; padding:4px 10px; font-size:0.75rem; color:#aaa; display:flex; gap:8px; align-items:center; }
.dot { width:8px; height:8px; border-radius:50%; background:#333; display:inline-block; }
.dot.on { background:#22c55e; animation:pulse 1.2s infinite; }
@keyframes pulse { 0%,100%{opacity:1} 50%{opacity:.4} }
</style>
</head>
<body>
<h1>📷 RASPBERRY CAM — IMX219</h1>
<div id="stream-box">
  <img src="/video_feed" alt="stream">
  <div class="badge">
    <span class="dot on"></span>LIVE
    <span class="dot" id="dot-qr"></span>QR
    <span class="dot" id="dot-yolo"></span>YOLO
    <span class="dot" id="dot-custom"></span>CUSTOM
  </div>
</div>
<div class="controls">
  <button class="btn" id="btn-qr" onclick="toggle('qr')">🔲 Detección QR</button>
  <button class="btn" id="btn-yolo" onclick="toggle('yolo')">🎯 YOLO COCO</button>
  <button class="btn" id="btn-custom" onclick="toggle('custom')">⭐ Modelo propio</button>
</div>
<div id="qr-result">Último QR: <span id="qr-data">—</span></div>
<script>
const state = {qr:false, yolo:false, custom:false};
function toggle(mode) {
  state[mode] = !state[mode];
  fetch('/toggle', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({mode, active:state[mode]})});
  document.getElementById('btn-'+mode).classList.toggle('active', state[mode]);
  document.getElementById('dot-'+mode).classList.toggle('on', state[mode]);
}
setInterval(() => {
  fetch('/qr_data').then(r=>r.json()).then(d=>{ document.getElementById('qr-data').textContent = d.data; });
}, 1000);
</script>
</body>
</html>
"""

@app.route("/")
def index():
    return render_template_string(HTML_PAGE)

@app.route("/video_feed")
def video_feed():
    return Response(generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/toggle", methods=["POST"])
def toggle():
    data = request.get_json()
    mode = data.get("mode")
    active = data.get("active", False)
    if mode in modes:
        modes[mode] = active
    return jsonify({"ok": True})

@app.route("/qr_data")
def qr_data():
    return jsonify({"data": last_qr})

if __name__ == "__main__":
    init_camera()
    print("[INFO] Servidor en http://0.0.0.0:5000")
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
