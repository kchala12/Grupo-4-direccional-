import io
import time
from flask import Flask, Response, render_template_string

# Intenta importar Picamera2 (recomendado, para libcamera)
try:
    from picamera2 import Picamera2
    USE_PICAMERA2 = True
except ImportError:
    import picamera
    USE_PICAMERA2 = False

app = Flask(__name__)

# ── Generador de frames ─────────────────────────────────────────────────────

def generate_frames_picamera2():
    cam = Picamera2()
    config = cam.create_video_configuration(
        main={"size": (640, 480), "format": "RGB888"}
    )
    cam.configure(config)
    cam.start()
    time.sleep(1)

    try:
        while True:
            frame_buffer = io.BytesIO()
            cam.capture_file(frame_buffer, format="jpeg")
            frame_buffer.seek(0)
            frame = frame_buffer.read()

            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            )
    finally:
        cam.stop()


def generate_frames_legacy():
    with picamera.PiCamera() as cam:
        cam.resolution = (640, 480)
        cam.framerate = 24
        time.sleep(2)  # calentamiento
        stream = io.BytesIO()

        for _ in cam.capture_continuous(stream, format="jpeg", use_video_port=True):
            stream.seek(0)
            frame = stream.read()
            stream.seek(0)
            stream.truncate()

            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            )


# ── Página HTML ─────────────────────────────────────────────────────────────

HTML_PAGE = """
<!DOCTYPE html>
<html lang="es">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Cámara IMX219 — Raspberry Pi</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            background: #111;
            color: #eee;
            font-family: sans-serif;
            display: flex;
            flex-direction: column;
            align-items: center;
            justify-content: center;
            min-height: 100vh;
            gap: 16px;
        }
        h1 { font-size: 1.2rem; color: #aaa; letter-spacing: 1px; }
        img {
            border: 2px solid #333;
            border-radius: 8px;
            max-width: 95vw;
        }
        p { font-size: 0.85rem; color: #555; }
    </style>
</head>
<body>
    <h1>📷 Cámara en vivo — IMX219</h1>
    <img src="/video_feed" alt="Stream de cámara">
    <p>Transmisión MJPEG en tiempo real</p>
</body>
</html>
"""

# ── Rutas Flask ─────────────────────────────────────────────────────────────

@app.route("/")
def index():
    return render_template_string(HTML_PAGE)


@app.route("/video_feed")
def video_feed():
    generator = generate_frames_picamera2 if USE_PICAMERA2 else generate_frames_legacy
    return Response(
        generator(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


# ── Arranque ────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    print(f"[INFO] Usando {'Picamera2' if USE_PICAMERA2 else 'picamera (legacy)'}")
    print("[INFO] Servidor en http://0.0.0.0:5000")
    print("[INFO] Abre en el navegador: http://<IP-raspberry>:5000")
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
