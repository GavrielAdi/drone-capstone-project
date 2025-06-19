from flask import Flask, Response, request
import threading
import cv2
import numpy as np

app = Flask(__name__)

# Shared frame dan lock untuk thread-safety
latest_frame = None
lock = threading.Lock()

# Load Haar cascade untuk face detection
face_cascade = cv2.CascadeClassifier(
    cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
)

@app.route('/upload', methods=['POST'])
def upload():
    global latest_frame
    data = request.get_data()
    img_np = np.frombuffer(data, dtype=np.uint8)
    frame = cv2.imdecode(img_np, cv2.IMREAD_COLOR)
    if frame is None:
        return "Failed to decode image", 400

    with lock:
        latest_frame = frame.copy()
    return "OK", 200

# MJPEG stream generator
def mjpeg_generator():
    while True:
        with lock:
            if latest_frame is None:
                continue
            frame = latest_frame.copy()

        # ✨ Tingkatkan brightness dan contrast
        frame = cv2.convertScaleAbs(frame, alpha=1.2, beta=20)
        # alpha = kontras, beta = brightness
        # Sesuaikan nilai ini jika terlalu terang / pucat

        # 🧠 Face detection (tetap aktif)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.3, 3)
        for (x, y, w, h) in faces:
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

        # 🔄 Encode JPEG (gunakan kualitas agak rendah untuk kecepatan)
        ret, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
        if not ret:
            continue
        chunk = jpeg.tobytes()

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + chunk + b'\r\n')

@app.route('/video')
def video_feed():
    return Response(
        mjpeg_generator(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )

@app.route('/')
def index():
    return """
    <html>
      <head><title>ESP32-CAM Live Stream</title></head>
      <body>
        <h1>Live Stream with Face Detection</h1>
        <img src="/video" width="640" />
      </body>
    </html>
    """

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, threaded=True)
