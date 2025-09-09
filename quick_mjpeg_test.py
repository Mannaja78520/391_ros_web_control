import cv2, threading, time
from flask import Flask, Response

app = Flask(__name__)
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

frame = None
def reader():
    global frame
    while True:
        cap.grab()
        ok, f = cap.read()
        if ok: frame = f
        else: time.sleep(0.01)
threading.Thread(target=reader, daemon=True).start()

@app.route("/snapshot")
def snapshot():
    global frame
    if frame is None:
        return "no frame yet", 503
    ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
    if not ok:
        return "encode fail", 500
    return Response(buf.tobytes(), mimetype="image/jpeg")

if __name__ == "__main__":
    app.run(host="127.0.0.1", port=5001, threaded=True)
