import mujoco
import numpy as np
import cv2
import base64
from flask import Flask, Response
import time

app = Flask(__name__)

# Load MuJoCo model
model = mujoco.MjModel.from_xml_path("./models/panda.xml")
data = mujoco.MjData(model)
renderer = mujoco.Renderer(model)

def generate_frames():
    while True:
        mujoco.mj_step(model, data)
        renderer.update_scene(data)
        img = renderer.render()
        
        # Encode as JPEG and stream
        _, buffer = cv2.imencode('.jpg', img)
        frame = base64.b64encode(buffer).decode('utf-8')

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
        time.sleep(0.033)  # ~30 FPS

@app.route('/')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    app.run(host='127.0.0.1', port=5000, debug=True)
