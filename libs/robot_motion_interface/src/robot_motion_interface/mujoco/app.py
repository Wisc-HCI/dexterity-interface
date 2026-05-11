import os
import time
import io
import threading

import mujoco
from PIL import Image
from fastapi import FastAPI
from fastapi.responses import HTMLResponse, StreamingResponse

os.environ.setdefault("MUJOCO_GL", "osmesa")

MODEL_XML = """
<mujoco>
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <light pos="0 0 3"/>
    <geom type="plane" size="2 2 .1"/>
    <body pos="0 0 1">
      <joint type="free"/>
      <geom type="box" size=".2 .2 .2" rgba="0.2 0.6 1 1"/>
    </body>
  </worldbody>
</mujoco>
"""

app = FastAPI()

model = mujoco.MjModel.from_xml_string(MODEL_XML)
data = mujoco.MjData(model)
renderer = mujoco.Renderer(model, height=480, width=640)

lock = threading.Lock()
latest_jpeg = None


def simulation_loop():
    global latest_jpeg

    while True:
        with lock:
            mujoco.mj_step(model, data)
            renderer.update_scene(data)
            pixels = renderer.render()

            img = Image.fromarray(pixels)
            buf = io.BytesIO()
            img.save(buf, format="JPEG", quality=80)
            latest_jpeg = buf.getvalue()

        time.sleep(1 / 30)


@app.on_event("startup")
def start_simulation():
    thread = threading.Thread(target=simulation_loop, daemon=True)
    thread.start()


@app.get("/")
def index():
    return HTMLResponse("""
    <html>
      <head>
        <title>MuJoCo Browser Viewer</title>
        <style>
          body {
            margin: 0;
            background: #111;
            color: white;
            font-family: system-ui, sans-serif;
            display: grid;
            place-items: center;
            min-height: 100vh;
          }
          img {
            width: 80vw;
            max-width: 960px;
            border-radius: 12px;
          }
        </style>
      </head>
      <body>
        <div>
          <h2>MuJoCo Docker Browser Viewer</h2>
          <img src="/stream" />
        </div>
      </body>
    </html>
    """)


@app.get("/stream")
def stream():
    def generate():
        while True:
            with lock:
                frame = latest_jpeg

            if frame is not None:
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n" +
                    frame +
                    b"\r\n"
                )

            time.sleep(1 / 30)

    return StreamingResponse(
        generate(),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )