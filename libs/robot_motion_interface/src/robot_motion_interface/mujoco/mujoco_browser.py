from robot_motion_interface.mujoco.mujoco_interface import MujocoInterface, MujocoControlMode

import asyncio
import threading
import http.server
import io
import mujoco
import numpy as np
import time
from pathlib import Path

from PIL import Image
import websockets
import websockets.exceptions


_HTML_TEMPLATE = """<!DOCTYPE html>
<html>
<head>
  <title>MuJoCo Viewer</title>
  <style>
    body   {{ margin: 0; background: #111; display: flex; align-items: center;
              justify-content: center; height: 100vh; }}
    canvas {{ max-width: 100%; max-height: 100vh; }}
  </style>
</head>
<body>
<canvas id="c"></canvas>
<script>
  const ws = new WebSocket("ws://" + location.hostname + ":{ws_port}");
  ws.binaryType = "blob";
  const canvas = document.getElementById("c");
  const ctx = canvas.getContext("2d");
  ws.onmessage = (e) => {{
    const url = URL.createObjectURL(e.data);
    const img = new Image();
    img.onload = () => {{
      canvas.width = img.width;
      canvas.height = img.height;
      ctx.drawImage(img, 0, 0);
      URL.revokeObjectURL(url);
    }};
    img.src = url;
  }};
</script>
</body>
</html>"""


class MujocoBrowserViewer(MujocoInterface):
    """MujocoInterface variant that streams rendered frames to a browser via WebSocket."""

    def __init__(self, *args, render_width: int = 1280, render_height: int = 720,
                 port: int = 7000, ws_port: int = 7001, **kwargs):
        """

        Args:
            render_width (int): Render frame width in pixels.
            render_height (int): Render frame height in pixels.
            port (int): HTTP port to serve the browser viewer page on.
            ws_port (int): WebSocket port for frame streaming.
            *args: Passed through to MujocoInterface.
            **kwargs: Passed through to MujocoInterface.
        """
        super().__init__(*args, **kwargs)
        self._render_width = render_width
        self._render_height = render_height
        self._port = port
        self._ws_port = ws_port
        self._latest_jpeg = None
        self._jpeg_lock = threading.Lock()


    @classmethod
    def from_yaml(cls, file_path: str, render_width: int = 1280,
                  render_height: int = 720, port: int = 7000, ws_port: int = 7001):
        """
        Construct from a mujoco_config YAML.

        Args:
            file_path (str): Path to mujoco config YAML.
            render_width (int): Render frame width in pixels.
            render_height (int): Render frame height in pixels.
            port (int): HTTP port to serve the browser viewer page on.
            ws_port (int): WebSocket port for frame streaming.
        Returns:
            (MujocoBrowserViewer): Initialized viewer.
        """
        import yaml

        with open(file_path, "r") as f:
            config = yaml.safe_load(f)

        pkg_dir = Path(__file__).resolve().parents[3]

        urdf_path = str((pkg_dir / config["urdf_path"]).resolve())
        ik_settings_path = str((pkg_dir / config["ik_settings_path"]).resolve())
        scene_path = str((pkg_dir / config["scene_path"]).resolve()) if "scene_path" in config else None

        return cls(
            urdf_path=urdf_path,
            ik_settings_path=ik_settings_path,
            joint_names=config["joint_names"],
            home_joint_positions=np.array(config["home_joint_positions"], dtype=float),
            base_frame=config["base_frame"],
            ee_frames=config["ee_frames"],
            target_tolerance=config["target_tolerance"],
            kp=np.array(config["kp"], dtype=float),
            kd=np.array(config["kd"], dtype=float),
            max_joint_delta=config["max_joint_delta"],
            control_mode=MujocoControlMode(config["control_mode"]),
            scene_path=scene_path,
            steps_per_render=config.get("steps_per_render", 5),
            render_width=render_width,
            render_height=render_height,
            port=port,
            ws_port=ws_port,
        )


    def start_loop(self):
        """
        Start the physics + render loop, HTTP page server, and WebSocket stream server.
        Prints the URL to open in a browser.
        """
        self._stop_event.clear()

        http_server = self._make_http_server()
        threading.Thread(target=http_server.serve_forever, daemon=True).start()
        threading.Thread(target=self._run_ws_server, daemon=True).start()

        print(f"MuJoCo browser viewer: http://localhost:{self._port}")

        def loop():
            step = 0
            while not self._stop_event.is_set():
                qpos = self._data.qpos[self._joint_qpos_indices]
                qvel = self._data.qvel[self._joint_dof_indices]
                self._cur_state = np.concatenate([qpos, qvel])
                joint_efforts = self._controller.step(self._cur_state)
                self._data.qfrc_applied[self._joint_dof_indices] = joint_efforts
                mujoco.mj_step(self._model, self._data)

                step += 1
                if step % self._steps_per_render == 0:
                    rgb = self.render_frame(width=self._render_width, height=self._render_height)
                    buf = io.BytesIO()
                    Image.fromarray(rgb).save(buf, format="JPEG", quality=80)
                    with self._jpeg_lock:
                        self._latest_jpeg = buf.getvalue()

        self._loop_thread = threading.Thread(target=loop, daemon=True)
        self._loop_thread.start()


    def _run_ws_server(self):
        async def handler(ws):
            try:
                while not self._stop_event.is_set():
                    with self._jpeg_lock:
                        jpeg = self._latest_jpeg
                    if jpeg is None:
                        await asyncio.sleep(0.01)
                        continue
                    await ws.send(jpeg)
                    await asyncio.sleep(1 / 30)
            except websockets.exceptions.ConnectionClosed:
                pass

        async def serve():
            async with websockets.serve(handler, "", self._ws_port):
                while not self._stop_event.is_set():
                    await asyncio.sleep(0.1)

        asyncio.run(serve())


    def _make_http_server(self):
        html = _HTML_TEMPLATE.format(ws_port=self._ws_port).encode()

        class Handler(http.server.BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_GET(self):
                if self.path == "/":
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html")
                    self.send_header("Content-Length", len(html))
                    self.end_headers()
                    self.wfile.write(html)

        return http.server.HTTPServer(("", self._port), Handler)


if __name__ == "__main__":
    pkg_dir = Path(__file__).resolve().parents[3]
    config_path = pkg_dir / "config" / "mujoco_config.yaml"

    arms = MujocoBrowserViewer.from_yaml(config_path)

    try:
        arms.start_loop()
        arms.home()
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        arms.stop_loop()
