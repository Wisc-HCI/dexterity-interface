from robot_motion_interface.mujoco.mujoco_interface import MujocoInterface, MujocoControlMode

import asyncio
import threading
import http.server
import io
import numpy as np
import time
from pathlib import Path

from PIL import Image
import websockets
import websockets.exceptions
from mujoco.rendering.classic.renderer import Renderer

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


class MujocoBrowserInterface(MujocoInterface):
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

        self._renderer = None
        self._render_width = render_width
        self._render_height = render_height
        self._port = port
        self._ws_port = ws_port
        self._latest_jpeg = None
        self._jpeg_lock = threading.Lock()


    @classmethod
    def from_yaml(cls, file_path: str):
        """
        Construct from a mujoco_config YAML.

        Browser-specific YAML keys (all optional, with defaults):
            render_width (int): Render frame width in pixels. Default 1280.
            render_height (int): Render frame height in pixels. Default 720.
            port (int): HTTP port for the viewer page. Default 7000.
            ws_port (int): WebSocket port for frame streaming. Default 7001.

        For all other required keys (urdf_path, ik_settings_path, joint_names, etc.),
        see MujocoInterface.

        Args:

            file_path (str): Path to mujoco config YAML.
        Returns:
            (MujocoBrowserInterface): Initialized viewer.
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
            render_width=config.get("render_width", 1280),
            render_height=config.get("render_height", 720),
            port=config.get("port", 7000),
            ws_port=config.get("ws_port", 7001),
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
                self._step()
                step += 1
                if step % self._steps_per_render == 0:
                    rgb = self._render_frame(width=self._render_width, height=self._render_height)
                    buf = io.BytesIO()
                    Image.fromarray(rgb).save(buf, format="JPEG", quality=80)
                    with self._jpeg_lock:
                        self._latest_jpeg = buf.getvalue()

        self._loop_thread = threading.Thread(target=loop, daemon=True)
        self._loop_thread.start()


    def _render_frame(self, width: int = 1280, height: int = 720) -> np.ndarray:
        """
        Render the current simulation state to an RGB image.

        Args:
            width (int): Render width in pixels.
            height (int): Render height in pixels.
        Returns:
            (np.ndarray): (H, W, 3) RGB pixel array of the current frame.
        """
        if self._renderer is None or self._renderer.width != width or self._renderer.height != height:
            self._model.vis.global_.offwidth = max(self._model.vis.global_.offwidth, width)
            self._model.vis.global_.offheight = max(self._model.vis.global_.offheight, height)
            self._renderer = Renderer(self._model, height=height, width=width)
        camera = 0 if self._model.ncam > 0 else None
        if camera is not None:
            self._renderer.update_scene(self._data, camera=camera)
        else:
            self._renderer.update_scene(self._data)
        return self._renderer.render()
    


    def _run_ws_server(self):
        """
        Run the WebSocket server that streams JPEG frames to connected clients.

        Runs an asyncio event loop in the calling thread. Each connected client
        receives the latest rendered frame at up to 30 fps. Exits when
        _stop_event is set.
        """
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
        """
        Build and return an HTTP server that serves the browser viewer page.

        The page hosts an HTML/JS WebSocket client that connects to ws_port
        and renders incoming JPEG frames on a canvas element.

        Returns:
            (http.server.HTTPServer): Server bound to self._port, ready to call serve_forever().
        """
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

    arms = MujocoBrowserInterface.from_yaml(config_path)

    try:
        arms.start_loop()
        arms.home()
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        arms.stop_loop()
