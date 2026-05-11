import threading
import mujoco
import mujoco.viewer

spec = mujoco.MjSpec.from_file(
    "/workspace/libs/robot_description/ros/src/robot_description/urdf/bimanual_arms_local.urdf"
)
spec.compiler.balanceinertia = True
model = spec.compile()
data = mujoco.MjData(model)
lock = threading.Lock()

def viewer_thread():
    with mujoco.viewer.launch_passive(model, data) as v:
        v.cam.lookat[2] = 1.0
        v.cam.distance = 2.0
        while v.is_running():
            with lock:
                mujoco.mj_step(model, data)
            v.sync()

t = threading.Thread(target=viewer_thread, daemon=True)
t.start()

t.join()