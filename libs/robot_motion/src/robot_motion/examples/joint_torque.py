from robot_motion import RobotProperties, JointTorqueController
import numpy as np


def main():
    joint_names = ["joint1", "joint2"]
    
    rp = RobotProperties(joint_names)

    kp = np.ones(rp.n_joints(), dtype=np.float64) * 1.0
    kd = np.ones(rp.n_joints(), dtype=np.float64) * 1.0
    setpoint = np.zeros(rp.n_joints())
    state = np.array([2.0, 3.0])

    # False for no gravity compensation
    controller = JointTorqueController(rp, kp, kd, False)
    controller.set_setpoint(setpoint)
    torque = controller.step(state)

    print("TORQUE:", torque)

if __name__ == "__main__":
    main()


        