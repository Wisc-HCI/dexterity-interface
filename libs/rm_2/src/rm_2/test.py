from rm_2 import RobotProperties, JointTorqueController
import numpy as np


    

if __name__ == "__main__":

        joint_names = ["joint1", "joint2"]



        rp = RobotProperties(joint_names)
        kp = np.ones(rp.n_joints(), dtype=np.float64) * 1.0
        kd = np.ones(rp.n_joints(), dtype=np.float64) * 1.0

        controller = JointTorqueController(rp, kp, kd)
        setpoint = np.zeros(rp.n_joints())

        controller.set_setpoint(setpoint)
        state = np.array([2.0, 3.0])
        torque = controller.step(state)
        print("TORQUE:", torque)
        