#pragma once

#include "robot_motion/controllers/controller.hpp"


namespace robot_motion {



class JointTorqueController : public Controller {

public:
    /**
    * @brief Construct a controller with proportional and derivative gains.
    * @param robot_properties RobotProperties object
    * @param kp Proportional gains
    * @param kd Derivative gains
    */
    JointTorqueController(const RobotProperties& robot_properties, const Eigen::VectorXd& kp, const Eigen::VectorXd& kd, bool gravity_compensation);




    /**
    * @brief Compute output torque with a "special" PD controller:
         tau = C(q, dq) * q + G(q) + kp * (setpoint_q - q) + kd * (-dq)
         Where C is the Coriolis Matrix and G is the Gravity vector (which can be toggled off).
         Based the robot dynamics model.
    * @param state (2*n_joints) Joint positions (rad) and velocities (rad/s).
    * @return (n_joints) Control output torque.
    */
    Eigen::VectorXd step(const Eigen::VectorXd& state) override;


private:
    bool gravity_compensation_;
};

} 
