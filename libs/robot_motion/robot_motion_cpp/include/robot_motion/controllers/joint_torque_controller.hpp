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
    JointTorqueController(const RobotProperties& robot_properties, const Eigen::VectorXd& kp, const Eigen::VectorXd& kd);




    /**
    * @brief Compute control command. In this case, all 0s.
    * @param state (2*n_joints) Joint positions (rad) and velocities (rad/s).
    * @return (n_joints) Control output (all 0s).
    */
    Eigen::VectorXd step(const Eigen::VectorXd& state);


};

} 
