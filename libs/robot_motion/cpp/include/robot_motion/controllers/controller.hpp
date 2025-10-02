#pragma once

#include "robot_motion/robot_properties/robot_properties.hpp"


#include <stdexcept>
#include <string>
#include <iostream> // TODO: REMOVE
#include <Eigen/Dense>

namespace robot_motion {


class Controller {

public:
    /**
    * @brief Construct a controller with proportional and derivative gains.
    * @param robot_properties RobotProperties object
    * @param kp Proportional gains
    * @param kd Derivative gains
    */
    Controller(const RobotProperties& robot_properties, const Eigen::VectorXd& kp, const Eigen::VectorXd& kd);

    /**
    * @brief Sets desired setpoint of controller
    * @param setpoint (Njoint) Desired setpoint (e.g. joint position)
    */
    void set_setpoint(const Eigen::VectorXd& setpoint);

    /**
    * @brief Resets prior setpoint to zero.
    */
    void reset();


    /**
    * @brief Compute control command. In this case, all 0s.
    * @param state Unused
    * @return (n_joints) Control output (all 0s).
    */
    Eigen::VectorXd step(const Eigen::VectorXd& state);


protected:
    Eigen::VectorXd kp_;
    Eigen::VectorXd kd_;
    Eigen::VectorXd setpoint_;
    Eigen::VectorXd prev_setpoint_;
    Eigen::VectorXd prev_state_;
    RobotProperties rp_;
};

} 
