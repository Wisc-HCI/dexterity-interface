#pragma once

#include "robot_motion/robot_properties/robot_properties.hpp"

#include <Eigen/Dense>
#include <stdexcept>
#include <string>
#include <iostream> // TODO: REMOVE

namespace robot_motion {


class Controller {

public:
    /**
    * @brief Construct a controller with proportional and derivative gains.
    * @param robot_properties RobotProperties object
    * @param kp Proportional gain matrix
    * @param kd Derivative gain matrix
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
    * @brief Compute control command.
    * @param state (n_joints * X) State vector (e.g. q;dq)
    * @return (n_joints) Control output (e.g., torques)
    */
    virtual Eigen::VectorXd step(const Eigen::VectorXd& state) const;


protected:
    Eigen::VectorXd kp_;
    Eigen::VectorXd kd_;
    Eigen::VectorXd setpoint_;
    Eigen::VectorXd prev_setpoint_;
    Eigen::VectorXd prev_state_;
    RobotProperties rp_;
};

} 
