#pragma once

#include "robot_motion/robot_properties/robot_properties.hpp"
#include "robot_motion/controllers/controller.hpp"

#include <vector>
#include <string>

#include <Eigen/Dense>


namespace robot_motion_interface {


class Interface {

public:
    /**
    * @brief Construct a robot motion interface. This is a simplified
        version of the Python interface bc it is designed to be wrapped by it.
    */
    Interface() = default;
    virtual ~Interface() = default;

    /**
     * @brief Set the controller's target joint positions for ALL joints (not blocking).
     * @param q (n_joints,) Desired joint angles in radians.
     */
    virtual void set_joint_positions(Eigen::VectorXd q) = 0;


    /**
     * @brief Get the current joint positions and velocities in order of joint_names.
     * @return (n_joints * 2,) Current joint angles in radians and joint velocities in rad/s.
     */
    virtual Eigen::VectorXd joint_state() = 0;

    /**
     * @brief Start the background runtime (e.g. for control loop).
     */
    virtual void start_loop() = 0;


protected:

    std::unique_ptr<robot_motion::RobotProperties> rp_;

};

} 

