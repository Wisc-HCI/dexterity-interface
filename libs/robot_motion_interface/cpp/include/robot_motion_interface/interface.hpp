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
    * @brief Construct a robot motion interface.
    */
    Interface() = default;
    virtual ~Interface() = default;

    /**
     * @brief Set the controller's target joint positions for ALL joints.
     * @param q (n_joints,) Desired joint angles in radians.
     */
    virtual void set_joint_positions(Eigen::VectorXd q) = 0;


    /**
     * @brief Set the controller's target joint positions at selected joints.
     * @param q (n_joints_names,) Desired joint angles in radians.
     * @param joint_names (n_joints_names,)  Names of joints to command in the same order as q.
     * @param blocking  If true, the call returns only after the controller achieves the target. 
            If false, returns after queuing the request.
     */
    virtual void set_joint_positions(Eigen::VectorXd q, std::vector<std::string> joint_names, bool blocking) = 0;

    /**
     * @brief Move the robot to the predefined home configuration.
     * @param blocking (bool): If true, the call returns only after the controller homes. 
            If false, returns after queuing the home request.
     */
    virtual void home(bool blocking);

    /**
     * @brief Get the current joint positions and velocities in order of joint_names.
     * @return (n_joints * 2,) Current joint angles in radians and joint velocities in rad/s.
     */
    virtual Eigen::VectorXd joint_state() = 0;

    /**
     * @brief Start the background runtime (e.g. for control loop).
     */
    virtual void start_loop() = 0;

    // TODO: Reset of interface


protected:

    /**
     * @brief Writes torque commands directly to motor.
     * @param tau (n_joints,) Commanded joint torques [N·m].
     */
    virtual void write_joint_torques(Eigen::VectorXd tau) = 0;



    std::unique_ptr<robot_motion::RobotProperties> rp_;
    Eigen::VectorXd home_joint_positions_;
};

} 

