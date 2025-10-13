#pragma once

#include "robot_motion_interface/interface.hpp"
#include "robot_motion_interface/utils/eigen_conversion.hpp"

#include <atomic>
#include <mutex>
#include <thread>
#include <chrono>
// TODO:REMOVE
#include <iostream>

#include "robot_motion/controllers/joint_torque_controller.hpp"
#include <franka/robot.h>

namespace robot_motion_interface {


class PandaInterface : public Interface{

public:
    /**
    * @brief Construct the panda motion interface
    * @param hostname IP of the Panda
    * @param urdf_path Path to urdf
    * @param joint_names (n_joints) Names of all the joints
    * @param kp (n_joints) Proportional gains for controllers
    * @param kd (n_joints) Derivative gains for controllers
    */
    PandaInterface(std::string hostname, std::string urdf_path, std::vector<std::string> joint_names,
        Eigen::VectorXd kp, Eigen::VectorXd kd);

    /**
     * @brief Set the controller's target joint positions for ALL joints (not blocking).
     * @param q (n_joints,) Desired joint angles in radians
     */
    void set_joint_positions(Eigen::VectorXd q) override;


    /**
     * @brief Get the current joint positions and velocities in order of joint_names
     * @return (n_joints * 2,) Current joint angles in radians and joint velocities in rad/s
     */
    Eigen::VectorXd joint_state() override;

    /**
     * @brief Start the background runtime (e.g. for control loop). This is blocking.
     */
    void start_loop() override;
    

protected:


    franka::Robot robot_;
    std::unique_ptr<robot_motion::Controller> controller_;

    std::atomic<bool> control_loop_running_ =  false;
    Eigen::VectorXd control_loop_state_{Eigen::VectorXd::Zero(14)};
    std::mutex control_loop_mutex_;
    

};

} 
