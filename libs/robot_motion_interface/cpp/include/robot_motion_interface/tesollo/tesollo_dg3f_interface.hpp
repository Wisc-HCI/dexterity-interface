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
#include "tesollo_communication.hpp"
#include <franka/robot.h>
#include <franka/exception.h>


namespace robot_motion_interface {


class TesolloDg3fInterface : public Interface{

public:
    /**
    * @brief Construct the panda motion interface
    * @param ip IP of the Panda
    * @param port Port of robot
    * @param joint_names (n_joints) Names of all the joints
    * @param kp (n_joints) Proportional gains for controllers
    * @param kd (n_joints) Derivative gains for controllers
    * @param control_loop_rate Frequency to run control loop (hz). Default: 500 hz.
    */
    TesolloDg3fInterface(std::string ip, int port,  std::vector<std::string> joint_names,
    Eigen::VectorXd kp, Eigen::VectorXd kd, int control_loop_rate = 500);

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
     * @brief Start the background runtime (e.g. for control loop). This is NOT blocking.
     */
    void start_loop() override;
    

protected:


    std::unique_ptr<robot_motion::Controller> controller_;

    std::unique_ptr<tesollo::TesolloCommunication> tesollo_client_;
    

};

} 
