#pragma once

#include "robot_motion_interface/interface.hpp"

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
     * @brief Set the controller's target joint positions for ALL joints
     * @param q (n_joints,) Desired joint angles in radians
     */
    void set_joint_positions(Eigen::VectorXd q);


    /**
     * @brief Set the controller's target joint positions at selected joints
     * @param q (n_joints_names,) Desired joint angles in radians
     * @param joint_names (n_joints_names,)  Names of joints to command in the same order as q
     * @param blocking  If true, the call returns only after the controller achieves the target 
            If false, returns after queuing the request
     */
    void set_joint_positions(Eigen::VectorXd q, std::vector<std::string> joint_names, bool blocking);

    /**
     * @brief Move the robot to the predefined home configuration
     * @param blocking (bool): If true, the call returns only after the controller homes 
            If false, returns after queuing the home request
     */
    void home(bool blocking);

    /**
     * @brief Get the current joint positions and velocities in order of joint_names
     * @return (n_joints * 2,) Current joint angles in radians and joint velocities in rad/s
     */
    Eigen::VectorXd joint_state();


    // TODO: Reset of interface


protected:

    /**
     * @brief Writes torque commands directly to motor
     * @param tau (n_joints,) Commanded joint torques [N·m]
     */
    void write_joint_torques(Eigen::VectorXd tau);

    /**
     * @brief Start the background runtime (e.g. for control loop)
     */
    void start_loop();

    franka::Robot robot_;

    // TODO: Move these to parent?
    robot_motion::RobotProperties rp_;
    std::unique_ptr<robot_motion::Controller> controller_;

};

} 
