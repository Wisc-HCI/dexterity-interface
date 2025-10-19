#pragma once

#include "robot_motion_interface/interface.hpp"
#include "robot_motion_interface/utils/eigen_conversion.hpp"

#include <atomic>
#include <mutex>
#include <thread>
#include <chrono>

#include "robot_motion/controllers/joint_torque_controller.hpp"
#include "tesollo_communication.hpp"



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
    * @param control_loop_frequency Frequency to run control loop (hz). Default: 500 hz
    */
    TesolloDg3fInterface(std::string ip, int port,  std::vector<std::string> joint_names,
        const Eigen::VectorXd& kp, const Eigen::VectorXd& kd, double control_loop_rate = 500.0);

    /**
     * @brief Set the controller's target joint positions for ALL joints (not blocking)
     * @param q (n_joints,) Desired joint angles in radians
     */
    void set_joint_positions(const Eigen::VectorXd& q) override;


    /**
     * @brief Get the current joint positions and velocities in order of joint_names
     * @return (n_joints * 2,) Current joint angles in radians and joint velocities in rad/s
     */
    Eigen::VectorXd joint_state() override;

    /**
     * @brief Start the background runtime (e.g. for control loop). This is blocking
     */
    void start_loop() override;
    
    /**
     * @brief Stop the background runtime (e.g. for control loop)
     */
    void stop_loop();

protected:

    /**
    * @brief Read the current joint positions from the motors
    * @return (n_joints,) Current joint positions in rads
    */
    Eigen::VectorXd _read_joint_position();

    /**
    * @brief Write duty cycle commands to the motors
    * @param duty (n_joints,) Duty cycle values for each motor  (-1000 to 1000)
    */
    void _write_duty(const Eigen::VectorXi& duty);

    /**
    * @brief Convert desired joint torques to corresponding duty cycle
    * @param torque (n_joints,) Desired joint torques in N·m.
    * @return (n_joints,) Equivalent duty cycle commands in the range [-1000, 1000].
    */
    Eigen::VectorXi _torque_to_duty(const Eigen::VectorXd& torque);


    std::unique_ptr<robot_motion::Controller> controller_;
    std::unique_ptr<tesollo::TesolloCommunication> tesollo_client_;
    double control_loop_frequency_;
    std::atomic<bool> run_loop_{false};
    Eigen::VectorXd control_loop_joint_state_;
    std::mutex control_loop_mutex_;

};



} 
