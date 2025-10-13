#include "robot_motion_interface/panda_interface.hpp"

#include <iostream>

namespace robot_motion_interface {


PandaInterface::PandaInterface(std::string hostname, std::string urdf_path, std::vector<std::string> joint_names,
    Eigen::VectorXd kp, Eigen::VectorXd kd) 
    : robot_(hostname) {

    //  TODO: Read these from params?
    robot_.setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});

    rp_ = std::make_unique<robot_motion::RobotProperties>(joint_names, urdf_path);
    controller_ = std::make_unique<robot_motion::JointTorqueController>(*rp_, kp, kd, false);
        

};


void PandaInterface::set_joint_positions(Eigen::VectorXd q){
    controller_->set_setpoint(q);
};



Eigen::VectorXd PandaInterface::joint_state() {
    if (control_loop_running_) {
        std::lock_guard<std::mutex> lock(this->control_loop_mutex_);
        return control_loop_state_;

    } else { 
        // This can only be used when control loop is NOT running
        franka::RobotState robot_state = robot_.readOnce();

        Eigen::VectorXd q = array_to_eigen(robot_state.q);
        Eigen::VectorXd dq = array_to_eigen(robot_state.dq);
        Eigen::VectorXd state(14); state << q, dq;

        return state;
    }

};



void PandaInterface::start_loop() {
    control_loop_running_ =  true;

    std::cout << "IN CONTROL V2" << std::endl; // TODO: REMOVE
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)> 
    callback = [this](const franka::RobotState& robot_state, franka::Duration time_step) -> franka::Torques {

        Eigen::VectorXd q = array_to_eigen(robot_state.q);
        Eigen::VectorXd dq = array_to_eigen(robot_state.dq);
        Eigen::VectorXd state(14); state << q, dq;

        {  // Update shared variable within mutex lock
            std::lock_guard<std::mutex> lock(this->control_loop_mutex_);
            this->control_loop_state_ = state;
        }
        
        Eigen::VectorXd tau = this->controller_->step(state);

        franka::Torques torques(eigen_to_array<double, 7>(tau));

        // Let other threads run
        std::this_thread::sleep_for(std::chrono::microseconds(350));
        return torques;
    };

    // TODO: Decide if should disable rate limiting and MaxCutoffFrequency and do this
    // ourselves to help match real/sim
    // robot_.control(callback, false, franka::kMaxCutoffFrequency);
    robot_.control(callback); // TODO: fix
    control_loop_running_ =  false;

};


} 
