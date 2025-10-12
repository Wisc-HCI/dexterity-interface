#include "robot_motion_interface/panda_interface.hpp"

#include <iostream>

namespace robot_motion_interface {


PandaInterface::PandaInterface(std::string hostname, std::string urdf_path, std::vector<std::string> joint_names,
    Eigen::VectorXd kp, Eigen::VectorXd kd, Eigen::VectorXd home_joint_positions) 
    : robot_(hostname) {

    //  TODO: Read these from params?
    robot_.setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});

    rp_ = std::make_unique<robot_motion::RobotProperties>(joint_names, urdf_path);
    // TODO: Change Controller to accept unique pointer??
    controller_ = std::make_unique<robot_motion::JointTorqueController>(*rp_, kp, kd, false);
    
    
    home_joint_positions_ = home_joint_positions;

};


void PandaInterface::set_joint_positions(Eigen::VectorXd q){
    controller_->set_setpoint(q);
};



void PandaInterface::set_joint_positions(Eigen::VectorXd q, std::vector<std::string> joint_names, bool blocking){
    // TODO: Handle extra stuff
    set_joint_positions(q);
};



Eigen::VectorXd PandaInterface::joint_state() {
    if (control_loop_running_) {
        std::lock_guard<std::mutex> lock(this->control_loop_mutex_);
        return control_loop_state_;

    } else { 
        // This can only be used when control loop is NOT running
        franka::RobotState robot_state = robot_.readOnce();

        std::array< double, 7 > q = robot_state.q;
        std::array< double, 7 > dq = robot_state.dq;

        Eigen::VectorXd state(14);
        std::copy(q.begin(), q.end(), state.data());
        std::copy(dq.begin(), dq.end(), state.data() + 7);

        return state;
    }

};


void PandaInterface::write_joint_torques(Eigen::VectorXd tau){

};


void PandaInterface::start_loop() {
    control_loop_running_ =  true;


    std::function<franka::Torques(const franka::RobotState&, franka::Duration)> 
    callback = [this](const franka::RobotState& robot_state, franka::Duration time_step) -> franka::Torques {
        // TODO: Move some of this conversion to utils

        std::array< double, 7 > q = robot_state.q;
        std::array< double, 7 > dq = robot_state.dq;

        Eigen::VectorXd state(14);
        std::copy(q.begin(), q.end(), state.data());
        std::copy(dq.begin(), dq.end(), state.data() + 7);

        
        {  // Update shared variable within mutex lock
            std::lock_guard<std::mutex> lock(this->control_loop_mutex_);
            this->control_loop_state_ = state;
        }
        
        Eigen::VectorXd tau = this->controller_->step(state);


        std::array<double, 7> tau_array;
        std::copy(tau.data(), tau.data() + 7, tau_array.begin());

        franka::Torques torques(tau_array);
        return torques;
    };

    // Disable rate limiting and MaxCutoffFrequency since we are doing this
    // ourselves to help match real/sim
    // robot_.control(callback, false, franka::kMaxCutoffFrequency);
    robot_.control(callback); // TODO: fix
    control_loop_running_ =  false;

};


} 
