#include "robot_motion_interface/panda_interface.hpp"

#include <iostream>

namespace robot_motion_interface {


PandaInterface::PandaInterface(std::string hostname, std::string urdf_path, std::vector<std::string> joint_names,
    Eigen::VectorXd kp, Eigen::VectorXd kd) : robot_(hostname), rp_(joint_names, urdf_path) {

    //  TODO: Read these from params?
    robot_.setCollisionBehavior(
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
        {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
        {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});

    controller_ = std::make_unique<robot_motion::JointTorqueController>(rp_, kp, kd, false);
    
    // TODO: REMOVE
    Eigen::VectorXd joint_pos(7); joint_pos << 0.0, -M_PI/4, 0.0, -3*M_PI/4, 0.0, M_PI/2, M_PI/4;
    set_joint_positions(joint_pos);
      
    start_loop();

};


void PandaInterface::set_joint_positions(Eigen::VectorXd q){
    controller_->set_setpoint(q);
};



void PandaInterface::set_joint_positions(Eigen::VectorXd q, std::vector<std::string> joint_names, bool blocking){
    // TODO: Handle extra stuff
    set_joint_positions(q);
};

void PandaInterface::home(bool blocking){

};


Eigen::VectorXd PandaInterface::joint_state(){

};


void PandaInterface::write_joint_torques(Eigen::VectorXd tau){

};


void PandaInterface::start_loop() {

    // TODO: Handle differnt control modes

    // TODO: Replace auto
    auto callback = [this](const franka::RobotState& robot_state, franka::Duration time_step) -> franka::Torques {
        // TODO: Move some of this conversion to utils

        std::array< double, 7 > q = robot_state.q;
        std::array< double, 7 > dq = robot_state.dq;

        Eigen::VectorXd state(14);
        std::copy(q.begin(), q.end(), state.data());
        std::copy(dq.begin(), dq.end(), state.data() + 7);
        
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

};


} 
