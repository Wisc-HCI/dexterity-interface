#include "robot_motion_interface/panda_interface.hpp"

namespace robot_motion_interface {


PandaInterface::PandaInterface(std::string hostname)
    : robot_(hostname) {

    //  TODO: Read these from params?
    robot_.setCollisionBehavior(
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});

    start_loop();

};


void PandaInterface::set_joint_positions(Eigen::VectorXd q){

};



void PandaInterface::set_joint_positions(Eigen::VectorXd q, std::vector<std::string> joint_names, bool blocking){

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

        std::array<double, 7> tau = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
        franka::Torques torques(tau);
        return torques;
    };

    // Disable rate limiting and MaxCutoffFrequency since we are doing this
    // ourselves to help match real/sim
    robot_.control(callback, false, franka::kMaxCutoffFrequency);

};


} 
