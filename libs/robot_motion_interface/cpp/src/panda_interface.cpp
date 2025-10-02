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


void PandaInterface::start_loop(){

};


} 
