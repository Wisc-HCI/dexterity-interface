#include "robot_motion_interface/panda_interface.hpp"

namespace robot_motion_interface {


PandaInterface::PandaInterface(std::string hostname, std::string urdf_path, std::vector<std::string> joint_names,
    Eigen::VectorXd kp, Eigen::VectorXd kd) : robot_(hostname) {

    //  TODO: Read these from params?
    robot_.setCollisionBehavior(
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});

    //   std::string urdf_path ="../robot_description/ros/bimanual_arms.urdf";
    //   robot_motion::RobotProperties rp_({"left_panda_joint2","left_panda_joint1","left_panda_joint3"}, urdf_path);
      
    //   robot_motion::JointTorqueController ctrl(rp, Kp, Kd, true);
    //   ctrl.set_setpoint((Eigen::VectorXd(3) << 1.0, 0.5, -0.2).finished());
  
    //   Eigen::VectorXd state(6); state << 0.9, 0.4, -0.1, 0.02, 0.00, 0.05;
    //   Eigen::VectorXd torque = ctrl.step(state);
  

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
