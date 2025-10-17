// Based off https://github.com/Tesollo-Delto/DELTO_B_ROS2/blob/93e332cf3ff009a3a593b8054308ebe253460325/delto_3f_driver/src/delto_3f_external_driver.cpp
// This is considered "external" driving of Tesollo with Real Time Control loop 

#include "robot_motion_interface/tesollo/tesollo_dg3f_interface.hpp"


namespace robot_motion_interface {


TesolloDg3fInterface::TesolloDg3fInterface(std::string ip, int port,  std::vector<std::string> joint_names,
    Eigen::VectorXd kp, Eigen::VectorXd kd, float control_loop_frequency) {
    

    rp_ = std::make_unique<robot_motion::RobotProperties>(joint_names); // Will not do Coriolis or Gravity Compensation
    controller_ = std::make_unique<robot_motion::JointTorqueController>(*rp_, kp, kd, false);
    control_loop_frequency_ =  control_loop_frequency;

    tesollo_client_ = std::make_unique<tesollo::TesolloCommunication>(ip, port);

    std::cout << "About to connect to tesollo" << std::endl;
    tesollo_client_->connect();
    std::cout << "Connected to tesollo" << std::endl;

    TesolloReceivedData received_data = tesollo_client_->get_data();

    std::cout << "JOINT" << received_data.joint[0] << received_data.joint[1] << std::endl;

    std::cout << "CURRENT"<< received_data.current[0] << received_data.current[1] << std::endl;
        

};


void TesolloDg3fInterface::set_joint_positions(Eigen::VectorXd q){
    controller_->set_setpoint(q);
};



Eigen::VectorXd TesolloDg3fInterface::joint_state() {


};



void TesolloDg3fInterface::start_loop() {
    
    // Loop at proper frequency
    std::chrono::nanoseconds duration(static_cast<int64_t>(1e9 / control_loop_frequency_));
    std::chrono::time_point<std::chrono::high_resolution_clock> next_loop_time = std::chrono::high_resolution_clock::now();
    std::vector<int> duty = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // -1000 to 1000, docs clamp -400 to 400
    // TODO: Handle  exiting gracefully
    tesollo_client_->send_duty(duty); // TODO: Make sure to only send duty when it changes

    std::cout << "JUST SENT DUTY" << std::endl;
    while (true) { 
        next_loop_time += duration;

        // std::vector<int> duty = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

        // tesollo_client_->send_duty(duty); // TODO: Make sure to only send duty when it changes
    
        std::this_thread::sleep_until(next_loop_time);
    }

};


} 
