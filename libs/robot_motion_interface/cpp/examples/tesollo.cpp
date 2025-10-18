#include "robot_motion_interface/tesollo/tesollo_dg3f_interface.hpp"


#include <iostream>
#include <cmath>


int main() {

    std::string ip = "192.168.4.8";
    int port = 502;
    std::vector<std::string> joint_names = {"left_F1M1","left_F2M1","left_F3M1",
        "left_F1M2","left_F2M2","left_F3M2","left_F1M3","left_F2M3","left_F3M3",
        "left_F1M4","left_F2M4","left_F3M4"}; 
                
    Eigen::VectorXd kp(12); kp << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    Eigen::VectorXd kd(12); kd << 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0;
    

    robot_motion_interface::TesolloDg3fInterface tesollo = robot_motion_interface::TesolloDg3fInterface(ip, port, joint_names, kp, kd);
    std::cout << "Initialized Tesollo Interface" << std::endl;
    
    // Eigen::VectorXd home_pos(7); home_pos << 0.0, -M_PI/4, 0.0, -3*M_PI/4, 0.0, M_PI/2, M_PI/4;
    // Eigen::VectorXd joint_pos(7); joint_pos << 0.0, -M_PI/3, 0.0, -3*M_PI/4, 0.0, M_PI/2, M_PI/4;
    
    // tesollo.set_joint_positions(home_pos);  // Uncomment this to home

    tesollo.start_loop(); // Loops at 500 hz blocking

    // while(1); // Prevent thread from exiting

    return 0;
}
