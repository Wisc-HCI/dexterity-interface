#include "robot_motion_interface/panda_interface.hpp"


#include <iostream>
#include <cmath>


int main() {

    std::string ip = "192.168.4.2";


    std::string urdf_path ="../robot_description/ros/bimanual_arms.urdf";    
    std::vector<std::string> joint_names = {"left_panda_joint1", "left_panda_joint2", "left_panda_joint3", "left_panda_joint4", 
        "left_panda_joint5" ,"left_panda_joint6", "left_panda_joint7"};

    Eigen::VectorXd kp(7); kp << 100.0, 100.0, 100.0, 100.0, 75.0, 150.0, 50.0;
    Eigen::VectorXd kd(7); kd << 20.0, 20.0, 20.0, 20.0, 7.5, 15.0, 5.0;


    robot_motion_interface::PandaInterface panda = robot_motion_interface::PandaInterface(ip, urdf_path, joint_names, kp, kd);
    
    // Default Pose
    Eigen::VectorXd joint_pos(7); joint_pos << 0.0, -M_PI/4, 0.0, -3*M_PI/4, 0.0, M_PI/2, M_PI/4;
    panda.set_joint_positions(joint_pos);

    std::cout << "Initialized Panda Interface" << std::endl;

    while(true);  // Keep thread alive

    return 0;
}
