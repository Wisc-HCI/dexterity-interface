#include "robot_motion_interface/panda_interface.hpp"


#include <iostream>
#include <cmath>


int main() {

    std::string ip = "192.168.4.3";


    std::string urdf_path ="../robot_description/ros/bimanual_arms.urdf";    
    std::vector<std::string> joint_names = {"right_panda_joint1", "right_panda_joint2", "right_panda_joint3", "right_panda_joint4", 
        "right_panda_joint5" ,"right_panda_joint6", "right_panda_joint7"};
    Eigen::VectorXd kp(7); kp << 40.0, 40.0, 40.0, 40.0, 25.0, 20.0, 10.0;
    Eigen::VectorXd kd(7); kd << 20.0, 20.0, 20.0, 20.0, 7.5, 15.0, 5.0;
    Eigen::VectorXd home_pos(7); home_pos << 0.0, -M_PI/4, 0.0, -3*M_PI/4, 0.0, M_PI/2, M_PI/4;

    robot_motion_interface::PandaInterface panda = robot_motion_interface::PandaInterface(ip, urdf_path, joint_names, kp, kd, home_pos);
    std::cout << "Initialized Panda Interface" << std::endl;

    Eigen::VectorXd joint_pos(7); joint_pos << 0.0, -M_PI/3, 0.0, -3*M_PI/4, 0.0, M_PI/2, M_PI/4;
    
    panda.home(false);
    // panda.set_joint_positions(joint_pos);

    panda.start_loop();

    return 0;
}
