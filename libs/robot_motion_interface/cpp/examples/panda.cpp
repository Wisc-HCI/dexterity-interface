#include "robot_motion_interface/panda_interface.hpp"


#include <iostream>
#include <cmath>
#include <csignal>

// Need to be public bc std::signal cannot take params
robot_motion_interface::PandaInterface* panda_ptr = nullptr;
volatile std::sig_atomic_t shutdown_requested = 0;

/**
 * @brief Called by std::signal to stop the tesollo loop
 */
void signal_handler(int signum) {
    if (panda_ptr) {
        panda_ptr->stop_loop();
    }
    shutdown_requested = 1;
}

int main() {

    std::string ip = "192.168.4.2";


    std::string urdf_path ="../robot_description/ros/src/robot_description/urdf/bimanual_arms.urdf";    
    std::vector<std::string> joint_names = {"right_panda_joint1", "right_panda_joint2", "right_panda_joint3", "right_panda_joint4", 
        "right_panda_joint5" ,"right_panda_joint6", "right_panda_joint7"};
        
    Eigen::VectorXd kp(7); kp << 40.0, 40.0, 40.0, 40.0, 25.0, 20.0, 10.0;
    Eigen::VectorXd kd(7); kd << 20.0, 20.0, 20.0, 20.0, 7.5, 15.0, 5.0;
    

    robot_motion_interface::PandaInterface panda = robot_motion_interface::PandaInterface(ip, urdf_path, joint_names, kp, kd);
    panda_ptr = &panda; 
    
    Eigen::VectorXd home_pos(7); home_pos << 0.0, -M_PI/4, 0.0, -3*M_PI/4, 0.0, M_PI/2, M_PI/4;
    Eigen::VectorXd joint_pos(7); joint_pos << 0.0, -M_PI/3, 0.0, -3*M_PI/4, 0.0, M_PI/2, M_PI/4;
    
    panda.set_joint_positions(home_pos);  // Uncomment this to home
    // panda.set_joint_positions(joint_pos);  // Uncomment this to try different position

    panda.start_loop();

    // Wait for shutdown
    while (!shutdown_requested) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }


    return 0;
}
