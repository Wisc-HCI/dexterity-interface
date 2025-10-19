#include "robot_motion_interface/tesollo/tesollo_dg3f_interface.hpp"


#include <iostream>
#include <cmath>

// Need to be public bc std::signal cannot take params
robot_motion_interface::TesolloDg3fInterface* tesollo_ptr = nullptr;
volatile std::sig_atomic_t shutdown_requested = 0;

/**
 * @brief Called by std::signal to stop the tesollo loop
 */
void signal_handler(int signum) {
    if (tesollo_ptr) {
        tesollo_ptr->stop_loop();
    }
    shutdown_requested = 1;
}


int main() {
    // Handle ctrl-c shutdown 
    std::signal(SIGINT, signal_handler);

    std::string ip = "192.168.4.8";
    int port = 502;
    std::vector<std::string> joint_names = {"left_F1M1","left_F2M1","left_F3M1",
        "left_F1M2","left_F2M2","left_F3M2","left_F1M3","left_F2M3","left_F3M3",
        "left_F1M4","left_F2M4","left_F3M4"}; 
    Eigen::VectorXd kp(12); kp << 1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2, 1.2;
    Eigen::VectorXd kd(12); kd << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    robot_motion_interface::TesolloDg3fInterface tesollo = robot_motion_interface::TesolloDg3fInterface(ip, port, joint_names, kp, kd);
    tesollo_ptr = &tesollo; 

    Eigen::VectorXd joint_pos(12); joint_pos << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    tesollo.set_joint_positions(joint_pos); 

    tesollo.start_loop(); // Loops at 500 hz 

    // Wait for shutdown
    while (!shutdown_requested) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
