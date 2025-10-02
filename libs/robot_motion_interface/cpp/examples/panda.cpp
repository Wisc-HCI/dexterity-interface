#include "robot_motion_interface/panda_interface.hpp"


#include <iostream>


int main() {

    std::string ip = "192.168.4.2";
    robot_motion_interface::PandaInterface panda = robot_motion_interface::PandaInterface(ip);
    std::cout << "Initialized Panda Interface" << std::endl;

    while(true);  // Keep thread alive

    return 0;
}
