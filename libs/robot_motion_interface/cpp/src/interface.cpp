
#include "robot_motion_interface/interface.hpp"
#include <string>
#include <filesystem>


// Just for testing. Will turn into actual class
int main() {

    std::string urdf_path ="../robot_description/ros/bimanual_arms.urdf";
    robot_motion::RobotProperties rp({"left_panda_joint2","left_panda_joint1","left_panda_joint3"}, urdf_path);
    
    Eigen::VectorXd Kp(3); Kp << 100,100,100;
    Eigen::VectorXd Kd(3); Kd << 10,10,10;

    robot_motion::JointTorqueController ctrl(rp, Kp, Kd, true);
    ctrl.set_setpoint((Eigen::VectorXd(3) << 1.0, 0.5, -0.2).finished());

    Eigen::VectorXd state(6); state << 0.9, 0.4, -0.1, 0.02, 0.00, 0.05;
    Eigen::VectorXd torque = ctrl.step(state);

    std::cout << "CONTROL OUTPUT: " << torque << std::endl;

    return 0;
}
