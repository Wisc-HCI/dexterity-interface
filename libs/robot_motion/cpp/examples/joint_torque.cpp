#include "robot_motion/robot_properties/robot_properties.hpp"
#include "robot_motion/controllers/joint_torque_controller.hpp"

#include <iostream>
#include <Eigen/Dense>



/*
 * @brief Example that demonstrates joint torque control 3 joints of a robot arm.
        It shows how to load a urdf, set the joint setpoint, and print the torque
        control output given the state.
 */
int main() {

    std::string urdf_path ="../robot_description/ros/src/robot_description/urdf/bimanual_arms.urdf";
    robot_motion::RobotProperties rp({"left_panda_joint2","left_panda_joint1","left_panda_joint3"}, urdf_path);
    
    Eigen::VectorXd Kp(3); Kp << 100,100,100;
    Eigen::VectorXd Kd(3); Kd << 10,10,10;

    robot_motion::JointTorqueController ctrl(rp, Kp, Kd, true);
    ctrl.set_setpoint((Eigen::VectorXd(3) << 1.0, 0.5, -0.2).finished());

    Eigen::VectorXd state(6); state << 0.9, 0.4, -0.1, 0.02, 0.00, 0.05;
    Eigen::VectorXd torque = ctrl.step(state);

    std::cout << "CONTROL OUTPUT: " << torque.transpose() << std::endl;

    return 0;
}
