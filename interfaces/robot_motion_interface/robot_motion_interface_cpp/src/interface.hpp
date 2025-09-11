
#include "robot_motion/robot_properties/robot_properties.hpp"


using robot_motion::RobotProperties;
using robot_motion::Controller;

// Just for testing. Will turn into actual class
int main() {
    RobotProperties rp({"j0","j1","j2"});

    Eigen::VectorXd Kp(3); Kp << 100,100,100;
    Eigen::VectorXd Kd(3); Kd << 10,10,10;

    Controller ctrl(rp, Kp, Kd);
    ctrl.set_setpoint((Eigen::VectorXd(3) << 1.0, 0.5, -0.2).finished());

    Eigen::VectorXd q(3);  q  << 0.9, 0.4, -0.1;
    Eigen::VectorXd dq(3); dq << 0.02, 0.00, 0.05;
    Eigen::VectorXd state(6); state << q, dq;

    auto u = ctrl.step(state);
    std::cout << "u = " << u.transpose() << "\n";
    return 0;
}
