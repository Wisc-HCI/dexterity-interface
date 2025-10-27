#include <robot_motion/controllers/controller.hpp>

namespace robot_motion {

Controller::Controller(const RobotProperties& robot_properties, const Eigen::VectorXd& kp, 
    const Eigen::VectorXd& kd) : rp_(robot_properties) {
    kp_ = kp;
    kd_ = kd;

    if (kp_.size() != kd_.size()) {
        throw std::invalid_argument("kp and kd must be the same dimension");
    }
}

void Controller::set_setpoint(const Eigen::VectorXd& setpoint) {
    if (setpoint.size() != rp_.n_joints()) {
        throw std::invalid_argument("Setpoint dimension does not match joint size");
    }
    prev_setpoint_ = setpoint_;
    setpoint_ = setpoint;
}

void Controller::reset() {
    // TODO: CHECK IF CORRECT
    setpoint_.setZero();
    prev_setpoint_.setZero();
    prev_state_.setZero();
}

Eigen::VectorXd Controller::step(const Eigen::VectorXd& state) {
    return Eigen::VectorXd::Zero(rp_.n_joints());
}

}
