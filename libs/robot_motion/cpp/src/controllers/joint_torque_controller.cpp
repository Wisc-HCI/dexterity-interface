#include <robot_motion/controllers/joint_torque_controller.hpp>

namespace robot_motion {

JointTorqueController::JointTorqueController(const RobotProperties& robot_properties, const Eigen::VectorXd& kp, 
    const Eigen::VectorXd& kd, bool gravity_compensation) : Controller(robot_properties, kp, kd){

    
    prev_setpoint_ = Eigen::VectorXd::Zero(rp_.n_joints());
    prev_state_ = Eigen::VectorXd::Zero(rp_.n_joints());
    gravity_compensation_ = gravity_compensation;

}



Eigen::VectorXd JointTorqueController::step(const Eigen::VectorXd& state) {
    
    int n = rp_.n_joints();
    
    // Return 0 control output when no setpoint
    if (setpoint_.size() == 0) return Eigen::VectorXd::Zero(n);

    Eigen::VectorXd q = state.head(n);
    Eigen::VectorXd dq = state.tail(n);

    Eigen::VectorXd e = setpoint_ - q;
    Eigen::VectorXd de = -dq;

    Eigen::VectorXd coriolis = rp_.coriolis(q, dq);

    Eigen::VectorXd torque = coriolis + kp_.cwiseProduct(e) + kd_.cwiseProduct(de);

    if (gravity_compensation_) {
        Eigen::VectorXd gravity = rp_.gravity(q);
        torque += gravity;
    }
    
    return torque;
}


}
