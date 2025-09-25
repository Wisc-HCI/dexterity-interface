#include <robot_motion/controllers/joint_torque_controller.hpp>


namespace robot_motion {

JointTorqueController::JointTorqueController(const RobotProperties& robot_properties, const Eigen::VectorXd& kp, 
    const Eigen::VectorXd& kd) : Controller(robot_properties, kp, kd){
    
    prev_setpoint_ = Eigen::VectorXd::Zero(rp_.n_joints());
    prev_state_ = Eigen::VectorXd::Zero(rp_.n_joints());

}



Eigen::VectorXd JointTorqueController::step(const Eigen::VectorXd& state) {
    
    int n = rp_.n_joints();

    // Return 0 control output when no setpoint
    if (setpoint_.size() == 0) return Eigen::VectorXd::Zero(n);

    Eigen::VectorXd cur_q = state.head(n);
    Eigen::VectorXd cur_dq = state.tail(n);

    Eigen::VectorXd e = setpoint_ - cur_q;
    Eigen::VectorXd de = -cur_dq;

    Eigen::VectorXd torque = kp_.cwiseProduct(e) + kd_.cwiseProduct(de);

    std::cout << "cur_q: " << cur_q.transpose() << std::endl << std::flush;
    std::cout << "cur_dq: " << cur_dq.transpose() << std::endl << std::flush;
    std::cout << "e: " << e.transpose() << std::endl << std::flush;
    std::cout << "de: " << de.transpose() << std::endl << std::flush;
    std::cout << "torque: " << torque.transpose() << std::endl << std::flush;

    // TODO: Gravity compensation, coriolis, friction compensation

    return torque;
}


}
