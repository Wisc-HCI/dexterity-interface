// Based offhttps://github.com/Tesollo-Delto/DELTO_B_ROS2/blob/devel/delto_3f_driver/src/system_interface.cpp
#include "robot_motion_interface/tesollo/tesollo_dg3f_interface.hpp"


namespace robot_motion_interface {


TesolloDg3fInterface::TesolloDg3fInterface(std::string ip, int port,  std::vector<std::string> joint_names,
    const Eigen::VectorXd& kp, const Eigen::VectorXd& kd, double control_loop_frequency) {
    
    rp_ = std::make_unique<robot_motion::RobotProperties>(joint_names); // Will not do Coriolis or Gravity Compensation
    controller_ = std::make_unique<robot_motion::JointTorqueController>(*rp_, kp, kd, false);
    control_loop_frequency_ =  control_loop_frequency;

    tesollo_client_ = std::make_unique<tesollo::TesolloCommunication>(ip, port);
    tesollo_client_->connect();

    control_loop_joint_state_ = Eigen::VectorXd::Zero(2 * rp_->n_joints());
        
};


void TesolloDg3fInterface::set_joint_positions(const Eigen::VectorXd& q){
    controller_->set_setpoint(q);
};


Eigen::VectorXd TesolloDg3fInterface::joint_state() {
    if (run_loop_) {
        {  // Update shared variable within mutex lock
            std::lock_guard<std::mutex> lock(this->control_loop_mutex_);
            return control_loop_joint_state_;
        }
    } else {
        // Should not be moving so velocity is 0
        Eigen::VectorXd joint_state = Eigen::VectorXd::Zero(2 * rp_->n_joints());
        joint_state.head(rp_->n_joints()) << _read_joint_position();
        return joint_state;
    }

};


void TesolloDg3fInterface::start_loop() {
    
    // Loop at proper frequency
    double dt = 1.0 / control_loop_frequency_;
    std::chrono::nanoseconds duration(static_cast<int64_t>(1e9 * dt));
    std::chrono::time_point<std::chrono::high_resolution_clock> next_loop_time = std::chrono::high_resolution_clock::now();
    
    run_loop_ = true;
    while (run_loop_) { 
        next_loop_time += duration;

        Eigen::VectorXd pos = _read_joint_position();

        {  // Update shared variable within mutex lock
            std::lock_guard<std::mutex> lock(this->control_loop_mutex_);

            Eigen::VectorXd prev_pos = control_loop_joint_state_.head(rp_->n_joints());
            
            // TODO: Decide if want to average/lowpass current velocity with last velocity
            Eigen::VectorXd vel = (pos - prev_pos) / dt;
            Eigen::VectorXd joint_state(2 * rp_->n_joints()); joint_state << pos, vel;
            control_loop_joint_state_ = joint_state;
            Eigen::VectorXd torque = controller_->step(joint_state); 
            // TODO: allow disabling coriolis so warning doesn't pop up

            Eigen::VectorXi duty = _torque_to_duty(torque);
            _write_duty(duty);
        }
        
        std::this_thread::sleep_until(next_loop_time);
    }

    _write_duty(Eigen::VectorXi::Zero(rp_->n_joints())); // Stop movement

};

void TesolloDg3fInterface::stop_loop() {
    run_loop_ = false;
} 

Eigen::VectorXd TesolloDg3fInterface::_read_joint_position() {
    TesolloReceivedData received_data = tesollo_client_->get_data();
    return vector_to_eigen(received_data.joint);
};

void TesolloDg3fInterface::_write_duty(const Eigen::VectorXi& duty) {
    std::vector<int> duty_vec = eigen_to_vector(duty);
    tesollo_client_->send_duty(duty_vec);
};

Eigen::VectorXi TesolloDg3fInterface::_torque_to_duty(const Eigen::VectorXd& torque) {
    
    // TODO: Figure out these constants more
    double TORQUE_TO_VOLT = 13.875 / 1.15;
    double MAX_MOTOR_VOLT = 11.1;
    double MAX_DUTY = 1000;

    Eigen::VectorXd volt = torque * TORQUE_TO_VOLT;
    Eigen::VectorXi duty = (volt / MAX_MOTOR_VOLT * MAX_DUTY).cast<int>();

    // Clamp to [-MAX_DUTY, MAX_DUTY]
    Eigen::VectorXi duty_clamped = duty.array().min(MAX_DUTY).max(-MAX_DUTY);

    return duty_clamped;
}

}