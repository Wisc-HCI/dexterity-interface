#include "robot_motion_interface/panda_interface.hpp"

#include <iostream>

namespace robot_motion_interface {


PandaInterface::PandaInterface(std::string hostname, std::string urdf_path, std::vector<std::string> joint_names,
    const Eigen::VectorXd& kp, const Eigen::VectorXd& kd, double max_joint_delta) 
    : robot_(hostname) {
    
    // Set the collision behavior.
    std::array<double, 7> lower_torque_thresholds_nominal{{25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.}};
    std::array<double, 7> upper_torque_thresholds_nominal{{35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
    std::array<double, 7> lower_torque_thresholds_acceleration{{25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0}};
    std::array<double, 7> upper_torque_thresholds_acceleration{{35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
    std::array<double, 6> lower_force_thresholds_nominal{{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
    std::array<double, 6> upper_force_thresholds_nominal{{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
    std::array<double, 6> lower_force_thresholds_acceleration{{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
    std::array<double, 6> upper_force_thresholds_acceleration{{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
    robot_.setCollisionBehavior(
        lower_torque_thresholds_acceleration, upper_torque_thresholds_acceleration,
        lower_torque_thresholds_nominal, upper_torque_thresholds_nominal,
        lower_force_thresholds_acceleration, upper_force_thresholds_acceleration,
        lower_force_thresholds_nominal, upper_force_thresholds_nominal);

    rp_ = std::make_unique<robot_motion::RobotProperties>(joint_names, urdf_path);
    controller_ = std::make_unique<robot_motion::JointTorqueController>(*rp_, kp, kd, false, max_joint_delta);
};


void PandaInterface::set_joint_positions(const Eigen::VectorXd& q){
    controller_->set_setpoint(q);
};



Eigen::VectorXd PandaInterface::joint_state() {
    if (control_loop_running_) {
        std::lock_guard<std::mutex> lock(this->control_loop_mutex_);
        return control_loop_state_;

    } else { 
        // This can only be used when control loop is NOT running
        franka::RobotState robot_state = robot_.readOnce();

        Eigen::VectorXd q = array_to_eigen(robot_state.q);
        Eigen::VectorXd dq = array_to_eigen(robot_state.dq);
        Eigen::VectorXd state(14); state << q, dq;

        return state;
    }

};



void PandaInterface::start_loop() {

    // Put in own thread
    control_thread_ = std::thread([this]() {

        std::function<franka::Torques(const franka::RobotState&, franka::Duration)> 
        callback = [this](const franka::RobotState& robot_state, franka::Duration time_step) -> franka::Torques {
            Eigen::VectorXd q = array_to_eigen(robot_state.q);
            Eigen::VectorXd dq = array_to_eigen(robot_state.dq);
            Eigen::VectorXd state(14); state << q, dq;
          
            {  // Update shared variable within mutex lock
                std::lock_guard<std::mutex> lock(this->control_loop_mutex_);
                this->control_loop_state_ = state;
    
            }

            Eigen::VectorXd tau = this->controller_->step(state);
            
            franka::Torques torques(eigen_to_array<7>(tau));
            
            return torques;
        };
        
        // Continuously try recovering from communication_constraints_violation
        while (true) {
            try {
                this->control_loop_running_ =  true;
                this->robot_.control(callback);
            } catch (const franka::Exception& e) {
                this->control_loop_running_ =  false;
                
                std::string err = e.what();
                if (err.find("communication_constraints_violation") != std::string::npos) {
                    std::cerr << "[WARNING] Recovering in PandaInterface from " << err << std::endl;
                    robot_.automaticErrorRecovery();
                } else {
                    std::cerr << "[ERROR] Shutting down control loop in PandaInterface due to " << err << std::endl;
                    break; 
                }



            }
        }
        
    });

};

void PandaInterface::stop_loop() {
    if (!control_loop_running_) return;

    control_loop_running_ =  false;

    try {
        robot_.stop();
    } catch (const franka::Exception& e) {
        std::cerr << "[WARNING] Recovering in PandaInterface from " << e.what() << std::endl;
        robot_.automaticErrorRecovery();

    }
    if (control_thread_.joinable()) control_thread_.join();
}

} 
