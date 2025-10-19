#include <robot_motion/robot_properties/robot_properties.hpp>

namespace robot_motion {

RobotProperties::RobotProperties(const std::vector<std::string>& joint_names) {   

    joint_names_ = joint_names;
    n_joints_ = static_cast<int>(joint_names.size());
}

RobotProperties::RobotProperties(const std::vector<std::string>& joint_names, std::string urdf_path) {   

    joint_names_ = joint_names;
    n_joints_ = static_cast<int>(joint_names.size());

    // Pinocchio setup
    pinocchio::urdf::buildModel(urdf_path, pin_model_);
    pin_data_ = pinocchio::Data(pin_model_);

    // Skip index 0 (universe base)
    std::vector<std::string> pin_joint_names(
        pin_model_.names.begin() + 1,
        pin_model_.names.end()
    );
    pin_reorder_indices_ = get_reorder_indices(pin_joint_names, joint_names_);
    pin_joint_length_ = pin_joint_names.size();
        
}


int RobotProperties::n_joints() const {
    return n_joints_;
}

const std::vector<std::string>& RobotProperties::joint_names() const {
    return joint_names_;
}



Eigen::VectorXd RobotProperties::coriolis(Eigen::VectorXd q, Eigen::VectorXd dq) {

    if (pin_model_.njoints == 1) {
        // std::cerr << "Warning: cannot calculate coriolis since urdf was not passed in constructor." << std::endl;
        // TODO: UNCOMMENT AFTER DEBUG
        return Eigen::VectorXd::Zero(n_joints_);
    }

    q = apply_original_order(q, pin_reorder_indices_, pin_joint_length_);
    dq = apply_original_order(dq, pin_reorder_indices_, pin_joint_length_);

    Eigen::MatrixXd coriolis_matrix = pinocchio::computeCoriolisMatrix(pin_model_, pin_data_, q, dq);
    Eigen::VectorXd coriolis = coriolis_matrix * dq;

    return apply_reorder(coriolis, pin_reorder_indices_);
}


Eigen::VectorXd RobotProperties::gravity(Eigen::VectorXd q) {
    if (pin_model_.njoints == 1) {
        std::cerr << "Warning: cannot calculate gravity since urdf was not passed in constructor." << std::endl;
        return Eigen::VectorXd::Zero(n_joints_);
    }

    q = apply_original_order(q, pin_reorder_indices_, pin_joint_length_);

    Eigen::VectorXd gravity = pinocchio::computeGeneralizedGravity(pin_model_, pin_data_, q);

    return apply_reorder(gravity, pin_reorder_indices_);

}


Eigen::VectorXd RobotProperties::friction(Eigen::VectorXd dq) {
    // TODO: Friction model needed?
    return Eigen::VectorXd::Zero(n_joints_);

}


} 
