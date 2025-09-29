#include <robot_motion/robot_properties/robot_properties.hpp>

namespace robot_motion {

RobotProperties::RobotProperties(const std::vector<std::string>& joint_names) {   

    joint_names_ = joint_names;
    n_joints_ = static_cast<int>(joint_names.size());
}

RobotProperties::RobotProperties(const std::vector<std::string>& joint_names, std::string urdf_path) {   

    joint_names_ = joint_names;
    n_joints_ = static_cast<int>(joint_names.size());

    pinocchio::urdf::buildModel(urdf_path, pin_model_);
    std::cout << "model name: " << pin_model_.name << std::endl;
    
    // Create data required by the algorithms
    pin_data_ = pinocchio::Data(pin_model_);
    
    // Sample a random configuration
    Eigen::VectorXd q = randomConfiguration(pin_model_);
    std::cout << "q: " << q.transpose() << std::endl;
    std::cout << "Model has " << pin_model_.njoints << " joints:" << std::endl;

    // // Note: Joint 0 is the universe joint (root, no DOFs)
    // for (pinocchio::JointIndex joint_id = 0; joint_id < pin_model_.njoints; ++joint_id)
    // {
    //     const std::string &name = pin_model_.names[joint_id];
    //     const std::size_t idx_v = pin_model_.joints[joint_id].idx_v();
    //     const std::size_t nv = pin_model_.joints[joint_id].nv();

    //     std::cout << "Joint ID: " << joint_id
    //               << ", Name: " << name
    //               << ", idx_v: " << idx_v
    //               << ", DoFs: " << nv
    //               << std::endl;
    // }

    
    Eigen::VectorXi reorder_indices = get_reorder_indices( pin_model_.names, joint_names_);
    std::cout << "Reorder idx: " << reorder_indices << std::endl;
    Eigen::VectorXd reorder_q = apply_reorder(q, reorder_indices);
    std::cout << "Reorder q: " << reorder_q << std::endl;
}


int RobotProperties::n_joints() const {
    return n_joints_;
}

const std::vector<std::string>& RobotProperties::joint_names() const {
    return joint_names_;
}



Eigen::VectorXd RobotProperties::coriolis(Eigen::VectorXd q, Eigen::VectorXd dq) {

}


Eigen::VectorXd RobotProperties::gravity(Eigen::VectorXd q) {

}


Eigen::VectorXd RobotProperties::friction(Eigen::VectorXd dq) {

}


} 
