#include <rm_2/robot_properties/robot_properties.hpp>

namespace rm_2 {

RobotProperties::RobotProperties(const std::vector<std::string>& joint_names) {   
    joint_names_ = joint_names;
    n_joints_ = static_cast<int>(joint_names.size());
}

int RobotProperties::n_joints() const {
    return n_joints_;
}

const std::vector<std::string>& RobotProperties::joint_names() const {
    return joint_names_;
}

} 
