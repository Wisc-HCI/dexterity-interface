#pragma once

#include <string>
#include <vector>

namespace robot_motion {

class RobotProperties {
public:
    /**
     * @brief Construct RobotProperties class.
     * @param joint_names (n_joints) List of joint names (ordered as in 
     *                    URDF/robot model).
     */
    RobotProperties(const std::vector<std::string>& joint_names);

    /**
     * @brief Get the number of joints.
     * @return Number of joints.
     */
    int n_joints() const;

    /**
     * @brief Get joint names.
     * @return (n_joints) Vector of joint names
     */
    const std::vector<std::string>& joint_names() const;

private:
    std::vector<std::string> joint_names_;
    int n_joints_;
};

}