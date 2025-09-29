#pragma once

#include <robot_motion/utils/vector_utils.hpp>

#include <string>
#include <vector>

#include <Eigen/Dense>

// TODO: Update after testing
#include <iostream>
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

namespace robot_motion {

class RobotProperties {
public:
    /**
     * @brief Construct RobotProperties class as a storage class. 
     *      Coriolis, gravity, friction will all be 0 bc no urdf passed.
     * @param joint_names (n_joints) List of joint names (ordered as in 
     *                    URDF/robot model).
     */
    RobotProperties(const std::vector<std::string>& joint_names);

    /**
     * @brief Construct RobotProperties class with coriolis, gravity, friction.
     * @param joint_names (n_joints) List of joint names (ordered as in 
     *                    URDF/robot model).
     * @param urdf_path Relative or absolute path to urdf.
     */
    RobotProperties(const std::vector<std::string>& joint_names,  std::string urdf_path);  


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


    /**
     * @brief Compute the Coriolis vector: C(q, dq) * dq.
     * @param q Joint positions vector
     * @param dq Joint velocities vector
     * @return (n_joints) Vector of forces that Coriolis applies on each joint (Nm)
     */
    Eigen::VectorXd coriolis(Eigen::VectorXd q, Eigen::VectorXd dq);

    /**
     * @brief Compute the gravity vector: G(q)
     * @param q Joint positions vector
     * @return (n_joints) Vector of forces that gravity applies on each joint (Nm)
     */
    Eigen::VectorXd gravity(Eigen::VectorXd q);

    /**
     * @brief Compute the friction vector: F(dq)
     * @param dq Joint velocities vector
     * @return (n_joints) Vector of forces that gravity applies on each joint (Nm)
     */
    Eigen::VectorXd friction(Eigen::VectorXd dq);
    

private:
    std::vector<std::string> joint_names_;
    int n_joints_;
    pinocchio::Model pin_model_;
    pinocchio::Data pin_data_;

};

}