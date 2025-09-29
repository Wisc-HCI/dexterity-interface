#include <Eigen/Dense>


namespace robot_motion {
    /**
    * @brief Creates a reorder plan mapping desired_labels to their indices in source_labels
    * @param source_labels The original vector of labels
    * @param desired_labels The desired order of labels (can be subset of source_labels)
    * @return (n_desired_labels) An Eigen vector where each entry is the index of desired_labels[i] in source_labels
    * @throws std::runtime_error if a desired label is not found in source_labels
    */
    Eigen::VectorXi get_reorder_indices(std::vector<std::string> source_labels, std::vector<std::string> desired_labels);
    
    /**
    * @brief Applies the reorder indices (from get_reorder_indices) to an input vector
    * @param input The input Eigen vector to reorder
    * @param indices (n_desired_labels) The indices vector containing indices for reordering.
    * @return (n_desired_labels) A reordered Eigen vector according to the indices (may be 
        subset of input)
    */
    Eigen::VectorXd apply_reorder(Eigen::VectorXd input, Eigen::VectorXi indices);

}