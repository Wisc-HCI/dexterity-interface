#include <Eigen/Dense>
#include <vector>

namespace robot_motion {
    /**
    * @brief Creates a reorder plan mapping desired_labels to their indices in source_labels
    * @param source_labels (n_source_labels) The original vector of labels
    * @param desired_labels The desired order of labels (can be subset of source_labels)
    * @return (n_desired_labels) Vector where indices[i] = j means input[i] maps to source_labels[j]
    * @throws std::runtime_error if a desired label is not found in source_labels
    */
    Eigen::VectorXi get_reorder_indices(const std::vector<std::string>& source_labels, const std::vector<std::string>& desired_labels);
    
    /**
    * @brief Applies the reorder indices (from get_reorder_indices) to an input vector
    * @param input The input Eigen vector to reorder
    * @param indices (n_desired_labels) Vector where indices[i] = j means input[i] maps to source_labels[j]
    * @return (n_desired_labels) A reordered Eigen vector according to the indices (may be 
        subset of input)
    */
   Eigen::VectorXd apply_reorder(const Eigen::VectorXd& input, const Eigen::VectorXi& indices);

   
    /**
    * @brief Applies original order of source_labels and fills in remaining positions with zeros.
    * @param input The Eigen vector containing values to order like the original source_labels.
    * @param indices (n_desired_labels) Vector where indices[i] = j means input[i] maps to source_labels[j]
    * @param source_label_size Length of original source_label.
    * @return (n_source_labels) Eigen vector with input values at j and zeros elsewhere.
    *
    */
    Eigen::VectorXd apply_original_order(const Eigen::VectorXd& input, const Eigen::VectorXi& indices, size_t source_label_size);

}