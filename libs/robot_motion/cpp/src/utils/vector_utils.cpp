#include <robot_motion/utils/vector_utils.hpp>



namespace robot_motion {

    Eigen::VectorXi get_reorder_indices(const std::vector<std::string>& source_labels, const std::vector<std::string>& desired_labels) {
        size_t n = desired_labels.size();
        Eigen::VectorXi indices(n);

        for (size_t i = 0; i < n; ++i) {
            std::string label = desired_labels[i];
            
            // Search for label in source_labels
            std::vector<std::string>::const_iterator it = std::find(source_labels.cbegin(), source_labels.cend(), label);
            
            if (it == source_labels.end()) {
                // Label not found so raise error
                throw std::runtime_error(label + " from desired_labels not found in source_labels");
               
            } else {
                size_t index = std::distance(source_labels.cbegin(), it);
                indices[i] = index;

            }
        }

        return indices;
    }


    Eigen::VectorXd apply_reorder(const Eigen::VectorXd& input, const Eigen::VectorXi& indices) {
        size_t n = indices.size();
        Eigen::VectorXd result(n);
        for (size_t i = 0; i < n; ++i) {
            result[i] = input[indices[i]];
        }

        return result;

    }


    Eigen::VectorXd apply_original_order(const Eigen::VectorXd& input, const Eigen::VectorXi& indices, size_t source_label_size) {
        Eigen::VectorXd result = Eigen::VectorXd::Zero(source_label_size);
        
        for (size_t i = 0; i < indices.size(); ++i) {
            if (i >= source_label_size) {
                throw std::out_of_range("apply_original_order: index out of bounds");
            }
            
            result[indices[i]] = input[i];
        }

        return result;
    }

}