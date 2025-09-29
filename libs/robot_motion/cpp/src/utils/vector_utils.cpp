#include <robot_motion/utils/vector_utils.hpp>



namespace robot_motion {

    Eigen::VectorXi get_reorder_indices(std::vector<std::string> source_labels, std::vector<std::string> desired_labels) {
        size_t n = desired_labels.size();
        Eigen::VectorXi indices(n);

        for (size_t i = 0; i < n; ++i) {
            std::string label = desired_labels[i];
            
            // Search for label in source_labels
            std::vector<std::string>::iterator it = std::find(source_labels.begin(), source_labels.end(), label);
            
            if (it == source_labels.end()) {
                // Label not found so raise error
                throw std::runtime_error(label + " from desired_labels not found in source_labels");
               
            } else {
                size_t index = std::distance(source_labels.begin(), it);
                indices[i] = index;

            }
        }

        return indices;
    }


    Eigen::VectorXd apply_reorder(Eigen::VectorXd input, Eigen::VectorXi indices) {
        Eigen::VectorXd result(indices.size());
        for (size_t i = 0; i < indices.size(); ++i) {
            result[i] = input[indices[i]];
        }

        return result;

    }

}