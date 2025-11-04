#pragma once

#include <array>
#include <Eigen/Dense>


namespace robot_motion_interface {

/**
 * @brief Converts a fixed-size std::array to an Eigen::VectorXd (deep copy)
 * @tparam N Number of elements in the array
 * @param arr (N) The input array.
 * @return (N) A Eigen Vector copied from arr
 */

template <std::size_t N>
inline Eigen::VectorXd array_to_eigen(const std::array<double, N>& arr) {
    Eigen::VectorXd vec(N);
    std::copy(arr.begin(), arr.end(), vec.data());
    return vec;
}

/**
 * @brief Converts an Eigen::VectorXd to a fixed-size std::array (deep copy)
 * @tparam N Number of elements to copy
 * @param vec (N) The input Eigen::VectorXd
 * @return (N) A fixed-size array copied from vec
 */
template <std::size_t N>
inline std::array<double, N> eigen_to_array(const Eigen::VectorXd& vec) {
    assert(vec.size() == static_cast<int>(N) && "Vector size mismatch");
    std::array<double, N> arr;
    std::copy(vec.data(), vec.data() + N, arr.begin());
    return arr;
}

/**
 * @brief Converts a std::vector<double> to an Eigen::VectorXf (NOT
         deep copy, still references same memory)
 * @param std_vec Input standard vector to convert
 * @return An Eigen vector referencing the same data as std_vec
 */
inline Eigen::VectorXd vector_to_eigen(std::vector<double> std_vec) {
    Eigen::Map<Eigen::VectorXd> vec(std_vec.data(), std_vec.size());
    return vec;
}


/**
 * @brief Converts an Eigen::VectorXf to std::vector<double> (deep copy)
 * @param eigen_vec Input eigen vector to convert.
 * @return Standard vector with data copied from eigen vector.
 */
inline std::vector<double> eigen_to_vector(Eigen::VectorXd eigen_vec) {
    std::vector<double> std_vec(eigen_vec.data(), eigen_vec.data() + eigen_vec.size());
    return std_vec;
}


/**
 * @brief Converts an Eigen::VectorXi to std::vector<int> (deep copy)
 * @param eigen_vec Input eigen vector to convert.
 * @return Standard vector with data copied from eigen vector.
 */
inline std::vector<int> eigen_to_vector(Eigen::VectorXi eigen_vec) {
    std::vector<int> std_vec(eigen_vec.data(), eigen_vec.data() + eigen_vec.size());
    return std_vec;
}




} 

