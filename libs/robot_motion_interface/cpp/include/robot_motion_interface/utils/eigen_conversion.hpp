#pragma once

#include <array>
#include <Eigen/Dense>


namespace robot_motion_interface {

/**
 * @brief Converts a fixed-size std::array to an Eigen::VectorXd
 * @tparam T Element type (e.g., double, float)
 * @tparam N Number of elements in the array
 * @param arr (N) The input std::array.
 * @return (N) A Eigen Vector copied from arr
 */

template <typename T, std::size_t N>
inline Eigen::VectorXd array_to_eigen(const std::array<T, N>& arr)
{
    Eigen::VectorXd vec(N);
    std::copy(arr.begin(), arr.end(), vec.data());
    return vec;
}


/**
 * @brief Converts an Eigen::VectorXd to a fixed-size std::array
 * @tparam T Element type (e.g., double, float)
 * @tparam N Number of elements to copy
 * @param vec (N) The input Eigen::VectorXd
 * @return (N) A fixed-size array copied from vec
 */
template <typename T, std::size_t N>
inline std::array<T, N> eigen_to_array(const Eigen::VectorXd& vec)
{
    assert(vec.size() == static_cast<int>(N) && "Vector size mismatch");
    std::array<T, N> arr;
    std::copy(vec.data(), vec.data() + N, arr.begin());
    return arr;
}

} 

