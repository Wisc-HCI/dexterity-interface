#pragma once

#include <array>
#include <Eigen/Dense>


namespace robot_motion_interface {


template <typename T, std::size_t N>
inline Eigen::VectorXd array_to_eigen(const std::array<T, N>& arr)
{
    Eigen::VectorXd vec(N);
    std::copy(arr.begin(), arr.end(), vec.data());
    return vec;
}

template <typename T, std::size_t N>
inline std::array<T, N> eigen_to_array(const Eigen::VectorXd& vec)
{
    assert(vec.size() == static_cast<int>(N) && "Vector size mismatch");
    std::array<T, N> arr;
    std::copy(vec.data(), vec.data() + N, arr.begin());
    return arr;
}

} 

