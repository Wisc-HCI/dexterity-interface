
#include "robot_motion_interface/interface.hpp"
#include "robot_motion_interface/panda_interface.hpp"


#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

namespace py = pybind11;
using namespace robot_motion_interface;



PYBIND11_MODULE(robot_motion_interface_pybind, m) {

    // Accept flexible Eigen views (handles NumPy 1D arrays)
    using VecRef = Eigen::Ref<const Eigen::VectorXd>;
    py::class_<PandaInterface, Interface>(m, "PandaInterface")
        .def(py::init<std::string, std::string, const std::vector<std::string>&, VecRef, VecRef>())
        .def("set_joint_positions", &PandaInterface::set_joint_positions)
        .def("joint_state", &PandaInterface::joint_state)
        .def("start_loop", &PandaInterface::start_loop);

}