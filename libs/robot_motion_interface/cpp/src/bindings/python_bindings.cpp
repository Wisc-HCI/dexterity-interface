
#include "robot_motion_interface/interface.hpp"
#include "robot_motion_interface/panda_interface.hpp"
#include "robot_motion_interface/tesollo/tesollo_dg3f_interface.hpp"


#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

namespace py = pybind11;
using namespace robot_motion_interface;



PYBIND11_MODULE(robot_motion_interface_pybind, m) {
    py::class_<Interface>(m, "Interface");

    // Allow NumPy 1D arrays
    using VecRef = Eigen::Ref<const Eigen::VectorXd>;
    py::class_<PandaInterface, Interface>(m, "PandaInterface")
        .def(py::init<std::string, std::string, const std::vector<std::string>&, VecRef, VecRef>())
        .def("set_joint_positions", &PandaInterface::set_joint_positions)
        .def("joint_state", &PandaInterface::joint_state)
        .def("start_loop", &PandaInterface::start_loop);


    using VecRef = Eigen::Ref<const Eigen::VectorXd>;
    py::class_<TesolloDg3fInterface, Interface>(m, "TesolloDg3fInterface")
        .def(py::init<std::string, int, const std::vector<std::string>&, VecRef, VecRef, double>())
        .def("set_joint_positions", &TesolloDg3fInterface::set_joint_positions)
        .def("joint_state", &TesolloDg3fInterface::joint_state)
        .def("start_loop", &TesolloDg3fInterface::start_loop)
        .def("stop_loop", &TesolloDg3fInterface::stop_loop);
}