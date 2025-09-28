#include "rm_2/robot_properties/robot_properties.hpp"
#include "rm_2/controllers/controller.hpp"
#include "rm_2/controllers/joint_torque_controller.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

// TODO remove after debug
#include <pybind11/iostream.h>



namespace py = pybind11;
using namespace rm_2;



PYBIND11_MODULE(rm_2_pybind, m) {
    py::class_<Controller>(m, "Controller");  // bind base first

    py::class_<RobotProperties>(m, "RobotProperties")
        .def(py::init<const std::vector<std::string>&>())
        .def("n_joints", &RobotProperties::n_joints)
        .def("joint_names", &RobotProperties::joint_names,
             py::return_value_policy::reference_internal);

    // Accept flexible Eigen views (handles NumPy 1D arrays nicely)
    using VecRef = Eigen::Ref<const Eigen::VectorXd>;
    py::class_<JointTorqueController, Controller>(m, "JointTorqueController")
        .def(py::init<const RobotProperties&, VecRef, VecRef>(),
             py::arg("props"), py::arg("kp"), py::arg("kd"))
        .def("step", &JointTorqueController::step)
        .def("set_setpoint", &JointTorqueController::set_setpoint);
}