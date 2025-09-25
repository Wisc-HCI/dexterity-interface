#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "robot_motion/robot_properties/robot_properties.hpp"
#include "robot_motion/controllers/controller.hpp"
#include "robot_motion/controllers/joint_torque_controller.hpp"


namespace py = pybind11;
using namespace robot_motion;


PYBIND11_MODULE(robot_motion_pybind, m) {
    py::class_<RobotProperties>(m, "RobotProperties")
        .def(py::init<const std::vector<std::string>&>())
        .def("n_joints", &RobotProperties::n_joints)
        .def("joint_names", &RobotProperties::joint_names, py::return_value_policy::reference_internal);

    py::class_<Controller>(m, "Controller");

    py::class_<JointTorqueController, Controller>(m, "JointTorqueController")
        .def(py::init<const RobotProperties&, const Eigen::VectorXd&, const Eigen::VectorXd&>())
        .def("step", &JointTorqueController::step);
}