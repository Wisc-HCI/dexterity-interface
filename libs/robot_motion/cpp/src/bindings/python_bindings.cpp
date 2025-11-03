#include "robot_motion/robot_properties/robot_properties.hpp"
#include "robot_motion/controllers/controller.hpp"
#include "robot_motion/controllers/joint_torque_controller.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>




namespace py = pybind11;
using namespace robot_motion;



PYBIND11_MODULE(robot_motion_pybind, m) {
    py::class_<Controller>(m, "Controller");  // bind base first

    py::class_<RobotProperties>(m, "RobotProperties")
        .def(py::init<const std::vector<std::string>&>())
        .def(py::init<const std::vector<std::string>&, std::string>())
        .def("n_joints", &RobotProperties::n_joints)
        .def("joint_names", &RobotProperties::joint_names,
             py::return_value_policy::reference_internal)
        .def("forward_kinematics", &RobotProperties::forward_kinematics);

    // Allow NumPy 1D arrays
    using VecRef = Eigen::Ref<const Eigen::VectorXd>;
    py::class_<JointTorqueController, Controller>(m, "JointTorqueController")
        .def(py::init<const RobotProperties&, VecRef, VecRef, bool>(),
             py::arg("props"), py::arg("kp"), py::arg("kd"), py::arg("gravity_compensation"))
        .def("step", &JointTorqueController::step)
        .def("set_setpoint", &JointTorqueController::set_setpoint);
}