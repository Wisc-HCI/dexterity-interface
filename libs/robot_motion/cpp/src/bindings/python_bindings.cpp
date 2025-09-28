#include "robot_motion/robot_properties/robot_properties.hpp"
#include "robot_motion/controllers/controller.hpp"
#include "robot_motion/controllers/joint_torque_controller.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

// TODO remove after debug
#include <pybind11/iostream.h>



namespace py = pybind11;
using namespace robot_motion;



PYBIND11_MODULE(robot_motion_pybind, m) {
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
        // .def("step", &JointTorqueController::step)
        
        // TODO: Remove after debut
        .def("step", [](JointTorqueController &self, const Eigen::VectorXd &state) {
            // Move redirect object to the heap or outer scope so it persists
            auto stream = std::make_unique<py::scoped_ostream_redirect>(
                std::cout,
                py::module_::import("sys").attr("stdout")
            );

            // Now call the C++ method
            auto result = self.step(state);

            // Keep stream alive until after the C++ method returns
            stream.reset();

            return result;
        })
        .def("set_setpoint", &JointTorqueController::set_setpoint);
}