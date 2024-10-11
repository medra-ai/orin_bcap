/***
 * Python bindings for the DensoController
 *
 */
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include "logging.hpp"
#include "DensoController.hpp"

namespace py = pybind11;
using namespace denso_controller;

PYBIND11_MODULE(_medra_bcap, m) {
  py::class_<RobotTrajectory>(m, "RobotTrajectory")
    .def(py::init<>())
    .def_readonly("dimension", &RobotTrajectory::dimension)
    .def_readwrite("trajectory", &RobotTrajectory::trajectory)
    .def("size", &RobotTrajectory::size);

  py::register_exception<bCapException>(m, "bCapException");
  py::register_exception<SlaveMoveException>(m, "SlaveMoveException");
  py::register_exception<EnterSlaveModeException>(m, "EnterSlaveModeException");
  py::register_exception<ExitSlaveModeException>(m, "ExitSlaveModeException");

  py::enum_<DensoController::ExecuteServoTrajectoryError>(m, "ExecuteServoTrajectoryError")
    .value("SUCCESS", DensoController::ExecuteServoTrajectoryError::SUCCESS)
    .value("ENTER_SLAVE_MODE_FAILED", DensoController::ExecuteServoTrajectoryError::ENTER_SLAVE_MODE_FAILED)
    .value("SLAVE_MOVE_FAILED", DensoController::ExecuteServoTrajectoryError::SLAVE_MOVE_FAILED)
    .value("EXIT_SLAVE_MODE_FAILED", DensoController::ExecuteServoTrajectoryError::EXIT_SLAVE_MODE_FAILED);
  
  py::enum_<DensoController::ExecuteServoTrajectoryResult>(m, "ExecuteServoTrajectoryResult")
    .value("COMPLETE", DensoController::ExecuteServoTrajectoryResult::COMPLETE)
    .value("FORCE_LIMIT_EXCEEDED", DensoController::ExecuteServoTrajectoryResult::FORCE_LIMIT_EXCEEDED)
    .value("ERROR", DensoController::ExecuteServoTrajectoryResult::ERROR);

  py::class_<DensoController>(m, "DensoController")
    .def(py::init<>())

    // High level commands
    .def("Start", [](DensoController &dc) {
      setup_pybind11_logging();
      return dc.Start();
    }, py::call_guard<py::gil_scoped_release>())
    .def("Stop", &DensoController::Stop, py::call_guard<py::gil_scoped_release>())

    // Error handling functions
    .def("Motor", &DensoController::Motor, py::call_guard<py::gil_scoped_release>(),
          py::arg("command")
    )
    .def("ManualReset", &DensoController::ManualReset, py::call_guard<py::gil_scoped_release>())
    .def("ClearError", &DensoController::ClearError, py::call_guard<py::gil_scoped_release>())
    .def("GetErrorDescription", &DensoController::GetErrorDescription, py::call_guard<py::gil_scoped_release>(),
          py::arg("error_code")
    )

    .def("GetJointPositions", &DensoController::GetJointPositions, py::call_guard<py::gil_scoped_release>())
    .def("ExecuteServoTrajectory", &DensoController::ExecuteServoTrajectory, py::call_guard<py::gil_scoped_release>(),
          py::arg("traj"), py::arg("total_force_limit"), py::arg("total_torque_limit"), py::arg("per_axis_force_torque_limits")
    )
    .def("GetTrajectoryExecutionEnabled", &DensoController::GetTrajectoryExecutionEnabled, py::call_guard<py::gil_scoped_release>())
    .def("SetTrajectoryExecutionEnabled", &DensoController::SetTrajectoryExecutionEnabled, py::call_guard<py::gil_scoped_release>())
    .def("SetTcpLoad", &DensoController::SetTcpLoad, py::call_guard<py::gil_scoped_release>(),
          py::arg("tool_value")
    )
    .def("GetMountingCalib", &DensoController::GetMountingCalib, py::call_guard<py::gil_scoped_release>(),
          py::arg("work_coordinate")
    )

    .def_readonly("current_waypoint_index", &DensoController::current_waypoint_index);
}