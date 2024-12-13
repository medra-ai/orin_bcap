/***
 * Python bindings for the DensoController
 *
 */
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/chrono.h>

#include "logging.hpp"
#include "DensoController.hpp"

namespace py = pybind11;
using namespace denso_controller;

PYBIND11_MODULE(_medra_bcap, m) {
  py::class_<JointPosition>(m, "JointPosition")
    .def(py::init<>());

  py::class_<RobotTrajectory>(m, "RobotTrajectory")
    .def(py::init<>());

  py::class_<TimestampedWaypoint>(m, "TimestampedWaypoint")
    .def(py::init<>())
    .def_readonly("time", &TimestampedWaypoint::time)
    .def_readonly("joint_position", &TimestampedWaypoint::joint_position);

  py::class_<TimestampedTrajectory>(m, "TimestampedTrajectory")
    .def(py::init<>());
  
  py::class_<TimestampedForceReading>(m, "TimestampedForceReading")
    .def(py::init<>())
    .def_readonly("time", &TimestampedForceReading::time)
    .def_readonly("force_torque_values", &TimestampedForceReading::force_torque_values);
  
  py::class_<TimestampedForceSequence>(m, "TimestampedForceSequence")
    .def(py::init<>())
    .def("__getitem__", [](const TimestampedForceSequence &t, size_t i) {
      return t[i];
    });

  py::class_<TrajectoryExecutionResult>(m, "TrajectoryExecutionResult")
    .def_readonly("error_code", &TrajectoryExecutionResult::error_code)
    .def_readonly("result_code", &TrajectoryExecutionResult::result_code)
    .def_readonly("joint_positions", &TrajectoryExecutionResult::joint_positions)
    .def_readonly("force_torque_values", &TrajectoryExecutionResult::force_torque_values);

  py::register_exception<bCapException>(m, "bCapException");
  py::register_exception<SlaveMoveException>(m, "SlaveMoveException");
  py::register_exception<EnterSlaveModeException>(m, "EnterSlaveModeException");
  py::register_exception<ExitSlaveModeException>(m, "ExitSlaveModeException");

  py::enum_<ExecuteServoTrajectoryError>(m, "ExecuteServoTrajectoryError")
    .value("SUCCESS", ExecuteServoTrajectoryError::SUCCESS)
    .value("ENTER_SLAVE_MODE_FAILED", ExecuteServoTrajectoryError::ENTER_SLAVE_MODE_FAILED)
    .value("SLAVE_MOVE_FAILED", ExecuteServoTrajectoryError::SLAVE_MOVE_FAILED)
    .value("EXIT_SLAVE_MODE_FAILED", ExecuteServoTrajectoryError::EXIT_SLAVE_MODE_FAILED);
  
  py::enum_<ExecuteServoTrajectoryResult>(m, "ExecuteServoTrajectoryResult")
    .value("COMPLETE", ExecuteServoTrajectoryResult::COMPLETE)
    .value("FORCE_LIMIT_EXCEEDED", ExecuteServoTrajectoryResult::FORCE_LIMIT_EXCEEDED)
    .value("ERROR", ExecuteServoTrajectoryResult::ERROR);

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
    );
}