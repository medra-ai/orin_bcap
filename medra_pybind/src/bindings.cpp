/***
 * Python bindings for the DensoController
 *
 */
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include "DensoController.hpp"

namespace py = pybind11;
using namespace denso_controller;

PYBIND11_MODULE(_medra_bcap, m) {
  py::class_<BCAP_VARIANT>(m, "BCAP_VARIANT")
    .def(py::init<>())
    .def_readwrite("Type", &BCAP_VARIANT::Type)
    .def_readwrite("Arrays", &BCAP_VARIANT::Arrays)
    .def_property("CharValue",
                  [](BCAP_VARIANT &v) { return v.Value.CharValue; },
                  [](BCAP_VARIANT &v, u_char value) { v.Value.CharValue = value; })
    .def_property("ShortValue",
                  [](BCAP_VARIANT &v) { return v.Value.ShortValue; },
                  [](BCAP_VARIANT &v, u_short value) { v.Value.ShortValue = value; })
    .def_property("LongValue",
                  [](BCAP_VARIANT &v) { return v.Value.LongValue; },
                  [](BCAP_VARIANT &v, uint32_t value) { v.Value.LongValue = value; })
    .def_property("FloatValue",
                  [](BCAP_VARIANT &v) { return v.Value.FloatValue; },
                  [](BCAP_VARIANT &v, float value) { v.Value.FloatValue = value; })
    .def_property("DoubleValue",
                  [](BCAP_VARIANT &v) { return v.Value.DoubleValue; },
                  [](BCAP_VARIANT &v, double value) { v.Value.DoubleValue = value; })
    .def_property("String", 
                  [](BCAP_VARIANT &v) { return std::string(reinterpret_cast<char*>(v.Value.String)); },
                  [](BCAP_VARIANT &v, const std::string &str) { 
                    strncpy(reinterpret_cast<char*>(v.Value.String), str.c_str(), 40); 
                    v.Value.String[40] = '\0'; 
                  })
    .def_property("FloatArray", 
                  [](BCAP_VARIANT &v) { return py::array_t<float>({16}, v.Value.FloatArray); },
                  [](BCAP_VARIANT &v, py::array_t<float> arr) { 
                    auto buf = arr.request();
                    std::memcpy(v.Value.FloatArray, buf.ptr, 16 * sizeof(float)); 
                  })
    .def_property("DoubleArray", 
                  [](BCAP_VARIANT &v) { return py::array_t<double>({16}, v.Value.DoubleArray); },
                  [](BCAP_VARIANT &v, py::array_t<double> arr) { 
                    auto buf = arr.request();
                    std::memcpy(v.Value.DoubleArray, buf.ptr, 16 * sizeof(double)); 
                  })
    .def_property("Data",
                  [](BCAP_VARIANT &v) { return reinterpret_cast<uintptr_t>(v.Value.Data); },
                  [](BCAP_VARIANT &v, uintptr_t data) { v.Value.Data = reinterpret_cast<void*>(data); });

  py::class_<RobotTrajectory>(m, "RobotTrajectory")
    .def(py::init<>())
    .def_readonly("dimension", &RobotTrajectory::dimension)
    .def_readwrite("trajectory", &RobotTrajectory::trajectory)
    .def("size", &RobotTrajectory::size);

  py::register_exception<bCapException>(m, "bCapException");
  py::register_exception<SlaveMoveException>(m, "SlaveMoveException");
  py::register_exception<EnterSlaveModeException>(m, "EnterSlaveModeException");
  py::register_exception<ExitSlaveModeException>(m, "ExitSlaveModeException");

  py::class_<DensoController>(m, "DensoController")
    .def(py::init<>())

    // Low level commands
    .def("bCapOpen", &DensoController::bCapOpen, py::call_guard<py::gil_scoped_release>())
    .def("bCapClose", &DensoController::bCapClose, py::call_guard<py::gil_scoped_release>())
    .def("bCapServiceStart", &DensoController::bCapServiceStart, py::call_guard<py::gil_scoped_release>())
    .def("bCapServiceStop", &DensoController::bCapServiceStop, py::call_guard<py::gil_scoped_release>())
    .def("bCapControllerConnect", &DensoController::bCapControllerConnect, py::call_guard<py::gil_scoped_release>())
    .def("bCapControllerDisconnect", &DensoController::bCapControllerDisconnect, py::call_guard<py::gil_scoped_release>())
    .def("bCapGetRobot", &DensoController::bCapGetRobot, py::call_guard<py::gil_scoped_release>())
    .def("bCapReleaseRobot", &DensoController::bCapReleaseRobot, py::call_guard<py::gil_scoped_release>())
    .def("bCapClearError", &DensoController::bCapClearError, py::call_guard<py::gil_scoped_release>())
    .def("bCapRobotExecute", &DensoController::bCapRobotExecute, py::call_guard<py::gil_scoped_release>())
    .def("bCapRobotMove", &DensoController::bCapRobotMove, py::call_guard<py::gil_scoped_release>())
    .def("bCapMotor", &DensoController::bCapMotor, py::call_guard<py::gil_scoped_release>())
    .def("bCapSlvChangeMode", &DensoController::bCapSlvChangeMode, py::call_guard<py::gil_scoped_release>())
    .def("bCapSlvMove", &DensoController::bCapSlvMove, py::call_guard<py::gil_scoped_release>())
    .def("SetExtSpeed", &DensoController::SetExtSpeed, py::call_guard<py::gil_scoped_release>())
    .def("ManualReset", &DensoController::ManualReset, py::call_guard<py::gil_scoped_release>())
    .def("SetTcpLoad", &DensoController::SetTcpLoad, py::call_guard<py::gil_scoped_release>())
    .def("GetMountingCalib", &DensoController::GetMountingCalib, py::call_guard<py::gil_scoped_release>())
    .def("GetErrorDescription", &DensoController::GetErrorDescription, py::call_guard<py::gil_scoped_release>())

    // High level commands
    .def("bCapEnterProcess", &DensoController::bCapEnterProcess, py::call_guard<py::gil_scoped_release>())
    .def("bCapExitProcess", &DensoController::bCapExitProcess, py::call_guard<py::gil_scoped_release>())
    .def("CommandServoJoint", &DensoController::CommandServoJoint, py::call_guard<py::gil_scoped_release>())
    .def("ExecuteServoTrajectory", &DensoController::ExecuteServoTrajectory, py::call_guard<py::gil_scoped_release>())

    // Utilities
    .def("CommandFromVector", &DensoController::CommandFromVector, py::call_guard<py::gil_scoped_release>())
    .def("GetCurJnt", &DensoController::GetCurJnt, py::call_guard<py::gil_scoped_release>())
    .def("VectorFromVNT", &DensoController::VectorFromVNT, py::call_guard<py::gil_scoped_release>())
    .def("RadVectorFromVNT", &DensoController::RadVectorFromVNT, py::call_guard<py::gil_scoped_release>())
    .def("VNTFromVector", &DensoController::VNTFromVector, py::call_guard<py::gil_scoped_release>())
    .def("VNTFromRadVector", &DensoController::VNTFromRadVector, py::call_guard<py::gil_scoped_release>())

    // Class members
    .def_readonly("server_ip_address", &DensoController::server_ip_address)
    .def_readonly("server_port_num", &DensoController::server_port_num)
    .def_readonly("iSockFD", &DensoController::iSockFD)
    .def_readonly("lhController", &DensoController::lhController)
    .def_readonly("lhRobot", &DensoController::lhRobot)
    .def_readonly("current_waypoint_index", &DensoController::current_waypoint_index);


  m.def("VRad2Deg", &VRad2Deg);
  m.def("Rad2Deg", &Rad2Deg);
  m.def("Deg2Rad", &Deg2Rad);
}