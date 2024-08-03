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

PYBIND11_MODULE(medra_bcap, m) {
  py::enum_<BCAP_HRESULT>(m, "BCAP_HRESULT")
    .value("BCAP_S_OK", BCAP_S_OK)
    .value("BCAP_E_NOTIMPL", BCAP_E_NOTIMPL)
    .value("BCAP_E_ABORT", BCAP_E_ABORT)
    .value("BCAP_E_FAIL", BCAP_E_FAIL)
    .value("BCAP_E_UNEXPECTED", BCAP_E_UNEXPECTED)
    .value("BCAP_E_INVALIDRCVPACKET", BCAP_E_INVALIDRCVPACKET)
    .value("BCAP_E_INVALIDSNDPACKET", BCAP_E_INVALIDSNDPACKET)
    .value("BCAP_E_INVALIDARGTYPE", BCAP_E_INVALIDARGTYPE)
    .value("BCAP_E_ROBOTISBUSY", BCAP_E_ROBOTISBUSY)
    .value("BCAP_E_INVALIDCOMMAND", BCAP_E_INVALIDCOMMAND)
    .value("BCAP_E_PACKETSIZEOVER", BCAP_E_PACKETSIZEOVER)
    .value("BCAP_E_ARGSIZEOVER", BCAP_E_ARGSIZEOVER)
    .value("BCAP_E_ACCESSDENIED", BCAP_E_ACCESSDENIED)
    .value("BCAP_E_HANDLE", BCAP_E_HANDLE)
    .value("BCAP_E_OUTOFMEMORY", BCAP_E_OUTOFMEMORY)
    .value("BCAP_E_INVALIDARG", BCAP_E_INVALIDARG);

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

  py::class_<bCapException>(m, "bCapException")
    .def(py::init<>())
    .def(py::init<const std::string&, int>());
    .def("error_code", &bCapException::error_code)
    .def("error_description", &bCapException::error_description);

  py::class_<RobotTrajectory>(m, "RobotTrajectory")
    .def(py::init<>())
    .def_readonly("dimension", &RobotTrajectory::dimension)
    .def_readwrite("trajectory", &RobotTrajectory::trajectory)
    .def("size", &RobotTrajectory::size);

  py::class_<DensoController>(m, "DensoController")
    .def(py::init<>())

    // Low level commands
    .def("bCapOpen", &DensoController::bCapOpen)
    .def("bCapClose", &DensoController::bCapClose)
    .def("bCapServiceStart", &DensoController::bCapServiceStart)
    .def("bCapServiceStop", &DensoController::bCapServiceStop)
    .def("bCapControllerConnect", &DensoController::bCapControllerConnect)
    .def("bCapControllerDisconnect", &DensoController::bCapControllerDisconnect)
    .def("bCapGetRobot", &DensoController::bCapGetRobot)
    .def("bCapReleaseRobot", &DensoController::bCapReleaseRobot)
    .def("bCapRobotExecute", &DensoController::bCapRobotExecute)
    .def("bCapRobotMove", &DensoController::bCapRobotMove)
    .def("bCapMotor", &DensoController::bCapMotor)
    .def("bCapSlvChangeMode", &DensoController::bCapSlvChangeMode)
    .def("bCapSlvMove", &DensoController::bCapSlvMove)
    .def("SetExtSpeed", &DensoController::SetExtSpeed)
    .def("ManualReset", &DensoController::ManualReset) // Untested
    .def("SetTcpLoad", &DensoController::SetTcpLoad) // Untested
    .def("ChangeTool", &DensoController::ChangeTool) // Untested, alternative to SetTcpLoad?
    .def("GetMountingCalib", &DensoController::GetMountingCalib) // Untested
    .def("GetErrorDescription", &DensoController::GetErrorDescription) // Untested

    // High level commands
    .def("bCapEnterProcess", &DensoController::bCapEnterProcess)
    .def("bCapExitProcess", &DensoController::bCapExitProcess)
    .def("executeServoTrajectory", &DensoController::executeServoTrajectory)

    // Utilities
    .def("CommandFromVector", &DensoController::CommandFromVector)
    .def("GetCurJnt", &DensoController::GetCurJnt)
    .def("VectorFromVNT", &DensoController::VectorFromVNT)
    .def("RadVectorFromVNT", &DensoController::RadVectorFromVNT)
    .def("VNTFromVector", &DensoController::VNTFromVector)
    .def("VNTFromRadVector", &DensoController::VNTFromRadVector)

    // Class members
    .def_readonly("server_ip_address", &DensoController::server_ip_address)
    .def_readonly("server_port_num", &DensoController::server_port_num)
    .def_readonly("iSockFD", &DensoController::iSockFD)
    .def_readonly("lhController", &DensoController::lhController)
    .def_readonly("lhRobot", &DensoController::lhRobot);

  m.def("VRad2Deg", &VRad2Deg);
  m.def("Rad2Deg", &Rad2Deg);
  m.def("Deg2Rad", &Deg2Rad);
}