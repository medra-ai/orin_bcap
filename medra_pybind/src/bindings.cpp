/***
 * Python bindings for the DensoController
 *
 */
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "DensoController.hpp"

namespace py = pybind11;
using namespace denso_controller;

PYBIND11_MODULE(denso_pybind, m) {
  py::class_<bCapException>(m, "bCapException")
    .def(py::init<>())
    .def(py::init<const std::string&, int>());

  py::class_<DensoController>(m, "DensoController")
    .def(py::init<const char*, int>())
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
    .def("bCapEnterProcess", &DensoController::bCapEnterProcess)
    .def("bCapExitProcess", &DensoController::bCapExitProcess)
    .def("CommandFromVector", &DensoController::CommandFromVector)
    .def("GetCurJnt", &DensoController::GetCurJnt)
    .def("VectorFromVNT", &DensoController::VectorFromVNT)
    .def("RadVectorFromVNT", &DensoController::RadVectorFromVNT)
    .def("VNTFromVector", &DensoController::VNTFromVector)
    .def("VNTFromRadVector", &DensoController::VNTFromRadVector)
    .def_readonly("server_ip_address", &DensoController::server_ip_address)
    .def_readonly("server_port_num", &DensoController::server_port_num)
    .def_readonly("iSockFD", &DensoController::iSockFD)
    .def_readonly("lhController", &DensoController::lhController)
    .def_readonly("lhRobot", &DensoController::lhRobot);

  m.def("VRad2Deg", &VRad2Deg);
  m.def("Rad2Deg", &Rad2Deg);
  m.def("Deg2Rad", &Deg2Rad);
}