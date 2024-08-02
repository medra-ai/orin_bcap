/***
 * Python bindings for the Denso Robot controller Orin bCap drivers
 *
 */
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>


// #include "medra_pybind/include/bindings.h"
#include "medra_denso_robot.hpp"

namespace py = pybind11;
using namespace medra_denso_robot;

PYBIND11_MODULE(medra_denso_robot, m) {
  py::class_<MedraDensoRobot>(m, "MedraDensoRobot")
    .def(py::init<const std::string &, const int *, const std::string &, int, int>())
    .def("controller_connect", &MedraDensoRobot::ControllerConnect)
    .def("exec_take_arm", &MedraDensoRobot::ExecTakeArm)
    .def("exec_give_arm", &MedraDensoRobot::ExecGiveArm)
    .def("exec_cur_jnt", &MedraDensoRobot::ExecCurJnt)
    .def("exec_slave_move", &MedraDensoRobot::ExecSlaveMove)
    .def("change_mode", &MedraDensoRobot::ChangeMode);
}