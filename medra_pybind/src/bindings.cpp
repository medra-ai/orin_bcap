/***
 * Python bindings for the Denso Robot controller Orin bCap drivers
 *
 */
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>


// #include "medra_pybind/include/bindings.h"
#include "bCAPClient/bcap_client.h"

namespace py = pybind11;

PYBIND11_MODULE(denso_pybind, m) {
  m.doc() = "BCAP Client module";

// Binding definitions for bcap_client bCap_Robot* functions
  m.def("bCap_RobotGetVariable", &bCap_RobotGetVariable, "Get robot variable",
        py::arg("fd"), py::arg("hRobot"), py::arg("bstrName"), py::arg("bstrOption"), py::arg("hVariable"));
  
  m.def("bCap_RobotGetVariableNames", &bCap_RobotGetVariableNames, "Get robot variable names",
        py::arg("fd"), py::arg("hRobot"), py::arg("bstrOption"), py::arg("pVal"));
  
  m.def("bCap_RobotExecute", &bCap_RobotExecute, "Execute robot command",
        py::arg("fd"), py::arg("hRobot"), py::arg("bstrCommand"), py::arg("vntParam"), py::arg("pVal"));
  
  m.def("bCap_RobotAccelerate", &bCap_RobotAccelerate, "Accelerate robot",
        py::arg("fd"), py::arg("hRobot"), py::arg("lAxis"), py::arg("fAccel"), py::arg("fDecel"));
  
  m.def("bCap_RobotChange", &bCap_RobotChange, "Change robot settings",
        py::arg("fd"), py::arg("hRobot"), py::arg("bstrName"));
  
  m.def("bCap_RobotChuck", &bCap_RobotChuck, "Chuck robot",
        py::arg("fd"), py::arg("hRobot"), py::arg("bstrOption"));
  
  m.def("bCap_RobotDrive", &bCap_RobotDrive, "Drive robot",
        py::arg("fd"), py::arg("hRobot"), py::arg("lNo"), py::arg("fMov"), py::arg("bstrOption"));
  
  m.def("bCap_RobotGoHome", &bCap_RobotGoHome, "Send robot home",
        py::arg("fd"), py::arg("hRobot"));
  
  m.def("bCap_RobotHalt", &bCap_RobotHalt, "Halt robot",
        py::arg("fd"), py::arg("hRobot"), py::arg("bstrOption"));
  
  m.def("bCap_RobotHold", &bCap_RobotHold, "Hold robot",
        py::arg("fd"), py::arg("hRobot"), py::arg("bstrOption"));
  
  m.def("bCap_RobotMove", &bCap_RobotMove, "Move robot",
        py::arg("fd"), py::arg("hRobot"), py::arg("lComp"), py::arg("vntPose"), py::arg("bstrOption"));
  
  m.def("bCap_RobotRotate", &bCap_RobotRotate, "Rotate robot",
        py::arg("fd"), py::arg("hRobot"), py::arg("vntRotSuf"), py::arg("fDeg"), py::arg("vntPivot"), py::arg("bstrOption"));
  
  m.def("bCap_RobotSpeed", &bCap_RobotSpeed, "Set robot speed",
        py::arg("fd"), py::arg("hRobot"), py::arg("lAxis"), py::arg("fSpeed"));
  
  m.def("bCap_RobotUnchuck", &bCap_RobotUnchuck, "Unchuck robot",
        py::arg("fd"), py::arg("hRobot"), py::arg("bstrOption"));
  
  m.def("bCap_RobotUnhold", &bCap_RobotUnhold, "Unhold robot",
        py::arg("fd"), py::arg("hRobot"), py::arg("bstrOption"));
  
  m.def("bCap_RobotGetAttribute", &bCap_RobotGetAttribute, "Get robot attribute",
        py::arg("fd"), py::arg("hRobot"), py::arg("pVal"));
  
  m.def("bCap_RobotGetHelp", &bCap_RobotGetHelp, "Get robot help",
        py::arg("fd"), py::arg("hRobot"), py::arg("pVal"));
  
  m.def("bCap_RobotGetName", &bCap_RobotGetName, "Get robot name",
        py::arg("fd"), py::arg("hRobot"), py::arg("pVal"));
  
  m.def("bCap_RobotGetTag", &bCap_RobotGetTag, "Get robot tag",
        py::arg("fd"), py::arg("hRobot"), py::arg("pVal"));
  
  m.def("bCap_RobotPutTag", &bCap_RobotPutTag, "Put robot tag",
        py::arg("fd"), py::arg("hRobot"), py::arg("newVal"));
  
  m.def("bCap_RobotGetID", &bCap_RobotGetID, "Get robot ID",
        py::arg("fd"), py::arg("hRobot"), py::arg("pVal"));
  
  m.def("bCap_RobotPutID", &bCap_RobotPutID, "Put robot ID",
        py::arg("fd"), py::arg("hRobot"), py::arg("newVal"));
  
  m.def("bCap_RobotRelease", &bCap_RobotRelease, "Release robot",
        py::arg("fd"), py::arg("hRobot"));

}