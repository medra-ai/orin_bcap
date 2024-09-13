/**
 * C++ Wrapper around Denso b-CAP library
 * Based on https://github.com/quangounet/denso/blob/master/cpp/src/DensoController.h
 */ 

#ifndef DensoController_hpp
#define DensoController_hpp

#include "b-Cap.h"
#include "util.hpp"
#include <exception>
#include <string>
#include <cassert>
#include <vector>
#include <tuple>

#define DEFAULT_SERVER_IP_ADDRESS    "192.168.0.1"
#define DEFAULT_SERVER_PORT_NUM      5007

#define E_BUF_FULL           0x83201483
#define S_BUF_FULL           0x0F200501

#define PI 3.1415926535897932
#define nSEC_PER_SECOND 1E9
#define dReal float

namespace denso_controller {

class bCapException : public std::exception {
public:
    bCapException(std::string msg) {
        _err_msg = msg;
    }

    bCapException(std::string msg, std::string err_description) {
        _err_msg = msg + " Description: " + err_description;
    }

    const char* what() const noexcept override {
        return _err_msg.c_str();
    }

private:
    std::string _err_msg;
};

// Custom exceptions for ExecuteServoTrajectory
class SlaveMoveException : public std::exception {
public:
    SlaveMoveException(std::string msg="") {
        _err_msg = "Failed to Execute SlaveMove " + msg;
    }

    const char* what() const noexcept override {
        return _err_msg.c_str();
    }

private:
    std::string _err_msg;
};
class EnterSlaveModeException : public std::exception {
public:
    EnterSlaveModeException(std::string msg="") {
        _err_msg = "Failed to ENTER slave mode " + msg;
    }

    const char* what() const noexcept override {
        return _err_msg.c_str();
    }
private:
    std::string _err_msg;
};
class ExitSlaveModeException : public std::exception {
public:
    ExitSlaveModeException(std::string msg="") {
        _err_msg = "Failed to EXIT slave mode " + msg;
    }

    const char* what() const noexcept override {
        return _err_msg.c_str();
    }
private:
    std::string _err_msg;
};


class DensoController {

public:
    DensoController();
    // Delete the copy constructor because the class contains
    // a non-copyable atomic variable.
    DensoController(const DensoController& other) = delete;

    // low level commands
    void bCapOpen();
    void bCapClose();
    void bCapServiceStart();
    void bCapServiceStop();
    void bCapControllerConnect();
    void bCapControllerDisconnect();
    void bCapGetRobot();
    void bCapReleaseRobot();
    BCAP_HRESULT bCapClearError();
    BCAP_HRESULT bCapRobotExecute(const char* command, const char* option);
    BCAP_HRESULT bCapRobotMove(const char* pose, const char* option);
    BCAP_HRESULT bCapMotor(bool command);
    BCAP_HRESULT bCapSlvChangeMode(const char* mode);
    BCAP_HRESULT printSlvMode();
    BCAP_HRESULT bCapSlvMove(BCAP_VARIANT* pose, BCAP_VARIANT* result);
    BCAP_HRESULT SetExtSpeed(const char* speed);
    BCAP_HRESULT ManualReset();
    BCAP_HRESULT SetTcpLoad(const int32_t tool_value);
    BCAP_HRESULT ChangeTool(char* tool_name); // Alternative to SetTcpLoad?
    std::tuple<BCAP_HRESULT, std::vector<double>> GetMountingCalib(const char* work_coordinate);
    std::string GetErrorDescription(BCAP_HRESULT error_code);

    // high level commands
    void bCapEnterProcess();
    void bCapExitProcess();
    void CommandServoJoint(const std::vector<double> joint_position);
    void ClosedLoopCommandServoJoint(std::vector<double> last_waypoint);
    void ExecuteServoTrajectory(RobotTrajectory& traj);

    // utilities
    const char* CommandFromVector(std::vector<double> q);
    std::tuple<BCAP_HRESULT, std::vector<double>> GetCurJnt();
    std::tuple<BCAP_HRESULT, std::vector<double>> GetForceValue();
    std::vector<double> VectorFromVNT(BCAP_VARIANT vnt0);
    std::vector<double> RadVectorFromVNT(BCAP_VARIANT vnt0);
    BCAP_VARIANT VNTFromVector(std::vector<double> vect0);
    BCAP_VARIANT VNTFromRadVector(std::vector<double> vect0);

    // class variables
    const char* server_ip_address;
    int server_port_num;
    int iSockFD;
    uint32_t lhController;
    uint32_t lhRobot;

    int current_waypoint_index;

private:
    // The purpose of this variable is two-fold:
    //   1. The _runForceSensingLoop function only runs while this variable is false.
    //   2. The _runForceSensingLoop function sets this variable to true if the force limit is exceeded.
    std::atomic<bool> _force_limit_exceeded;

    // Runs force sensing loop. Should be used in a separate thread.
    // This function runs while _force_limit_exceeded is false.
    // If the force limit is exceeded, _force_limit_exceeded is set to true.
    void _runForceSensingLoop(
        double totalForceLimit,
        double totalTorqueLimit,
        std::vector<double> tcpForceTorqueLimit
    );
};


class DensoArmMutex {
public:
    DensoArmMutex(DensoController &controller);
    ~DensoArmMutex();

private:
    DensoController &_controller;
};


////////////////////////////// Utilities //////////////////////////////
std::vector<double> VRad2Deg(std::vector<double> vect0);
std::vector<double> VDeg2Rad(std::vector<double> vect0);

double Rad2Deg(double x);
double Deg2Rad(double x);

} // close namespace denso_controller

#endif // DensoController_hpp
