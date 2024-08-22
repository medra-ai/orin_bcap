/**
 * C++ Wrapper around Denso b-CAP library
 * Based on https://github.com/quangounet/denso/blob/master/cpp/src/DensoController.h
 */ 

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

class DensoController {

public:
    DensoController();

    // low level commands
    void bCapOpen();
    void bCapClose();
    void bCapServiceStart();
    void bCapServiceStop();
    void bCapControllerConnect();
    void bCapControllerDisconnect();
    void bCapGetRobot();
    void bCapReleaseRobot();
    void bCapClearError();
    BCAP_HRESULT bCapRobotExecute(const char* command, const char* option);
    BCAP_HRESULT bCapRobotMove(const char* pose, const char* option);
    BCAP_HRESULT bCapMotor(bool command);
    BCAP_HRESULT bCapSlvChangeMode(const char* mode);
    BCAP_HRESULT bCapSlvMove(BCAP_VARIANT* pose, BCAP_VARIANT* result);
    BCAP_HRESULT SetExtSpeed(const char* speed);
    BCAP_HRESULT ManualReset();
    BCAP_HRESULT SetTcpLoad(const int32_t tool_value);
    BCAP_HRESULT ChangeTool(char* tool_name); // Alternative to SetTcpLoad?
    std::tuple<BCAP_HRESULT, std::vector<double>> GetMountingCalib(const char* work_coordinate);
    std::string GetErrorDescription(const char* error_code);

    // high level commands
    void bCapEnterProcess();
    void bCapExitProcess();
    void CommandServoJoint(const std::vector<double> joint_position);
    void ExecuteServoTrajectory(RobotTrajectory& traj);

    // utilities
    const char* CommandFromVector(std::vector<double> q);
    std::tuple<BCAP_HRESULT, std::vector<double>> GetCurJnt();
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

double Rad2Deg(double x);
double Deg2Rad(double x);

} // close namespace denso_controller
