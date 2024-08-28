/**
 * C++ Wrapper around Denso b-CAP library
 * Based on https://github.com/quangounet/denso/blob/master/cpp/src/DensoController.cpp
 */ 

#include "b-Cap.h"
#include "DensoController.hpp"
#include <iostream>
#include <math.h>
#include <time.h>

// start realtime headers
#include <iostream>
#include <sched.h>
#include <sys/resource.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>


namespace denso_controller {

DensoArmMutex::DensoArmMutex(DensoController &controller) : _controller(controller) {
    _controller.bCapRobotExecute("TakeArm", "");
    _controller.bCapRobotExecute("Motor", "1");
}

DensoArmMutex::~DensoArmMutex() {
    _controller.bCapRobotExecute("GiveArm", "");
}


DensoController::DensoController() {
    server_ip_address = DEFAULT_SERVER_IP_ADDRESS;
    server_port_num = DEFAULT_SERVER_PORT_NUM;
    iSockFD = 0;
    lhController = 0;
    lhRobot = 0;
    current_waypoint_index = 0;
}

////////////////////////////// Low Level Commands //////////////////////////////

void DensoController::bCapOpen() {
    std::cout << "\033[1;32mInitialize and start b-CAP.\033[0m\n";
    BCAP_HRESULT hr = bCap_Open(server_ip_address, server_port_num, &iSockFD);
    if FAILED(hr) {
        throw bCapException("\033[1;31mbCap_Open failed.\033[0m");
    }
}

void DensoController::bCapClose() {
    std::cout << "\033[1;32mStop b-CAP.\033[0m\n";
    BCAP_HRESULT hr = bCap_Close(iSockFD);
    if FAILED(hr) {
        throw bCapException("\033[1;31mbCap_Close failed.\033[0m");
    }
}

void DensoController::bCapServiceStart() {
    std::cout << "\033[1;32mStart b-CAP service.\033[0m\n";
    BCAP_HRESULT hr = bCap_ServiceStart(iSockFD);
    if FAILED(hr) {
        throw bCapException("\033[1;31mbCap_ServiceStart failed.\033[0m");
    }
}

void DensoController::bCapServiceStop() {
    std::cout << "\033[1;32mStop b-CAP service.\033[0m\n";
    BCAP_HRESULT hr = bCap_ServiceStop(iSockFD);
    if FAILED(hr) {
        throw bCapException("\033[1;31mbCap_ServiceStop failed.\033[0m");
    }
}

void DensoController::bCapControllerConnect() {
    std::cout << "Getting controller handle. Server ip address: " << server_ip_address << std::endl;
    BCAP_HRESULT hr = bCap_ControllerConnect(iSockFD, "b-CAP", "caoProv.DENSO.VRC9", server_ip_address, "", &lhController);
    if FAILED(hr) {
        throw bCapException("\033[1;31mbCap_ControllerConnect failed.\033[0m");
    }
}

void DensoController::bCapControllerDisconnect() {
    std::cout << "Release controller handle.\n";
    BCAP_HRESULT hr = bCap_ControllerDisconnect(iSockFD, lhController);
    if FAILED(hr) {
        throw bCapException("\033[1;31mbCap_ControllerDisconnect failed.\033[0m");
    }
}

void DensoController::bCapGetRobot() {
    std::cout << "Get robot handle.\n";
    BCAP_HRESULT hr = bCap_ControllerGetRobot(iSockFD, lhController, "Arm", "", &lhRobot);
    if FAILED(hr) {
        throw bCapException("\033[1;31mbCap_ControllerGetRobot failed.\033[0m");
    }
}

void DensoController::bCapReleaseRobot() {
    std::cout << "Release robot handle.\n";
    BCAP_HRESULT hr = bCap_RobotRelease(iSockFD, lhRobot);
    if FAILED(hr) {
        throw bCapException("\033[1;31mbCap_RobotRelease failed.\033[0m");
    }
}

BCAP_HRESULT DensoController::bCapClearError() {
    long lResult;
    BCAP_HRESULT hr = bCap_ControllerExecute(iSockFD, lhController, "ClearError", "", &lResult);
    return hr;
}

BCAP_HRESULT DensoController::bCapRobotExecute(const char* command, const char* option) {
    long lResult;
    BCAP_HRESULT hr = bCap_RobotExecute(iSockFD, lhRobot, command, option, &lResult);
    return hr;
}

BCAP_HRESULT DensoController::bCapRobotMove(const char* pose, const char* option) {
    BCAP_HRESULT hr = bCap_RobotMove(iSockFD, lhRobot, 1L, pose, option);
    return hr;
}

BCAP_HRESULT DensoController::bCapMotor(bool command) {
    BCAP_HRESULT hr;
    if (command) {
        std::cout << "\033[1;33mTurn motor on.\033[0m\n";
        hr = bCapRobotExecute("Motor", "1");
    }
    else{
        std::cout << "\033[1;33mTurn motor off.\033[0m\n";
        hr = bCapRobotExecute("Motor", "0");
    }
    return hr;
}

BCAP_HRESULT DensoController::bCapSlvChangeMode(const char* mode) {
    BCAP_HRESULT hr = bCapRobotExecute("slvChangeMode", mode);
    if SUCCEEDED(hr) {
        // long lResult;
        // BCAP_HRESULT hr1 = bCap_RobotExecute(iSockFD, lhRobot, "slvGetMode", "", &lResult);
        // if (lResult > 512) {
        //     std::cout << "Changed to mode 2 ";
        //     if (lResult == 513) std::cout << "P-type.\n";
        //     if (lResult == 514) std::cout << "J-type.\n";
        //     if (lResult == 515) std::cout << "T-type.\n";
        // }
        // else if (lResult > 256) {
        //     std::cout << "Changed to mode 1 ";
        //     if (lResult == 257) std::cout << "P-type.\n";
        //     if (lResult == 258) std::cout << "J-type.\n";
        //     if (lResult == 259) std::cout << "T-type.\n";
        // }
        // else if (lResult > 0) {
        //     std::cout << "Changed to mode 0 ";
        //     if (lResult == 1) std::cout << "P-type.\n";
        //     if (lResult == 2) std::cout << "J-type.\n";
        //     if (lResult == 3) std::cout << "T-type.\n";
        // }
        // else {
        //     std::cout << "Released slave mode.\n";
        // }
    }
    return hr;
}

BCAP_HRESULT DensoController::bCapSlvMove(BCAP_VARIANT* pose, BCAP_VARIANT* result) {
    BCAP_HRESULT hr;
    hr = bCap_RobotExecute2(iSockFD, lhRobot, "slvMove", pose, result);
    if (FAILED(hr)) {
        std::cerr << "Failed to execute b-CAP slave move.\n";
    }
    return hr;
}

BCAP_HRESULT DensoController::SetExtSpeed(const char* speed) {
    BCAP_HRESULT hr;
    hr = bCapRobotExecute("ExtSpeed", speed);
    if SUCCEEDED(hr) {
        std::cout << "External speed is set to " << speed << std::endl;
    }
    return hr;
}

BCAP_HRESULT DensoController::ManualReset() {
    BCAP_HRESULT hr = BCAP_S_OK;
    long lResult;
    hr = bCap_ControllerExecute(iSockFD, lhController, "ManualReset", "", &lResult);
    if SUCCEEDED(hr) {
        std::cout << "Executed Manual Reset" << std::endl;
    }
    return hr;
}

BCAP_HRESULT DensoController::SetTcpLoad(const int32_t tool_value) {
    // Our Robotiq grippers
    // tool_value = 1; // ROBOTIQ_2F85_GRIPPER_PAYLOAD
    // tool_value = 2; // ROBOTIQ_2F140_GRIPPER_PAYLOAD
    long lValue = tool_value;
    auto arm_mutex = DensoArmMutex(*this);

    BCAP_HRESULT hr = BCAP_S_OK;
    uint32_t lhVar;
    hr = bCap_RobotGetVariable(iSockFD, lhRobot, "@CURRENT_TOOL", "", &lhVar);   /* Get var handle */
    if FAILED(hr) {
        std::cerr << "Set TCP Load failed to get @CURRENT_TOOL variable" << std::endl;
        return hr;
    }

    hr = bCap_VariablePutValue(iSockFD, lhVar, VT_I4, 1, &lValue);      /* Put Value */
    if FAILED(hr) {
        std::cerr << "Set TCP Load failed to put value" << std::endl;
    }

    hr = bCap_VariableRelease(iSockFD, lhVar);	/* Release var handle*/
    if FAILED(hr) {
        std::cerr << "Set TCP Load failed to release variable" << std::endl;
    }

    return hr;
}

BCAP_HRESULT DensoController::ChangeTool(char* tool_name) {
    // tool_name = "Tool1";
    // tool_name = "Tool2";

    long lResult;
    auto arm_mutex = DensoArmMutex(*this);

    BCAP_HRESULT hr;
    hr = bCap_RobotChange(iSockFD, lhRobot, tool_name); /* Change Tool */
    if SUCCEEDED(hr) {
        std::cout << "Tool changed to " << tool_name << std::endl;
    }
    else {
        std::cerr << "Failed to change tool" << std::endl;
    }

    return hr;
}

/* Populates mounting_calib with the offset from the specified work coordinate.
 */
std::tuple<BCAP_HRESULT, std::vector<double>> DensoController::GetMountingCalib(const char* work_coordinate) {
    double work_def[8]; // Should this be 6?
    std::vector<double> mounting_calib = std::vector<double>(8);

    BCAP_HRESULT hr = BCAP_S_OK;
    hr = bCap_RobotExecute(iSockFD, lhRobot, "getWorkDef", work_coordinate, &work_def);
    if FAILED(hr) {
        std::cerr << "Failed to get mounting calibration" << std::endl;
        return {hr, mounting_calib};
    }

    mounting_calib.resize(0);
    for (int i = 0; i < 8; i++) {
        mounting_calib.push_back(work_def[i]);
    }
    std::cout << "Got Mounting Calibration: (" << mounting_calib[0] << ", " 
                                               << mounting_calib[1] << ", " 
                                               << mounting_calib[2] << ", " 
                                               << mounting_calib[3] << ", " 
                                               << mounting_calib[4] << ", " 
                                               << mounting_calib[5] << ")" << std::endl;
    return {hr, mounting_calib};
}

std::string DensoController::GetErrorDescription(BCAP_HRESULT error_code) {
    char err_code_str[32];
    sprintf(err_code_str, "%d", error_code);
    char error_description[1024] = {0}; // Denso defined local receive buffer LOCALRECBUFFER_SZ = 1024

    BCAP_HRESULT hr = BCAP_S_OK;
    hr = bCap_ControllerExecute(iSockFD, lhController, "GetErrorDescription", err_code_str, error_description);
    if FAILED(hr) {
        return "Failed to get error description";
    }
    return std::string(error_description);
}

////////////////////////////// High Level Commands //////////////////////////////

void DensoController::bCapEnterProcess() {
    // Only set priority on Linux machines
    // #ifdef __linux__
    #ifdef false
        // start setup realtime
        // Set process priority (nice value)
        int priority = -9;
        int result = setpriority(PRIO_PROCESS, 0, priority);
        if (result == -1) {
            std::cerr << "Failed to set priority: " << strerror(errno) << std::endl;
            throw bCapException("Failed to set scheduler priority");
        }
        // Set scheduler to FIFO
        struct sched_param param;
        param.sched_priority = sched_get_priority_max(SCHED_FIFO);
        result = sched_setscheduler(0, SCHED_FIFO, &param);
        if (result == -1) {
            std::cerr << "Failed to set scheduler: " << strerror(errno) << std::endl;
            throw bCapException("Failed to change scheduler");
        }
        // end setup realtime
    #endif

    BCAP_HRESULT hr;

    bCapOpen();
    std::cout << "b-Cap port opened" << std::endl;
    bCapServiceStart();
    std::cout << "b-Cap service started" << std::endl;
    bCapControllerConnect();
    std::cout << "Connected to controller" << std::endl;
    bCapGetRobot();
    std::cout << "Obtained robot handle" << std::endl;

    long lResult;
    hr = bCap_ControllerExecute(iSockFD, lhController, "ClearError", "", &lResult);
    if FAILED(hr) {
        std::string err_description = GetErrorDescription(hr);
        std::cerr << "\033[1;31mClearError failed: " << err_description << "\033[0m" << std::endl;
        bCapExitProcess();
        throw bCapException("\033[1;31mFail to clear error.\033[0m", err_description);
    }

    hr = ManualReset();
    if FAILED(hr) {
        std::string err_description = GetErrorDescription(hr);
        std::cerr << "\033[1;31mManualReset failed: " << err_description << "\033[0m" << std::endl;
        bCapExitProcess();
        throw bCapException("\033[1;31mFail to execute manual reset.\033[0m", err_description);
    }

    auto arm_mutex = DensoArmMutex(*this);
    hr = bCapMotor(true);
    if FAILED(hr) {
        std::string err_description = GetErrorDescription(hr);
        std::cerr << "\033[1;31mMotorOn failed: " << err_description << "\033[0m" << std::endl;
        bCapExitProcess();
        throw bCapException("\033[1;31mFail to turn motor on.\033[0m", err_description);
    }
    current_waypoint_index = 0;
}

void DensoController::bCapExitProcess() {
    BCAP_HRESULT hr;
    hr = bCapMotor(false);
    if FAILED(hr) {
        std::cerr << "Fail to turn off motor" << std::endl;
    }

    bCapReleaseRobot();
    bCapControllerDisconnect();
    bCapServiceStop();
    bCapClose();
}

/**
 * Command Servo Joints
 * Send a SlaveMode move to joint position command to the robot.
 *
 * @param joint_position Vector of 8 DOF joint positions in Radians.
 * @return 0 if successful, 1 otherwise.
 */
void DensoController::CommandServoJoint(const std::vector<double> joint_position) {
    BCAP_HRESULT hr = BCAP_S_OK;
    BCAP_VARIANT vntPose, vntReturn;
    vntPose = VNTFromRadVector(joint_position);
    hr = bCapSlvMove(&vntPose, &vntReturn);
    if (FAILED(hr)) {
        std::string err_description = GetErrorDescription(hr);
        throw bCapException("Failed CommandServoJoint slave move", err_description);
    }

    // Print the joint positions
    // std::cout << "slvmove (";
    // for (int i=0; i<joint_position.size(); ++i)
    //     std::cout << joint_position[i] << ' ';
    // std::cout << ")" << std::endl;
    
}

void DensoController::ExecuteServoTrajectory(RobotTrajectory& traj)
{
    BCAP_HRESULT hr;
    long lResult;

    auto arm_mutex = DensoArmMutex(*this);

    // Enter slave mode: mode 2 J-Type
    hr = bCapSlvChangeMode("514");
    if (FAILED(hr)) {
        std::cerr << "Failed to enter b-CAP slave mode." << std::endl;
        std::string err_description = GetErrorDescription(hr);
        throw EnterSlaveModeException(err_description);
    }
    // std::cout << "Slave mode ON" << std::endl;

    // Execute the trajectory
    BCAP_VARIANT vntPose, vntReturn;
    for (size_t i = 0; i < traj.size(); i++) {
        current_waypoint_index = i;
        const auto& joint_position = traj.trajectory[i];
        vntPose = VNTFromRadVector(joint_position);
        hr = bCapSlvMove(&vntPose, &vntReturn);

        if (FAILED(hr)) {
            std::cerr << "Failed to execute b-CAP slave move." << std::endl;
            
            std::string err_description = GetErrorDescription(hr);
            std::string msg = "Index " + std::to_string(i) + " of " + std::to_string(traj.size())
                + " ErrDescription: " + err_description;
            throw SlaveMoveException(msg);
        }

        // Print the joint positions
        // std::cout << "slvmove (";
        // for (int i=0; i<joint_position.size(); ++i)
        //     std::cout << joint_position[i] << ' ';
        // std::cout << ")" << std::endl;
    }
    // std::cout << "Exec traj done" << std::endl;

    // Exit slave mode
    hr = bCapSlvChangeMode("0");
    if (FAILED(hr)) {
        std::cerr << "Failed to exit b-CAP slave mode." << std::endl;
        std::string err_description = GetErrorDescription(hr);
        throw ExitSlaveModeException(err_description);
    }
    // std::cout << "Slave mode OFF" << std::endl;

}


////////////////////////////// Utilities //////////////////////////////

const char* DensoController::CommandFromVector(std::vector<double> q) {
    std::vector<double> tmp;
    tmp = VRad2Deg(q);
    std::string commandstring;
    commandstring = "J(" + std::to_string(tmp[0]) + ", " + std::to_string(tmp[1])
                    + ", " + std::to_string(tmp[2]) + ", " + std::to_string(tmp[3])
                    + ", " + std::to_string(tmp[4]) + ", " + std::to_string(tmp[5]) + ")";
    return commandstring.c_str(); // convert string -> const shar*
}

/* Populates jnt with the current joint values in degrees.
 */
std::tuple<BCAP_HRESULT, std::vector<double>> DensoController::GetCurJnt() {
    BCAP_HRESULT hr;
    double dJnt[8];
    std::vector<double> jnt(8);

    hr = bCap_RobotExecute(iSockFD, lhRobot, "CurJnt", "", &dJnt);
    if FAILED(hr) {
        std::cout << "\033[1;31mFail to get current joint values.\033[0m\n";
        return {hr, jnt};
    }
    for (int i = 0; i < 8; i++) {
        jnt[i] = dJnt[i];
    }
    return {hr, jnt};
}

std::vector<double> DensoController::VectorFromVNT(BCAP_VARIANT vnt0) {
    std::vector<double> vect;
    vect.resize(0);
    for (int i = 0; i < 8; i++) {
        vect.push_back(vnt0.Value.DoubleArray[i]);
    }
    return vect;
}

BCAP_VARIANT DensoController::VNTFromVector(std::vector<double> vect0) {
    assert(vect0.size() == 6 || vect0.size() == 8);
    BCAP_VARIANT vnt;
    vnt.Type = VT_R8 | VT_ARRAY;
    vnt.Arrays = 8;

    for (int i = 0; i < int(vect0.size()); i++) {
        vnt.Value.DoubleArray[i] = vect0[i];
    }
    return vnt;
}

std::vector<double> DensoController::RadVectorFromVNT(BCAP_VARIANT vnt0) {
    std::vector<double> vect;
    vect.resize(0);
    for (int i = 0; i < 8; i++) {
        vect.push_back(Deg2Rad(vnt0.Value.DoubleArray[i]));
    }
    return vect;
}

BCAP_VARIANT DensoController::VNTFromRadVector(std::vector<double> vect0) {
    assert(vect0.size() == 6 || vect0.size() == 8);
    BCAP_VARIANT vnt;
    vnt.Type = VT_R8 | VT_ARRAY;
    vnt.Arrays = 8;

    for (int i = 0; i < int(vect0.size()); i++) {
        vnt.Value.DoubleArray[i] = Rad2Deg(vect0[i]);
    }
    return vnt;
}

std::vector<double> VRad2Deg(std::vector<double> vect0) {
    std::vector<double> resvect;
    resvect.resize(0);
    for (int i = 0; i < int(vect0.size()); i++) {
        resvect.push_back(Rad2Deg(vect0[i]));
    }
    return resvect;
}

double Rad2Deg(double x) {
    double res = x * 180.0 / PI;
    if (res >= 0) {
        return fmod(res, 360.0);
    }
    else{
        return -1.0 * fmod(-1.0 * res, 360.0);
    }
}

double Deg2Rad(double x) {
    // double res;
    if (x >= 0) {
        return fmod(x, 360.0) * PI / 180.0;
    }
    else{
        return -1.0 * fmod(-1.0 * x, 360.0) * PI / 180;
    }
}

} // close namespace denso_controller
