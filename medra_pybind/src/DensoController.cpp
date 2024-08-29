/**
 * C++ Wrapper around Denso b-CAP library
 * Based on https://github.com/quangounet/denso/blob/master/cpp/src/DensoController.cpp
 */ 

#include "b-Cap.h"
#include "DensoController.hpp"

#include <iostream>
#include <math.h>
#include <time.h>

#include <spdlog/spdlog.h>

// start realtime headers
#include <iostream>
#include <sched.h>
#include <sys/resource.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <cstdio>


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
    SPDLOG_INFO("Initialize and start b-CAP.");
    BCAP_HRESULT hr = bCap_Open(server_ip_address, server_port_num, &iSockFD);
    if FAILED(hr) {
        throw bCapException("\033[1;31mbCap_Open failed.\033[0m");
    }
}

void DensoController::bCapClose() {
    SPDLOG_INFO("Stop b-CAP.");
    BCAP_HRESULT hr = bCap_Close(iSockFD);
    if FAILED(hr) {
        throw bCapException("\033[1;31mbCap_Close failed.\033[0m");
    }
}

void DensoController::bCapServiceStart() {
    SPDLOG_INFO("Start b-CAP service.");
    BCAP_HRESULT hr = bCap_ServiceStart(iSockFD);
    if FAILED(hr) {
        throw bCapException("\033[1;31mbCap_ServiceStart failed.\033[0m");
    }
}

void DensoController::bCapServiceStop() {
    SPDLOG_INFO("Stop b-CAP service.");
    BCAP_HRESULT hr = bCap_ServiceStop(iSockFD);
    if FAILED(hr) {
        throw bCapException("\033[1;31mbCap_ServiceStop failed.\033[0m");
    }
}

void DensoController::bCapControllerConnect() {
    SPDLOG_INFO("Getting controller handle. Server ip address: " + std::string(server_ip_address));
    BCAP_HRESULT hr = bCap_ControllerConnect(iSockFD, "b-CAP", "caoProv.DENSO.VRC9", server_ip_address, "", &lhController);
    if FAILED(hr) {
        throw bCapException("\033[1;31mbCap_ControllerConnect failed.\033[0m");
    }
}

void DensoController::bCapControllerDisconnect() {
    SPDLOG_INFO("Release controller handle.");
    BCAP_HRESULT hr = bCap_ControllerDisconnect(iSockFD, lhController);
    if FAILED(hr) {
        throw bCapException("\033[1;31mbCap_ControllerDisconnect failed.\033[0m");
    }
}

void DensoController::bCapGetRobot() {
    SPDLOG_INFO("Get robot handle.");
    BCAP_HRESULT hr = bCap_ControllerGetRobot(iSockFD, lhController, "Arm", "", &lhRobot);
    if FAILED(hr) {
        throw bCapException("\033[1;31mbCap_ControllerGetRobot failed.\033[0m");
    }
}

void DensoController::bCapReleaseRobot() {
    SPDLOG_INFO("Release robot handle.");
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
        SPDLOG_INFO("Turn motor on.");
        hr = bCapRobotExecute("Motor", "1");
    }
    else{
        SPDLOG_INFO("Turn motor off.");
        hr = bCapRobotExecute("Motor", "0");
    }
    return hr;
}

BCAP_HRESULT DensoController::bCapSlvChangeMode(const char* mode) {
    BCAP_HRESULT hr = bCapRobotExecute("slvChangeMode", mode);
    if FAILED(hr) {
        throw bCapException("\033[1;31mbCap_SlvChangeMode failed.\033[0m");
    }
    return hr;
}

BCAP_HRESULT DensoController::printSlvMode() {
    long lResult;
    BCAP_HRESULT hr = bCap_RobotExecute(iSockFD, lhRobot, "slvGetMode", "", &lResult);
    if (lResult > 512) {
        SPDLOG_INFO("Slave mode 2 ");
        if (lResult == 513) SPDLOG_INFO("P-type");
        if (lResult == 514) SPDLOG_INFO("J-type");
        if (lResult == 515) SPDLOG_INFO("T-type");
    }
    else if (lResult > 256) {
        SPDLOG_INFO("Slave mode 1 ");
        if (lResult == 257) SPDLOG_INFO("P-type");
        if (lResult == 258) SPDLOG_INFO("J-type");
        if (lResult == 259) SPDLOG_INFO("T-type");
    }
    else if (lResult > 0) {
        SPDLOG_INFO("Slave mode 0 ");
        if (lResult == 1) SPDLOG_INFO("P-type");
        if (lResult == 2) SPDLOG_INFO("J-type");
        if (lResult == 3) SPDLOG_INFO("T-type");
    }
    else {
        SPDLOG_INFO("Released slave mode.");
    }
    return hr;
}

BCAP_HRESULT DensoController::bCapSlvMove(BCAP_VARIANT* pose, BCAP_VARIANT* result) {
    BCAP_HRESULT hr;
    hr = bCap_RobotExecute2(iSockFD, lhRobot, "slvMove", pose, result);
    if (FAILED(hr)) {
        SPDLOG_ERROR("Failed to execute b-CAP slave move.");
    }
    return hr;
}

BCAP_HRESULT DensoController::SetExtSpeed(const char* speed) {
    BCAP_HRESULT hr;
    hr = bCapRobotExecute("ExtSpeed", speed);
    if SUCCEEDED(hr) {
        SPDLOG_INFO("External speed is set to " + std::string(speed));
    }
    return hr;
}

BCAP_HRESULT DensoController::ManualReset() {
    BCAP_HRESULT hr = BCAP_S_OK;
    long lResult;
    hr = bCap_ControllerExecute(iSockFD, lhController, "ManualReset", "", &lResult);
    if SUCCEEDED(hr) {
        SPDLOG_INFO("Executed Manual Reset");
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
        SPDLOG_ERROR("Set TCP Load failed to get @CURRENT_TOOL variable");
        return hr;
    }

    hr = bCap_VariablePutValue(iSockFD, lhVar, VT_I4, 1, &lValue);      /* Put Value */
    if FAILED(hr) {
        SPDLOG_ERROR("Set TCP Load failed to put value");
    }

    hr = bCap_VariableRelease(iSockFD, lhVar);	/* Release var handle*/
    if FAILED(hr) {
        SPDLOG_ERROR("Set TCP Load failed to release variable");
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
        SPDLOG_INFO("Tool changed to " + std::string(tool_name));
    }
    else {
        SPDLOG_ERROR("Failed to change tool");
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
        SPDLOG_ERROR("Failed to get mounting calibration");
        return {hr, mounting_calib};
    }

    mounting_calib.resize(0);
    for (int i = 0; i < 8; i++) {
        mounting_calib.push_back(work_def[i]);
    }
    SPDLOG_INFO("Got Mounting Calibration: (" + std::to_string(mounting_calib[0]) + ", " 
                                               + std::to_string(mounting_calib[1]) + ", " 
                                               + std::to_string(mounting_calib[2]) + ", " 
                                               + std::to_string(mounting_calib[3]) + ", " 
                                               + std::to_string(mounting_calib[4]) + ", " 
                                               + std::to_string(mounting_calib[5]) + ")");
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
            SPDLOG_ERROR("Failed to set priority: " + std::string(strerror(errno)));
            throw bCapException("Failed to set scheduler priority");
        }
        // Set scheduler to FIFO
        struct sched_param param;
        param.sched_priority = sched_get_priority_max(SCHED_FIFO);
        result = sched_setscheduler(0, SCHED_FIFO, &param);
        if (result == -1) {
            SPDLOG_ERROR("Failed to set scheduler: " + std::string(strerror(errno)));
            throw bCapException("Failed to change scheduler");
        }
        // end setup realtime
    #endif

    BCAP_HRESULT hr;

    bCapOpen();
    SPDLOG_INFO("b-Cap port opened");
    bCapServiceStart();
    SPDLOG_INFO("b-Cap service started");
    bCapControllerConnect();
    SPDLOG_INFO("Connected to controller");
    bCapGetRobot();
    SPDLOG_INFO("Obtained robot handle");

    long lResult;
    hr = bCap_ControllerExecute(iSockFD, lhController, "ClearError", "", &lResult);
    if FAILED(hr) {
        std::string err_description = GetErrorDescription(hr);
        SPDLOG_ERROR("ClearError failed: " + err_description);
        bCapExitProcess();
        throw bCapException("Fail to clear error.", err_description);
    }

    hr = ManualReset();
    if FAILED(hr) {
        std::string err_description = GetErrorDescription(hr);
        SPDLOG_ERROR("ManualReset failed: " + err_description);
        bCapExitProcess();
        throw bCapException("Fail to execute manual reset.", err_description);
    }

    auto arm_mutex = DensoArmMutex(*this);
    hr = bCapMotor(true);
    if FAILED(hr) {
        std::string err_description = GetErrorDescription(hr);
        SPDLOG_ERROR("MotorOn failed: " + err_description);
        bCapExitProcess();
        throw bCapException("Fail to turn motor on.", err_description);
    }
    current_waypoint_index = 0;
}

void DensoController::bCapExitProcess() {
    BCAP_HRESULT hr;
    hr = bCapMotor(false);
    if FAILED(hr) {
        SPDLOG_ERROR("Fail to turn off motor");
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
    // std::string msg = "slvmove (";
    // for (int i=0; i<joint_position.size(); ++i)
    //     msg += std::to_string(joint_position[i]) + ' ';
    // msg += ")";
    // SPDLOG_INFO(msg);
}

void DensoController::ExecuteServoTrajectory(RobotTrajectory& traj)
{
    BCAP_HRESULT hr;
    long lResult;

    auto arm_mutex = DensoArmMutex(*this);

    // Enter slave mode: mode 2 J-Type
    hr = bCapSlvChangeMode("514");
    if (FAILED(hr)) {
        SPDLOG_ERROR("Failed to enter b-CAP slave mode.");
        std::string err_description = GetErrorDescription(hr);
        throw EnterSlaveModeException(err_description);
    }
    // SPDLOG_INFO("Slave mode ON");

    // Execute the trajectory
    BCAP_VARIANT vntPose, vntReturn;
    for (size_t i = 0; i < traj.size(); i++) {
        current_waypoint_index = i;
        const auto& joint_position = traj.trajectory[i];
        vntPose = VNTFromRadVector(joint_position);
        hr = bCapSlvMove(&vntPose, &vntReturn);

        if (FAILED(hr)) {
            SPDLOG_ERROR("Failed to execute b-CAP slave move.");
            std::string err_description = GetErrorDescription(hr);
            std::string msg = "Index " + std::to_string(i) + " of " + std::to_string(traj.size())
                + " ErrDescription: " + err_description;
            throw SlaveMoveException(msg);
        }

        // Print the joint positions
        // std::string msg = "slvmove (";
        // for (int i=0; i<joint_position.size(); ++i)
        //     msg += std::to_string(joint_position[i]) + ' ';
        // msg += ")";
        // SPDLOG_INFO(msg);
    }

    // Close loop servo commands on last waypoint
    ClosedLoopCommandServoJoint(traj.trajectory.back());

    // SPDLOG_INFO("Exec traj done");

    // Exit slave mode
    hr = bCapSlvChangeMode("0");
    if (FAILED(hr)) {
        SPDLOG_ERROR("Failed to exit b-CAP slave mode.");
        std::string err_description = GetErrorDescription(hr);
        throw ExitSlaveModeException(err_description);
    }
    // SPDLOG_INFO("Slave mode OFF");

}

void DensoController::ClosedLoopCommandServoJoint(std::vector<double> last_waypoint) {
    /* Repeat the last servo j command until the robot reaches tolerance or timeout */
    double CLOSE_LOOP_JOINT_ANGLE_TOLERANCE = 0.0001;  // rad
    double CLOSE_LOOP_TIMEOUT = 0.1;  // 100ms

    BCAP_HRESULT hr;
    std::vector<double> current_jnt;
    std::tie(hr, current_jnt) = GetCurJnt();
    if (FAILED(hr)) {
        SPDLOG_ERROR("Closed loop servo j commands failed to get initial joint position");
    }
    
    // Print the initial joint error
    std::vector<double> joint_error;
    for (int i = 0; i < current_jnt.size(); ++i) {
        joint_error.push_back(current_jnt[i] - last_waypoint[i]);
    }
    char buffer[256] = {0};
    std::sprintf(buffer, "Before closed-loop servo commands, joint error: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                joint_error[0], joint_error[1], joint_error[2], joint_error[3], joint_error[4], joint_error[5]);
    SPDLOG_INFO(std::string(buffer));
    std::cout << std::string(buffer) << std::endl;

    auto initial_time = std::chrono::steady_clock::now();
    int count = 0;
    while (true) {
        // Check Timeout
        auto current_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - initial_time).count();
        if (duration > CLOSE_LOOP_TIMEOUT) {
            SPDLOG_DEBUG("Closed loop servo j commands TIMEOUT");
            break;
        }

        // Check Joint Tolerance
        std::tie(hr, current_jnt) = GetCurJnt();
        if (FAILED(hr)) {
            SPDLOG_ERROR("Closed loop servo j commands failed to get current joint position");
            break;
        }
        bool all_within_tolerance = true;
        for (int i = 0; i < current_jnt.size(); ++i) {
            if (std::abs(current_jnt[i] - last_waypoint[i]) > CLOSE_LOOP_JOINT_ANGLE_TOLERANCE) {
                all_within_tolerance = false;
                break;
            }
        }
        if (all_within_tolerance) {
            break;
        }

        // Command the last waypoint again
        CommandServoJoint(last_waypoint);
        count++;
    }

    // Print the joint remaining joint error
    joint_error.clear();
    for (int i = 0; i < current_jnt.size(); ++i) {
        joint_error.push_back(current_jnt[i] - last_waypoint[i]);
    }
    std::memset(buffer, 0, sizeof(buffer));
    std::sprintf(buffer, "After %d closed-loop servo commands, joint error: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                count, joint_error[0], joint_error[1], joint_error[2], joint_error[3], joint_error[4], joint_error[5]);
    SPDLOG_INFO(std::string(buffer));
    std::cout << std::string(buffer) << std::endl;

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
        SPDLOG_ERROR("Fail to get current joint values.");
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
