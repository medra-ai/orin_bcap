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

DensoArmMutex::DensoArmMutex(DensoReadWriteDriver &driver) : driver(driver) {
    driver.TakeArm();
    driver.Motor(true);
}

DensoArmMutex::~DensoArmMutex() {
    driver.GiveArm();
}


DensoReadDriver::DensoReadDriver() {
    server_ip_address = DEFAULT_SERVER_IP_ADDRESS;
    server_port_num = DEFAULT_SERVER_PORT_NUM;
    iSockFD = 0;
    lhController = 0;
    lhRobot = 0;

    session_name = "read-only";
}

void DensoReadDriver::bCapOpen() {
    SPDLOG_INFO("Initialize and start b-CAP.");
    BCAP_HRESULT hr = bCap_Open(server_ip_address, server_port_num, &iSockFD);
    if FAILED(hr) {
        throw bCapException("\033[1;31mbCap_Open failed.\033[0m");
    }
}

void DensoReadDriver::bCapClose() {
    SPDLOG_INFO("Stop b-CAP.");
    BCAP_HRESULT hr = bCap_Close(iSockFD);
    if FAILED(hr) {
        throw bCapException("\033[1;31mbCap_Close failed.\033[0m");
    }
}


void DensoReadDriver::bCapServiceStart() {
    SPDLOG_INFO("Start b-CAP service.");
    BCAP_HRESULT hr = bCap_ServiceStart(iSockFD);
    if FAILED(hr) {
        throw bCapException("\033[1;31mbCap_ServiceStart failed.\033[0m");
    }
}

void DensoReadDriver::bCapServiceStop() {
    SPDLOG_INFO("Stop b-CAP service.");
    BCAP_HRESULT hr = bCap_ServiceStop(iSockFD);
    if FAILED(hr) {
        throw bCapException("\033[1;31mbCap_ServiceStop failed.\033[0m");
    }
}

void DensoReadDriver::bCapControllerConnect() {
    SPDLOG_INFO("Getting controller handle. Server ip address: " + std::string(server_ip_address));
    BCAP_HRESULT hr = bCap_ControllerConnect(iSockFD, session_name, "caoProv.DENSO.VRC9", server_ip_address, "", &lhController);
    if FAILED(hr) {
        throw bCapException("\033[1;31mbCap_ControllerConnect failed.\033[0m");
    }
}

void DensoReadDriver::bCapControllerDisconnect() {
    SPDLOG_INFO("Release controller handle.");
    BCAP_HRESULT hr = bCap_ControllerDisconnect(iSockFD, lhController);
    if FAILED(hr) {
        throw bCapException("\033[1;31mbCap_ControllerDisconnect failed.\033[0m");
    }
}

void DensoReadDriver::bCapGetRobot() {
    SPDLOG_INFO("Get robot handle.");
    BCAP_HRESULT hr = bCap_ControllerGetRobot(iSockFD, lhController, "Arm", "", &lhRobot);
    if FAILED(hr) {
        throw bCapException("\033[1;31mbCap_ControllerGetRobot failed.\033[0m");
    }
}

void DensoReadDriver::bCapReleaseRobot() {
    SPDLOG_INFO("Release robot handle.");
    BCAP_HRESULT hr = bCap_RobotRelease(iSockFD, lhRobot);
    if FAILED(hr) {
        throw bCapException("\033[1;31mbCap_RobotRelease failed.\033[0m");
    }
}

/* Populates joint_positions with the current joint values.
   joint_positions is mutated into a vector of 8 doubles.
   The first 6 values are the joint angles in degrees, and the last 2 values
   are the positions of the auxiliary axes, or 0 if they are not used.
 */
BCAP_HRESULT DensoReadDriver::GetCurJnt(std::vector<double>& joint_positions) {
    double dJnt[8];
    BCAP_HRESULT hr;
    for (size_t attempt = 0; attempt < 3; ++attempt) {
        hr = bCap_RobotExecute(iSockFD, lhRobot, "CurJnt", "", &dJnt);
        if (SUCCEEDED(hr)) {
            break;
        }
        SPDLOG_WARN("Failed to get joint pos, attempt ", std::to_string(attempt));
    }
    if (FAILED(hr)) {
        SPDLOG_ERROR("Failed to get current joint values.");
        return hr;
    }

    joint_positions.resize(0);
    for (int i = 0; i < 8; i++) {
        joint_positions.push_back(dJnt[i]);
    }
    return hr;
}

/* Populates force_values with the current force values.
   force_values is mutated into a vector of 6 doubles.
   The first 3 values are the force values, and the last 3 values are the
   torque values.
 */
BCAP_HRESULT DensoReadDriver::GetForceValue(std::vector<double>& force_values) {
    double dForce[6];
    BCAP_HRESULT hr;
    for (size_t attempt = 0; attempt < 3; ++attempt) {
        hr = bCap_RobotExecute(iSockFD, lhRobot, "forceValue", "13", &dForce);
        if SUCCEEDED(hr) {
            break;
        }
        SPDLOG_WARN("Failed to get force value, attempt ", std::to_string(attempt));
    }
    if (FAILED(hr)) {
        SPDLOG_ERROR("Failed to get current force values.");
        return hr;
    }

    force_values.resize(0);
    for (int i = 0; i < 6; i++) {
        force_values.push_back(dForce[i]);
    }
    return hr;
}

/* Populates mounting_calib with the offset from the specified work coordinate.
 */
std::tuple<BCAP_HRESULT, std::vector<double>>
DensoReadDriver::GetMountingCalib(const char* work_coordinate) {
    double work_def[8]; // Should this be 6?
    std::vector<double> mounting_calib = std::vector<double>(8);

    BCAP_HRESULT hr = bCap_RobotExecute(iSockFD, lhRobot, "getWorkDef", work_coordinate, &work_def);
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

std::string DensoReadDriver::GetErrorDescription(BCAP_HRESULT error_code) {
    char err_code_str[32];
    sprintf(err_code_str, "%d", error_code);
    // Denso defined local receive buffer LOCALRECBUFFER_SZ = 1024
    char error_description[1024] = {0};

    BCAP_HRESULT hr = bCap_ControllerExecute(
        iSockFD,
        lhController,
        "GetErrorDescription",
        err_code_str,
        error_description
    );
    if FAILED(hr) {
        return "Failed to get error description";
    }
    return std::string(error_description);
}

DensoReadWriteDriver::DensoReadWriteDriver() {
    server_ip_address = DEFAULT_SERVER_IP_ADDRESS;
    server_port_num = DEFAULT_SERVER_PORT_NUM;
    iSockFD = 0;
    lhController = 0;
    lhRobot = 0;

    session_name = "write";
}

BCAP_HRESULT DensoReadWriteDriver::ClearError() {
    long lResult;
    BCAP_HRESULT hr = bCap_ControllerExecute(
        iSockFD, lhController, "ClearError", "", &lResult
    );
    return hr;
}

BCAP_HRESULT DensoReadWriteDriver::ManualReset() {
    long lResult;
    BCAP_HRESULT hr = bCap_ControllerExecute(
        iSockFD, lhController, "ManualReset", "", &lResult
    );
    if SUCCEEDED(hr) {
        SPDLOG_INFO("Executed Manual Reset");
    }
    return hr;
}

BCAP_HRESULT DensoReadWriteDriver::Motor(bool command) {
    BCAP_HRESULT hr;
    long lResult;
    if (command) {
        SPDLOG_INFO("Turn motor on.");
        hr = bCap_RobotExecute(iSockFD, lhRobot, "Motor", "1", &lResult);
    }
    else {
        SPDLOG_INFO("Turn motor off.");
        hr = bCap_RobotExecute(iSockFD, lhRobot, "Motor", "0", &lResult);
    }
    return hr;
}

BCAP_HRESULT DensoReadWriteDriver::TakeArm() {
    long lResult;
    SPDLOG_INFO("Take arm.");
    return bCap_RobotExecute(iSockFD, lhRobot, "TakeArm", "", &lResult);
}

BCAP_HRESULT DensoReadWriteDriver::GiveArm() {
    long lResult;
    SPDLOG_INFO("Give arm.");
    return bCap_RobotExecute(iSockFD, lhRobot, "GiveArm", "", &lResult);
}

BCAP_HRESULT DensoReadWriteDriver::SetExtSpeed(const char* speed) {
    long lResult;
    BCAP_HRESULT hr = bCap_RobotExecute(iSockFD, lhRobot, "ExtSpeed", speed, &lResult);
    if SUCCEEDED(hr) {
        SPDLOG_INFO("External speed is set to " + std::string(speed));
    }
    return hr;
}

BCAP_HRESULT DensoReadWriteDriver::SetTcpLoad(const int32_t tool_value) {
    // Our Robotiq grippers
    // tool_value = 1; // ROBOTIQ_2F85_GRIPPER_PAYLOAD
    // tool_value = 2; // ROBOTIQ_2F140_GRIPPER_PAYLOAD
    long lValue = tool_value;
    auto arm_mutex = DensoArmMutex(*this);

    BCAP_HRESULT hr = BCAP_S_OK;
    uint32_t lhVar;
    hr = bCap_RobotGetVariable(iSockFD, lhRobot, "@CURRENT_TOOL", "", &lhVar);  /* Get var handle */
    if FAILED(hr) {
        SPDLOG_ERROR("Set TCP Load failed to get @CURRENT_TOOL variable");
        return hr;
    }

    hr = bCap_VariablePutValue(iSockFD, lhVar, VT_I4, 1, &lValue);  /* Put Value */
    if FAILED(hr) {
        SPDLOG_ERROR("Set TCP Load failed to put value");
    }

    hr = bCap_VariableRelease(iSockFD, lhVar);	/* Release var handle*/
    if FAILED(hr) {
        SPDLOG_ERROR("Set TCP Load failed to release variable");
    }

    return hr;
}

BCAP_HRESULT DensoReadWriteDriver::SlvChangeMode(const char* mode) {
    long lResult;
    BCAP_HRESULT hr = bCap_RobotExecute(iSockFD, lhRobot, "slvChangeMode", mode, &lResult);
    if FAILED(hr) {
        throw bCapException("\033[1;31mbCap_SlvChangeMode failed.\033[0m");
    }
    return hr;
}


BCAP_HRESULT DensoReadWriteDriver::SlvMove(BCAP_VARIANT* pose, BCAP_VARIANT* result) {
    BCAP_HRESULT hr;
    for (size_t attempt = 0; attempt < 3; ++attempt) {
        hr = bCap_RobotExecute2(iSockFD, lhRobot, "slvMove", pose, result);
        if (SUCCEEDED(hr)) {
            break;
        }
        SPDLOG_WARN("Failed to execute slvMove, attempt ", std::to_string(attempt));
    }
    if (FAILED(hr)) {
        SPDLOG_ERROR("Failed to execute b-CAP slave move.");
    }
    return hr;
}

BCAP_HRESULT DensoReadWriteDriver::ForceSensor(const char* mode) {
    long lResult;
    BCAP_HRESULT hr = bCap_RobotExecute(iSockFD, lhRobot, "ForceSensor", mode, &lResult);
    if FAILED(hr) {
        SPDLOG_ERROR("Failed to execute ForceSensor.");
    }
    return hr;
}


DensoController::DensoController() :
    read_driver(DensoReadDriver()),
    write_driver(DensoReadWriteDriver())
{
    current_waypoint_index = 0;
}

void DensoController::Start() {
    read_driver.bCapOpen();
    SPDLOG_INFO("b-Cap port opened for read driver");
    read_driver.bCapServiceStart();
    SPDLOG_INFO("b-Cap service started for read driver");
    read_driver.bCapControllerConnect();
    SPDLOG_INFO("Connected to controller for read driver");
    read_driver.bCapGetRobot();
    SPDLOG_INFO("Obtained robot handle for read driver");

    write_driver.bCapOpen();
    SPDLOG_INFO("b-Cap port opened for write driver");
    write_driver.bCapServiceStart();
    SPDLOG_INFO("b-Cap service started for write driver");
    write_driver.bCapControllerConnect();
    SPDLOG_INFO("Connected to controller for write driver");
    write_driver.bCapGetRobot();
    SPDLOG_INFO("Obtained robot handle for write driver");

    BCAP_HRESULT hr;

    hr = ClearError();
    if FAILED(hr) {
        std::string err_description = read_driver.GetErrorDescription(hr);
        SPDLOG_ERROR("ClearError failed: " + err_description);
        Stop();
        throw bCapException("Fail to clear error.", err_description);
    }

    hr = write_driver.ManualReset();
    if FAILED(hr) {
        std::string err_description = GetErrorDescription(hr);
        SPDLOG_ERROR("ManualReset failed: " + err_description);
        Stop();
        throw bCapException("Fail to execute manual reset.", err_description);
    }

    // Set the external speed to 100%
    hr = write_driver.SetExtSpeed("100");
    if FAILED(hr) {
        std::string err_description = GetErrorDescription(hr);
        SPDLOG_ERROR("SetExtSpeed failed: " + err_description);
        Stop();
        throw bCapException("Fail to set external speed.", err_description);
    }

    auto arm_mutex = DensoArmMutex(write_driver);
    hr = write_driver.Motor(true);
    if FAILED(hr) {
        std::string err_description = GetErrorDescription(hr);
        SPDLOG_ERROR("MotorOn failed: " + err_description);
        Stop();
        throw bCapException("Fail to turn motor on.", err_description);
    }
    current_waypoint_index = 0;
}

void DensoController::Stop() {
    force_limit_exceeded = true;

    read_driver.bCapReleaseRobot();
    read_driver.bCapControllerDisconnect();
    read_driver.bCapServiceStop();
    read_driver.bCapClose();

    write_driver.bCapReleaseRobot();
    write_driver.bCapControllerDisconnect();
    write_driver.bCapServiceStop();
    write_driver.bCapClose();
}

BCAP_HRESULT DensoController::ClearError() {
    return write_driver.ClearError();
}

std::string DensoController::GetErrorDescription(BCAP_HRESULT error_code) {
    return read_driver.GetErrorDescription(error_code);
}

BCAP_HRESULT DensoController::SetTcpLoad(const int32_t tool_value) {
    return write_driver.SetTcpLoad(tool_value);
}

std::tuple<BCAP_HRESULT, std::vector<double>>
DensoController::GetMountingCalib(const char* work_coordinate) {
    return read_driver.GetMountingCalib(work_coordinate);
}

std::tuple<BCAP_HRESULT, std::vector<double>>
DensoController::GetJointPositions() {
    std::vector<double> joint_positions;
    BCAP_HRESULT hr = read_driver.GetCurJnt(joint_positions);

    // Convert joint positions to radians
    for (int i = 0; i < joint_positions.size(); i++) {
        joint_positions[i] = Deg2Rad(joint_positions[i]);
    }
    return {hr, joint_positions};
}


bool DensoController::ExecuteServoTrajectory(
    RobotTrajectory& traj
    // TODO: add force threshold parameters
)
{
    bool trajectory_execution_finished = true;
    BCAP_HRESULT hr;
    long lResult;

    auto arm_mutex = DensoArmMutex(write_driver);

    // Reset the force sensor to prevent drift in the force readings.
    // Do it here, instead of in the force sensing thread, because this call
    // requires the arm mutex.
    hr = write_driver.ForceSensor("0");

    // Start a thread to stream force sensor data, setting the
    force_limit_exceeded = false;
    std::thread force_sensing_thread(
        &DensoController::RunForceSensingLoop,
        this,
        // TODO: Parameterize these values
        1000.0,  // total force limit
        1000.0,  // total torque limit
        std::vector<double>{1000.0, 1000.0, 20.0, 1000.0, 1000.0, 1000.0}  // tcp force/torque limit
    );

    // Enter slave mode
    hr = write_driver.SlvChangeMode(SERVO_MODE_ON);
    if (FAILED(hr)) {
        std::string err_description = GetErrorDescription(hr);
        SPDLOG_ERROR("Failed to enter b-CAP slave mode." + err_description);
        throw EnterSlaveModeException(err_description);
    }
    SPDLOG_INFO("Slave mode ON");

    // Execute the trajectory
    BCAP_VARIANT vntPose, vntReturn;
    for (size_t i = 0; i < traj.size(); i++) {
        // Stop if the force exceedance condition is true
        if (force_limit_exceeded) {
            SPDLOG_INFO("Force limit exceeded. Stopping trajectory execution.");
            trajectory_execution_finished = false;
            break;
        }

        current_waypoint_index = i;
        const auto& joint_position = traj.trajectory[i];        
        vntPose = VNTFromRadVector(joint_position);
        hr = write_driver.SlvMove(&vntPose, &vntReturn);

        if (FAILED(hr)) {
            std::string err_description = GetErrorDescription(hr);
            SPDLOG_ERROR("Failed to execute b-CAP slave move. " + err_description);
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

    // Stop the force sensing thread.
    force_limit_exceeded = true;
    force_sensing_thread.join();

    // Close loop servo commands on last waypoint
    ClosedLoopCommandServoJoint(traj.trajectory.back());

    // SPDLOG_INFO("Exec traj done");

    // Exit slave mode
    hr = write_driver.SlvChangeMode(SERVO_MODE_OFF);
    if (FAILED(hr)) {
        std::string err_description = GetErrorDescription(hr);
        SPDLOG_ERROR("Failed to exit b-CAP slave mode." + err_description);
        throw ExitSlaveModeException(err_description);
    }
    SPDLOG_INFO("Slave mode OFF");

    return trajectory_execution_finished;
}

void DensoController::RunForceSensingLoop(
    double totalForceLimit,
    double totalTorqueLimit,
    std::vector<double> tcpForceTorqueLimit
) {
    while (!force_limit_exceeded) {
        // Check the force threshold
        BCAP_HRESULT hr;
        std::vector<double> force_values;
        hr = read_driver.GetForceValue(force_values);
        if (FAILED(hr)) {
            SPDLOG_ERROR("Failed to get force values.");
            throw bCapException("Force sensing failed.");
        }

        // Check the total force does not exceed the limit
        double total_force = std::sqrt(
            std::pow(force_values[0], 2) + std::pow(force_values[1], 2) + std::pow(force_values[2], 2)
        );
        if (total_force > totalForceLimit) {
            force_limit_exceeded = true;
            SPDLOG_INFO("Force limit exceeded. Total force: " + std::to_string(total_force));
            break;
        }

        // Check the total torque does not exceed the limit
        double total_torque = std::sqrt(
            std::pow(force_values[3], 2) + std::pow(force_values[4], 2) + std::pow(force_values[5], 2)
        );
        if (total_torque > totalTorqueLimit) {
            force_limit_exceeded = true;
            SPDLOG_INFO("Torque limit exceeded. Total torque: " + std::to_string(total_torque));
            break;
        }

        // Check the TCP force/torque does not exceed the limit
        for (int i = 0; i < 6; i++) {
            if (std::abs(force_values[i]) > tcpForceTorqueLimit[i]) {
                force_limit_exceeded = true;
                SPDLOG_INFO("TCP force/torque limit exceeded. Force/Torque: " + std::to_string(force_values[i]));
                break;
            }
        }

        sleep(0.001);
    }
}

void DensoController::ClosedLoopCommandServoJoint(std::vector<double> last_waypoint) {
    /* Repeat the last servo j command until the robot reaches tolerance or timeout */
    double CLOSE_LOOP_JOINT_ANGLE_TOLERANCE = 0.0001;  // rad
    double CLOSE_LOOP_TIMEOUT = 0.1;  // 100ms
    int CLOSE_LOOP_MAX_ITERATION = 10;

    std::vector<double> current_jnt_deg;
    BCAP_HRESULT hr = read_driver.GetCurJnt(current_jnt_deg);
    std::vector<double> current_jnt_rad = VDeg2Rad(current_jnt_deg);
    if (FAILED(hr)) {
        SPDLOG_ERROR("Closed loop servo j commands failed to get initial joint position");
    }
    
    // Print the initial joint error
    std::vector<double> joint_error;
    for (int i = 0; i < current_jnt_rad.size(); ++i) {
        joint_error.push_back(current_jnt_rad[i] - last_waypoint[i]);
    }
    char buffer[256] = {0};
    std::sprintf(buffer, "Before closed-loop servo commands, joint error: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                joint_error[0], joint_error[1], joint_error[2], joint_error[3], joint_error[4], joint_error[5]);
    SPDLOG_DEBUG(std::string(buffer));

    int count = 0;
    while (true) {
        // Check iteration count
        if (count >= CLOSE_LOOP_MAX_ITERATION)
        {
            break;
        }

        // Check Joint Tolerance
        hr = read_driver.GetCurJnt(current_jnt_deg);
        if (FAILED(hr)) {
            SPDLOG_ERROR("Closed loop servo j commands failed to get current joint position");
            break;
        }
        current_jnt_rad = VDeg2Rad(current_jnt_deg);

        bool all_within_tolerance = true;
        for (int i = 0; i < current_jnt_rad.size(); ++i) {
            if (std::abs(current_jnt_rad[i] - last_waypoint[i]) > CLOSE_LOOP_JOINT_ANGLE_TOLERANCE) {
                all_within_tolerance = false;
                break;
            }
        }
        if (all_within_tolerance) {
            break;
        }

        // Command the last waypoint again
        BCAP_VARIANT vntPose = VNTFromRadVector(last_waypoint);
        BCAP_VARIANT vntReturn;
        write_driver.SlvMove(&vntPose, &vntReturn);

        count++;
    }

    // Print the joint remaining joint error
    joint_error.clear();
    for (int i = 0; i < current_jnt_rad.size(); ++i) {
        joint_error.push_back(current_jnt_rad[i] - last_waypoint[i]);
    }
    std::memset(buffer, 0, sizeof(buffer));
    std::sprintf(buffer, "After %d closed-loop servo commands, joint error: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                count, joint_error[0], joint_error[1], joint_error[2], joint_error[3], joint_error[4], joint_error[5]);
    SPDLOG_DEBUG(std::string(buffer));
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

std::vector<double> VDeg2Rad(std::vector<double> vect0) {
    std::vector<double> resvect;
    resvect.resize(0);
    for (int i = 0; i < int(vect0.size()); i++) {
        resvect.push_back(Deg2Rad(vect0[i]));
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
