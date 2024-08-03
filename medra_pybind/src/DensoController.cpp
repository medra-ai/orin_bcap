#include "b-Cap.h"
#include "DensoController.hpp"
#include <iostream>
#include <math.h>
#include <time.h>

namespace denso_controller {

DensoController::DensoController() {
    server_ip_address = DEFAULT_SERVER_IP_ADDRESS;
    server_port_num = DEFAULT_SERVER_PORT_NUM;
}

////////////////////////////// Low Level Commands //////////////////////////////

void DensoController::bCapOpen() {
    std::cout << "\033[1;32mInitialize and start b-CAP.\033[0m\n";
    BCAP_HRESULT hr = bCap_Open(server_ip_address, server_port_num, &iSockFD);
    if FAILED(hr) {
        throw bCapException("\033[1;31bCap_Open failed.\033[0m\n");
    }
}

void DensoController::bCapClose() {
    std::cout << "\033[1;32mStop b-CAP.\033[0m\n";
    BCAP_HRESULT hr = bCap_Close(iSockFD);
    if FAILED(hr) {
        throw bCapException("\033[1;31bCap_Close failed.\033[0m\n");
    }
}

void DensoController::bCapServiceStart() {
    std::cout << "\033[1;32mStart b-CAP service.\033[0m\n";
    BCAP_HRESULT hr = bCap_ServiceStart(iSockFD);
    if FAILED(hr) {
        throw bCapException("\033[1;31bCap_ServiceStart failed.\033[0m\n");
    }
}

void DensoController::bCapServiceStop() {
    std::cout << "\033[1;32mStop b-CAP service.\033[0m\n";
    BCAP_HRESULT hr = bCap_ServiceStop(iSockFD);
    if FAILED(hr) {
        throw bCapException("\033[1;31mbCap_ServiceStop failed.\033[0m\n");
    }
}

void DensoController::bCapControllerConnect() {
    std::cout << "Get controller handle.\n";
    std::cout << "server ip address: " << server_ip_address << "..." << std::endl;
    BCAP_HRESULT hr = bCap_ControllerConnect(iSockFD, "b-CAP", "caoProv.DENSO.VRC9", server_ip_address, "", &lhController);
    if FAILED(hr) {
        throw bCapException("bCap_ConrtollerConnect failed.\n");
    }
}

void DensoController::bCapControllerDisconnect() {
    std::cout << "Release controller handle.\n";
    BCAP_HRESULT hr = bCap_ControllerDisconnect(iSockFD, lhController);
    if FAILED(hr) {
        throw bCapException("bCap_ConrtollerDisconnect failed.\n");
    }
}

void DensoController::bCapGetRobot() {
    std::cout << "Get robot handle.\n";
    BCAP_HRESULT hr = bCap_ControllerGetRobot(iSockFD, lhController, "Arm", "", &lhRobot);
    if FAILED(hr) {
        throw bCapException("bCap_ConrtollerDisconnect failed.\n");
    }
}

void DensoController::bCapReleaseRobot() {
    std::cout << "Release robot handle.\n";
    BCAP_HRESULT hr = bCap_RobotRelease(iSockFD, lhRobot);
    if FAILED(hr) {
        throw bCapException("bCap_RobotRelease failed.\n");
    }
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
        long lResult;
        BCAP_HRESULT hr1 = bCap_RobotExecute(iSockFD, lhRobot, "slvGetMode", "", &lResult);
        if (lResult > 512) {
            std::cout << "Changed to mode 2 ";
            if (lResult == 513) std::cout << "P-type.\n";
            if (lResult == 514) std::cout << "J-type.\n";
            if (lResult == 515) std::cout << "T-type.\n";
        }
        else if (lResult > 256) {
            std::cout << "Changed to mode 1 ";
            if (lResult == 257) std::cout << "P-type.\n";
            if (lResult == 258) std::cout << "J-type.\n";
            if (lResult == 259) std::cout << "T-type.\n";
        }
        else if (lResult > 0) {
            std::cout << "Changed to mode 0 ";
            if (lResult == 1) std::cout << "P-type.\n";
            if (lResult == 2) std::cout << "J-type.\n";
            if (lResult == 3) std::cout << "T-type.\n";
        }
        else {
            std::cout << "Released slave mode.\n";
        }
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
        std::cout << "External speed is set to " << speed << " %\n";
    }
    return hr;
}

BCAP_HRESULT DensoController::ManualReset() {
    BCAP_HRESULT hr = BCAP_S_OK;
    long lResult;
    hr = bCap_ControllerExecute(iSockFD, lhController, "ManualReset", "", &lResult);
    if SUCCEEDED(hr) {
        std::cout << "Executed Manual Reset %\n";
    }
    return hr;
}

BCAP_HRESULT DensoController::SetTcpLoad(const int32_t tool_value) {
    // Our Robotiq grippers
    // tool_value = 1; // ROBOTIQ_2F85_GRIPPER_PAYLOAD
    // tool_value = 2; // ROBOTIQ_2F140_GRIPPER_PAYLOAD
    long lValue = tool_value;
    
    BCAP_HRESULT hr = BCAP_S_OK;
    hr = bCapRobotExecute("TakeArm", "");
    if (FAILED(hr)) {
        std::cerr << "Set TcpLoad failed to take arm control authority." << std::endl;
        return hr;
    }

    uint32_t lhVar;
    hr = bCap_RobotGetVariable(iSockFD, lhRobot, "CURRENT_TOOL", "", &lhVar);   /* Get var handle */
    if FAILED(hr) {
        std::cerr << "Set TCP Load failed to get CURRENT_TOOL variable %\n";
        return hr;
    }

    hr = bCap_VariablePutValue(iSockFD, lhVar, VT_I4, 1, &lValue);      /* Put Value */
    if SUCCEEDED(hr){
        std::cout << "Set TCP Load successful %\n";
    }
    else {
        std::cerr << "Set TCP Load failed to put value %\n";
    }

    hr = bCap_VariableRelease(iSockFD, lhVar);	
    if FAILED(hr) {
        std::cerr << "Set TCP Load failed to release variable %\n";
    }

    hr = bCapRobotExecute("Givearm", "");
    if (FAILED(hr)) {
        std::cerr << "Set TcpLoad failed to give arm control authority." << std::endl;
    }
    return hr;
}

BCAP_HRESULT DensoController::ChangeTool(char* tool_name) {
    // tool_name = "Tool1";
    // tool_name = "Tool2";

    long lResult;
    BCAP_HRESULT hr = bCap_RobotExecute(iSockFD, lhRobot, "TakeArm", "", &lResult);
    if SUCCEEDED(hr) 
    {			
        hr = bCap_RobotChange(iSockFD, lhRobot, tool_name); /* Change Tool */
        if SUCCEEDED(hr) {
            std::cout << "Tool changed to " << tool_name << " %\n";
        }
        else {
            std::cerr << "Failed to change tool %\n";
        }
    }
    hr = bCap_RobotExecute(iSockFD, lhRobot, "GiveArm", "", &lResult);

    return hr;
}

std::vector<double> DensoController::GetMountingCalib(const char* work_coordinate) {
    double work_def[8]; // Should this be 6?

    BCAP_HRESULT hr = BCAP_S_OK;
    hr = bCap_RobotExecute(iSockFD, lhRobot, "getWorkDef", work_coordinate, &work_def);
    if FAILED(hr) {
        std::cerr << "Failed to get mounting calibration %\n";
    }

    std::vector<double> mounting_calib;
    mounting_calib.resize(0);
    for (int i = 0; i < 6; i++) {
        mounting_calib.push_back(work_def[i]);
    }
    std::cout << "Got Mounting Calibration: (" << mounting_calib[0] << ", " 
                                               << mounting_calib[1] << ", " 
                                               << mounting_calib[2] << ", " 
                                               << mounting_calib[3] << ", " 
                                               << mounting_calib[4] << ", " 
                                               << mounting_calib[5] << ")" << std::endl;
    return mounting_calib;
}

////////////////////////////// High Level Commands //////////////////////////////

int DensoController::executeServoTrajectory(RobotTrajectory& traj)
{
    BCAP_HRESULT hr;
    long lResult;

    hr = bCapRobotExecute("TakeArm", "");
    if (FAILED(hr)) {
        std::cerr << "Failed to get arm control authority." << std::endl;
        return 1;
    }
    std::cout << "Take arm done" << std::endl;

    // enter slave mode: mode 2 J-Type
    hr = bCapSlvChangeMode("514");
    if (FAILED(hr)) {
        std::cerr << "Failed to enter b-CAP slave mode." << std::endl;
        return 1;
    }
    std::cout << "Slave mode ON" << std::endl;

    BCAP_VARIANT vntPose, vntReturn;
    for (size_t i = 0; i < traj.size(); i++) {
        const auto& joint_position = traj.trajectory[i];
        vntPose = VNTFromRadVector(joint_position);
        hr = bCapSlvMove(&vntPose, &vntReturn);
        std::cout << "slvmove " <<  joint_position.at(5) << std::endl;
        if (FAILED(hr)) {
            std::cerr << "Failed to execute b-CAP slave move, index "
                << i << " of " << traj.size() << std::endl;
            return 1;
        }
    }
    std::cout << "Exec traj done" << std::endl;
    // exit slave mode
    hr = bCapSlvChangeMode("0");
    if (FAILED(hr)) {
        std::cerr << "Failed to exit b-CAP slave mode." << std::endl;
        return 1;
    }
    
    std::cout << "Slave mode OFF" << std::endl;

    hr = bCap_RobotExecute(iSockFD, lhRobot, "GiveArm", "", &lResult);
    if (FAILED(hr)) {
        std::cerr << "Failed to give arm control authority." << std::endl;
        return 1;
    }
    std::cout << "Give arm done" << std::endl;

    return 0;
}

void DensoController::bCapEnterProcess(){
    BCAP_HRESULT hr;

    bCapOpen();
    bCapServiceStart();
    bCapControllerConnect();
    std::cout << "Connected to controller" << std::endl;
    bCapGetRobot();
    std::cout << "Obtained robot handle" << std::endl;

    long lResult;
    hr = bCap_ControllerExecute(iSockFD, lhController, "ClearError", "", &lResult);

    hr = bCapRobotExecute("TakeArm", "");
    if FAILED(hr) {
        throw bCapException("\033[1;31mFail to get arm control authority.\033[0m\n");
    }

    hr = bCapMotor(true);
    if FAILED(hr) {
        bCapExitProcess();
        throw bCapException("\033[1;31mFail to turn motor on.\033[0m\n");
    }

}

void DensoController::bCapExitProcess() {
    BCAP_HRESULT hr;
    hr = bCapMotor(false);
    if FAILED(hr) {
        std::cout << "\033[1;31mFail to turn off motor.\033[0m\n";
    }

    hr = bCapRobotExecute("Givearm", "");
    if FAILED(hr) {
        std::cout << "\033[1;31mFail to release arm control authority.\033[0m\n";
    }

    bCapReleaseRobot();
    bCapControllerDisconnect();
    bCapServiceStop();
    bCapClose();
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

std::vector<double> DensoController::GetCurJnt() {
    BCAP_HRESULT hr;
    double dJnt[8];
    std::vector<double> jointvalues;
    jointvalues.resize(0);

    hr = bCap_RobotExecute(iSockFD, lhRobot, "CurJnt", "", &dJnt);
    if FAILED(hr) {
        std::cout << "\033[1;31mFail to get current joint values.\033[0m\n";
        return jointvalues;
    }
    for (int i = 0; i < 8; i++) {
        jointvalues.push_back(dJnt[i]);
    }
    return jointvalues;
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
