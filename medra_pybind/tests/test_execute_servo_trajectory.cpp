#define _USE_MATH_DEFINES
#include <math.h>

#include "DensoController.hpp"
#include <iostream>
#include <unistd.h>
#include <vector>

#define SERVER_IP_ADDRESS    "192.168.0.1"
#define SERVER_PORT_NUM      5007

#define E_BUF_FULL           0x83201483
#define S_BUF_FULL           0x0F200501

#define PERIOD     100
#define AMPLITUDE  1

int main(){

    int iSockFD;
    uint32_t lResult;
    uint32_t lhController, lhRobot;
    BCAP_HRESULT hr = BCAP_S_OK;

    double dJnt[8];
    BCAP_VARIANT vntPose, vntReturn;

    denso_controller::DensoController controller;
    controller.bCapEnterProcess();
    BCAP_HRESULT speed_hr = controller.SetExtSpeed("100");

    // Test GetErrorDescription
    std::string err_description = controller.GetErrorDescription(speed_hr);
    std::cout << "SetExtSpeed err description: " << err_description << std::endl;

    // TODO: Add manual reset and clear errors

    // Generate a trajectory
    std::vector<double> currentPose;
    auto jt_tuple = controller.GetCurJnt();
    hr = std::get<0>(jt_tuple);
    currentPose = std::get<1>(jt_tuple);
    if (FAILED(hr)) {
        std::cerr << "Failed to get current joint position" << std::endl;
        return 1;
    }

    currentPose = {
        denso_controller::Deg2Rad(currentPose[0]),
        denso_controller::Deg2Rad(currentPose[1]),
        denso_controller::Deg2Rad(currentPose[2]),
        denso_controller::Deg2Rad(currentPose[3]),
        denso_controller::Deg2Rad(currentPose[4]),
        denso_controller::Deg2Rad(currentPose[5]),
    };
    std::cout << currentPose[0]
              << " " << currentPose[1]
              << " " << currentPose[2]
              << " " << currentPose[3]
              << " " << currentPose[4]
              << " " << currentPose[5] << std::endl;

    std::vector<std::vector<double>> trajectory_poses = {currentPose};
    for (size_t i = 0; i < 1000; i++) {
        std::vector<double> newPose = {
            currentPose[0],
            currentPose[1],
            currentPose[2],
            currentPose[3],
            currentPose[4],
            currentPose[5] + 0.0001,
        };
        trajectory_poses.push_back(newPose);
        currentPose = newPose;
    }

    const size_t dimension = 6;
    RobotTrajectory trajectory;
    trajectory.trajectory = trajectory_poses;

    // Execute the trajectory
    controller.ExecuteServoTrajectory(trajectory);

    controller.bCapMotor(false);
    controller.bCapReleaseRobot();
    controller.bCapClose();
}