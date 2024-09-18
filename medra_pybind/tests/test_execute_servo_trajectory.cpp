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
    controller.Start();

    // TODO: Add manual reset and clear errors

    // Generate a trajectory
    std::vector<double> currentPose;
    auto jt_tuple = controller.GetJointPositions();
    hr = std::get<0>(jt_tuple);

    // Test GetErrorDescription
    std::string err_description = controller.GetErrorDescription(hr);
    std::cout << "CurJnt err description: " << err_description << std::endl;

    currentPose = std::get<1>(jt_tuple);
    if (FAILED(hr)) {
        std::cerr << "Failed to get current joint position" << std::endl;
        return 1;
    }
    std::cout << currentPose[0]
              << " " << currentPose[1]
              << " " << currentPose[2]
              << " " << currentPose[3]
              << " " << currentPose[4]
              << " " << currentPose[5] << std::endl;

    std::vector<std::vector<double>> forward_trajectory_poses = {currentPose};
    std::vector<std::vector<double>> reverse_trajectory_poses = {};
    for (size_t i = 0; i < 1000; i++) {
        std::vector<double> newPose = {
            currentPose[0],
            currentPose[1],
            currentPose[2],
            currentPose[3],
            currentPose[4],
            currentPose[5],
        };
        forward_trajectory_poses.push_back(newPose);
        reverse_trajectory_poses.insert(reverse_trajectory_poses.begin(), currentPose);
        currentPose = newPose;
    }

    const size_t dimension = 6;
    RobotTrajectory forward_trajectory;
    forward_trajectory.trajectory = forward_trajectory_poses;
    RobotTrajectory reverse_trajectory;
    reverse_trajectory.trajectory = reverse_trajectory_poses;


    // Execute the trajectory
    for (size_t iters = 0; iters < 100000; ++iters) {
        // std::optional<std::vector<double>> force_vector = std::vector<double>{10000.0, 10000.0, 10.0, 10000.0, 10000.0, 10000.0};
        auto total_force_limit = 10.0;
        if (!controller.ExecuteServoTrajectory(
            forward_trajectory,
            total_force_limit,
            std::nullopt,
            std::nullopt
        )) {
            std::cout << "Stopped early" << std::endl;
            break;
        }
        if (!controller.ExecuteServoTrajectory(
            reverse_trajectory,
            total_force_limit,
            std::nullopt,
            std::nullopt
        )) {
            std::cout << "Stopped early" << std::endl;
            break;
        }
    }

    controller.Stop();
}