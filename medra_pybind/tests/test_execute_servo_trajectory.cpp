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
    BCAP_HRESULT hr = BCAP_S_OK;

    denso_controller::DensoController controller;
    controller.Start();

    // TODO: Add manual reset and clear errors

    // Generate a trajectory
    denso_controller::JointPosition currentPose;
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

    denso_controller::RobotTrajectory forward_trajectory = {currentPose};
    denso_controller::RobotTrajectory reverse_trajectory = {};
    for (size_t i = 0; i < 100; i++) {
        denso_controller::JointPosition newPose = {
            currentPose[0],
            currentPose[1],
            currentPose[2],
            currentPose[3],
            currentPose[4],
            currentPose[5] + 0.001,
        };
        forward_trajectory.push_back(newPose);
        reverse_trajectory.insert(reverse_trajectory.begin(), currentPose);
        currentPose = newPose;
    }

    // Execute the trajectory
    for (size_t iters = 0; iters < 100000; ++iters) {
        // std::optional<std::vector<double>> force_vector = std::vector<double>{10000.0, 10000.0, 10.0, 10000.0, 10000.0, 10000.0};
        auto total_force_limit = 10.0;
        auto result = controller.ExecuteServoTrajectory(
            forward_trajectory,
            total_force_limit,
            std::nullopt,
            std::nullopt
        );
        if (result.error_code != denso_controller::ExecuteServoTrajectoryError::SUCCESS) {
            std::cout << "Stopped early" << std::endl;
            break;
        }

        result = controller.ExecuteServoTrajectory(
            reverse_trajectory,
            total_force_limit,
            std::nullopt,
            std::nullopt
        );
        if (result.error_code != denso_controller::ExecuteServoTrajectoryError::SUCCESS) {
            std::cout << "Stopped early" << std::endl;
            break;
        }
    }

    controller.Stop();
}