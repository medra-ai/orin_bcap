/**
 * C++ Wrapper around Denso b-CAP library
 * Based on https://github.com/quangounet/denso/blob/master/cpp/src/DensoController.cpp
 */

#include "b-Cap.h"
#include "DensoController.hpp"
#include "filter.hpp"

#include <iostream>
#include <math.h>
#include <time.h>
#include <thread>

#include <fstream>

#include <spdlog/spdlog.h>

// start realtime headers
#include <iostream>
#include <sched.h>
#include <sys/resource.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <cstdio>

namespace denso_controller
{

    DensoArmMutex::DensoArmMutex(DensoReadWriteDriver &driver) : driver(driver)
    {
        driver.TakeArm();
        driver.Motor(true);
    }

    DensoArmMutex::~DensoArmMutex()
    {
        driver.GiveArm();
    }

    DensoReadDriver::DensoReadDriver()
    {
        server_ip_address = DEFAULT_SERVER_IP_ADDRESS;
        server_port_num = DEFAULT_SERVER_PORT_NUM;
        iSockFD = 0;
        lhController = 0;
        lhRobot = 0;

        session_name = "read-only";
    }

    void DensoReadDriver::Start()
    {
        bCapOpen();
        SPDLOG_INFO("b-Cap port opened");
        bCapServiceStart();
        SPDLOG_INFO("b-Cap service started");
        bCapControllerConnect();
        SPDLOG_INFO("Connected to controller");
        bCapGetRobot();
        SPDLOG_INFO("Obtained robot handle");
    }

    void DensoReadDriver::Stop()
    {
        bCapReleaseRobot();
        SPDLOG_INFO("Released robot handle");
        bCapControllerDisconnect();
        SPDLOG_INFO("Disconnected from controller");
        bCapServiceStop();
        SPDLOG_INFO("b-Cap service stopped");
        bCapClose();
        SPDLOG_INFO("b-Cap port closed");
    }

    void DensoReadDriver::bCapOpen()
    {
        SPDLOG_INFO("Initialize and start b-CAP.");
        BCAP_HRESULT hr = bCap_Open(server_ip_address, server_port_num, &iSockFD);
        if FAILED (hr)
        {
            throw bCapException("\033[1;31mbCap_Open failed.\033[0m");
        }
    }

    void DensoReadDriver::bCapClose()
    {
        SPDLOG_INFO("Stop b-CAP.");
        BCAP_HRESULT hr = bCap_Close(iSockFD);
        if FAILED (hr)
        {
            throw bCapException("\033[1;31mbCap_Close failed.\033[0m");
        }
    }

    void DensoReadDriver::bCapServiceStart()
    {
        SPDLOG_INFO("Start b-CAP service.");
        BCAP_HRESULT hr = bCap_ServiceStart(iSockFD);
        if FAILED (hr)
        {
            throw bCapException("\033[1;31mbCap_ServiceStart failed.\033[0m");
        }
    }

    void DensoReadDriver::bCapServiceStop()
    {
        SPDLOG_INFO("Stop b-CAP service.");
        BCAP_HRESULT hr = bCap_ServiceStop(iSockFD);
        if FAILED (hr)
        {
            throw bCapException("\033[1;31mbCap_ServiceStop failed.\033[0m");
        }
    }

    void DensoReadDriver::bCapControllerConnect()
    {
        SPDLOG_INFO("Getting controller handle. Server ip address: " + std::string(server_ip_address));
        BCAP_HRESULT hr = bCap_ControllerConnect(iSockFD, session_name, "caoProv.DENSO.VRC9", server_ip_address, "", &lhController);
        if FAILED (hr)
        {
            throw bCapException("\033[1;31mbCap_ControllerConnect failed.\033[0m");
        }
    }

    void DensoReadDriver::bCapControllerDisconnect()
    {
        SPDLOG_INFO("Release controller handle.");
        BCAP_HRESULT hr = bCap_ControllerDisconnect(iSockFD, lhController);
        if FAILED (hr)
        {
            throw bCapException("\033[1;31mbCap_ControllerDisconnect failed.\033[0m");
        }
    }

    void DensoReadDriver::bCapGetRobot()
    {
        SPDLOG_INFO("Get robot handle.");
        BCAP_HRESULT hr = bCap_ControllerGetRobot(iSockFD, lhController, "Arm", "", &lhRobot);
        if FAILED (hr)
        {
            throw bCapException("\033[1;31mbCap_ControllerGetRobot failed.\033[0m");
        }
    }

    void DensoReadDriver::bCapReleaseRobot()
    {
        SPDLOG_INFO("Release robot handle.");
        BCAP_HRESULT hr = bCap_RobotRelease(iSockFD, lhRobot);
        if FAILED (hr)
        {
            throw bCapException("\033[1;31mbCap_RobotRelease failed.\033[0m");
        }
    }

    BCAP_HRESULT DensoReadDriver::ClearError()
    {
        long lResult;
        BCAP_HRESULT hr = bCap_ControllerExecute(
            iSockFD, lhController, "ClearError", "", &lResult);
        return hr;
    }

    BCAP_HRESULT DensoReadDriver::GetCurJnt(std::vector<double> &joint_positions)
    {
        double dJnt[8];
        BCAP_HRESULT hr;
        for (size_t attempt = 0; attempt < 3; ++attempt)
        {
            hr = bCap_RobotExecute(iSockFD, lhRobot, "CurJnt", "", &dJnt);
            if (SUCCEEDED(hr))
            {
                break;
            }
            SPDLOG_WARN("Failed to get joint pos, attempt " + std::to_string(attempt));
            ClearError();
        }
        if (FAILED(hr))
        {
            SPDLOG_ERROR("Failed to get current joint values.");
            return hr;
        }

        joint_positions.resize(0);
        for (int i = 0; i < 8; i++)
        {
            joint_positions.push_back(dJnt[i]);
        }
        return hr;
    }

    BCAP_HRESULT DensoReadDriver::GetForceValue(std::vector<double> &force_values)
    {
        double dForce[8];
        BCAP_HRESULT hr;
        for (size_t attempt = 0; attempt < 3; ++attempt)
        {
            hr = bCap_RobotExecute(iSockFD, lhRobot, "forceValue", "13", &dForce);
            if SUCCEEDED (hr)
            {
                break;
            }
            SPDLOG_WARN("Failed to get force value, attempt " + std::to_string(attempt));
            ClearError();
        }
        if (FAILED(hr))
        {
            SPDLOG_ERROR("Failed to get current force values.");
            return hr;
        }

        force_values.resize(0);
        for (int i = 0; i < 6; i++)
        {
            force_values.push_back(dForce[i]);
        }
        return hr;
    }

    std::tuple<BCAP_HRESULT, std::vector<double>>
    DensoReadDriver::GetMountingCalib(const char *work_coordinate)
    {
        double work_def[8]; // Should this be 6?
        std::vector<double> mounting_calib = std::vector<double>(8);

        BCAP_HRESULT hr = bCap_RobotExecute(iSockFD, lhRobot, "getWorkDef", work_coordinate, &work_def);
        if FAILED (hr)
        {
            SPDLOG_ERROR("Failed to get mounting calibration");
            return {hr, mounting_calib};
        }

        mounting_calib.resize(0);
        for (int i = 0; i < 8; i++)
        {
            mounting_calib.push_back(work_def[i]);
        }
        SPDLOG_INFO("Got Mounting Calibration: (" + std::to_string(mounting_calib[0]) + ", " + std::to_string(mounting_calib[1]) + ", " + std::to_string(mounting_calib[2]) + ", " + std::to_string(mounting_calib[3]) + ", " + std::to_string(mounting_calib[4]) + ", " + std::to_string(mounting_calib[5]) + ")");
        return {hr, mounting_calib};
    }

    std::string DensoReadDriver::GetErrorDescription(BCAP_HRESULT error_code)
    {
        char err_code_str[32];
        snprintf(err_code_str, sizeof(err_code_str), "%d", error_code);
        // Denso defined local receive buffer LOCALRECBUFFER_SZ = 1024
        char error_description[1024] = {0};

        BCAP_HRESULT hr = bCap_ControllerExecute(
            iSockFD,
            lhController,
            "GetErrorDescription",
            err_code_str,
            error_description);
        if FAILED (hr)
        {
            return "Failed to get error description";
        }
        return std::string(error_description);
    }

    BCAP_HRESULT DensoReadDriver::SetupTimer()
    {
        BCAP_HRESULT hr = bCap_ControllerGetVariable(
            iSockFD, lhController, "@TIME", "@ifnotmember", &time_handle
        );
        if FAILED (hr)
        {
            SPDLOG_ERROR("Failed to get @TIME variable");
        }
        return hr;
    }

    uint32_t DensoReadDriver::Timestamp()
    {
        uint32_t timestamp = 0;
        BCAP_VARIANT vntResult;
        BCAP_HRESULT hr = bCap_VariableGetValue(iSockFD, time_handle, &vntResult);
        if FAILED (hr)
        {
            SPDLOG_ERROR("Failed to get timestamp");
        } else {
            timestamp = vntResult.Value.LongValue;
        }
        return timestamp;
    }

    DensoReadWriteDriver::DensoReadWriteDriver()
    {
        server_ip_address = DEFAULT_SERVER_IP_ADDRESS;
        server_port_num = DEFAULT_SERVER_PORT_NUM;
        iSockFD = 0;
        lhController = 0;
        lhRobot = 0;

        session_name = "write";
    }


    BCAP_HRESULT DensoReadWriteDriver::ManualReset()
    {
        long lResult;
        BCAP_HRESULT hr = bCap_ControllerExecute(
            iSockFD, lhController, "ManualReset", "", &lResult);
        if SUCCEEDED (hr)
        {
            SPDLOG_INFO("Executed Manual Reset");
        }
        return hr;
    }

    BCAP_HRESULT DensoReadWriteDriver::Motor(bool command)
    {
        BCAP_HRESULT hr;
        long lResult;
        if (command)
        {
            hr = bCap_RobotExecute(iSockFD, lhRobot, "Motor", "1", &lResult);
        }
        else
        {
            hr = bCap_RobotExecute(iSockFD, lhRobot, "Motor", "0", &lResult);
        }
        return hr;
    }

    BCAP_HRESULT DensoReadWriteDriver::TakeArm()
    {
        long lResult;
        return bCap_RobotExecute(iSockFD, lhRobot, "TakeArm", "", &lResult);
    }

    BCAP_HRESULT DensoReadWriteDriver::GiveArm()
    {
        long lResult;
        return bCap_RobotExecute(iSockFD, lhRobot, "GiveArm", "", &lResult);
    }

    BCAP_HRESULT DensoReadWriteDriver::SetExtSpeed(const char *speed)
    {
        long lResult;
        BCAP_HRESULT hr = bCap_RobotExecute(iSockFD, lhRobot, "ExtSpeed", speed, &lResult);
        if SUCCEEDED (hr)
        {
            SPDLOG_INFO("External speed is set to " + std::string(speed));
        }
        return hr;
    }

    BCAP_HRESULT DensoReadWriteDriver::SetTcpLoad(const int32_t tool_value)
    {
        // Our Robotiq grippers
        // tool_value = 1; // ROBOTIQ_2F85_GRIPPER_PAYLOAD
        // tool_value = 2; // ROBOTIQ_2F140_GRIPPER_PAYLOAD
        long lValue = tool_value;
        auto arm_mutex = DensoArmMutex(*this);

        BCAP_HRESULT hr = BCAP_S_OK;
        uint32_t lhVar;
        hr = bCap_RobotGetVariable(iSockFD, lhRobot, "@CURRENT_TOOL", "", &lhVar); /* Get var handle */
        if FAILED (hr)
        {
            SPDLOG_ERROR("Set TCP Load failed to get @CURRENT_TOOL variable");
            return hr;
        }

        hr = bCap_VariablePutValue(iSockFD, lhVar, VT_I4, 1, &lValue); /* Put Value */
        if FAILED (hr)
        {
            SPDLOG_ERROR("Set TCP Load failed to put value");
        }

        hr = bCap_VariableRelease(iSockFD, lhVar); /* Release var handle*/
        if FAILED (hr)
        {
            SPDLOG_ERROR("Set TCP Load failed to release variable");
        }

        return hr;
    }

    BCAP_HRESULT DensoReadWriteDriver::SlvChangeMode(const char *mode)
    {
        long lResult;
        BCAP_HRESULT hr = bCap_RobotExecute(iSockFD, lhRobot, "slvChangeMode", mode, &lResult);
        if FAILED (hr)
        {
            throw bCapException("\033[1;31mbCap_SlvChangeMode failed.\033[0m");
        }
        return hr;
    }

    BCAP_HRESULT DensoReadWriteDriver::SlvMove(BCAP_VARIANT *pose, BCAP_VARIANT *result)
    {
        return bCap_RobotExecute2(iSockFD, lhRobot, "slvMove", pose, result);
    }

    BCAP_HRESULT DensoReadWriteDriver::ForceSensor(const char *mode)
    {
        long lResult;
        BCAP_HRESULT hr;
        for (size_t attempt = 0; attempt < 3; ++attempt)
        {
            hr = bCap_RobotExecute(iSockFD, lhRobot, "ForceSensor", mode, &lResult);
            if (SUCCEEDED(hr))
            {
                break;
            }
            SPDLOG_WARN("Failed to execute ForceSensor; attempt " + std::to_string(attempt));
            // Give some time to the controller in case it is busy
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        if FAILED (hr)
        {
            SPDLOG_ERROR("Failed to execute ForceSensor.");
        }
        return hr;
    }

    DensoController::DensoController() : read_driver(DensoReadDriver()),
                                         write_driver(DensoReadWriteDriver())
    {
        current_waypoint_index = 0;
    }

    void DensoController::Start()
    {
        read_driver.Start();
        write_driver.Start();

        BCAP_HRESULT hr;
        hr = ClearError();
        if FAILED (hr)
        {
            HandleError(hr, "ClearError failed.");
        }

        hr = write_driver.ManualReset();
        if FAILED (hr)
        {
            HandleError(hr, "ManualReset failed.");
        }

        // Set the external speed to 100%
        hr = write_driver.SetExtSpeed("100");
        if FAILED (hr)
        {
            HandleError(hr, "SetExtSpeed failed.");
        }

        auto arm_mutex = DensoArmMutex(write_driver);
        hr = write_driver.Motor(true);
        if FAILED (hr)
        {
            HandleError(hr, "MotorOn failed.");
        }
        current_waypoint_index = 0;

        hr = read_driver.SetupTimer();
        if FAILED (hr)
        {
            HandleError(hr, "SetupTimer failed.");
        }
    }

    void DensoController::Stop()
    {
        force_limit_exceeded = true;

        read_driver.Stop();
        write_driver.Stop();
    }

    BCAP_HRESULT DensoController::ClearError()
    {
        return write_driver.ClearError();
    }

    std::string DensoController::GetErrorDescription(BCAP_HRESULT error_code)
    {
        return read_driver.GetErrorDescription(error_code);
    }

    BCAP_HRESULT DensoController::SetTcpLoad(const int32_t tool_value)
    {
        return write_driver.SetTcpLoad(tool_value);
    }

    std::tuple<BCAP_HRESULT, std::vector<double>>
    DensoController::GetMountingCalib(const char *work_coordinate)
    {
        return read_driver.GetMountingCalib(work_coordinate);
    }

    std::tuple<BCAP_HRESULT, std::vector<double>>
    DensoController::GetJointPositions()
    {
        std::vector<double> joint_positions;
        BCAP_HRESULT hr = read_driver.GetCurJnt(joint_positions);

        // Convert joint positions to radians
        for (size_t i = 0; i < joint_positions.size(); i++)
        {
            joint_positions[i] = Deg2Rad(joint_positions[i]);
        }
        return {hr, joint_positions};
    }

    std::tuple<DensoController::ExecuteServoTrajectoryError, DensoController::ExecuteServoTrajectoryResult>
    DensoController::ExecuteServoTrajectory(
        const RobotTrajectory &traj,
        const std::optional<double> total_force_limit,
        const std::optional<double> total_torque_limit,
        const std::optional<std::vector<double>> per_axis_force_torque_limits)
    {
        auto arm_mutex = DensoArmMutex(write_driver);

        if (total_force_limit.has_value()
            || total_torque_limit.has_value()
            || per_axis_force_torque_limits.has_value()
        ) {
            // Wait some time for the arm and sensor to settle.
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            // Reset the force sensor to prevent drift in the force readings.
            // Do it here, instead of in the force sensing thread, because this call
            // requires the arm mutex.
            write_driver.ForceSensor("0");
            // TODO: handle error
        }

        // Start a thread to stream force sensor data
        force_limit_exceeded = false;
        std::thread force_sensing_thread(
            &DensoController::RunForceSensingLoop,
            this,
            total_force_limit,
            total_torque_limit,
            per_axis_force_torque_limits);

        // Enter slave mode
        SPDLOG_INFO("Entering slave mode");
        switch (EnterSlaveMode())
        {
            case EnterSlaveModeResult::SUCCESS:
                SPDLOG_INFO("Slave mode ON");
                break;
            case EnterSlaveModeResult::ENTER_SLAVE_MODE_FAILED:
                // TODO: Get the stack of errors from the controller
                // std::string err_description = GetErrorDescription(hr);
                SPDLOG_ERROR("Failed to enter b-CAP slave mode.");  // + err_description);
                return {
                    ExecuteServoTrajectoryError::ENTER_SLAVE_MODE_FAILED,
                    ExecuteServoTrajectoryResult::ERROR
                };
        }

        // Execute the trajectory
        // BCAP_VARIANT vntPose, vntReturn;
        CommandServoJointResult result;
        for (size_t i = 0; i < traj.size(); i++)
        {
            current_waypoint_index = i;
            // Stop if the force exceedance condition is true
            if (current_waypoint_index > 4 && force_limit_exceeded)
            {
                SPDLOG_INFO("Force limit exceeded after "
                            + std::to_string(current_waypoint_index)
                            + " of " + std::to_string(traj.size())
                            + " waypoints. Stopping trajectory execution.");
                break;
            }

            const auto &joint_position = traj.trajectory[i];
            // vntPose = VNTFromRadVector(joint_position);
            // if (i == 0) {
            //     auto time_of_first_move = std::chrono::steady_clock::now();
            //     auto elapsed = time_of_first_move - slave_mode_on;
            //     SPDLOG_INFO("Time to first move: " + std::to_string(elapsed.count()));
            // }
            result = CommandServoJoint(joint_position);
            switch (result)
            {
                case CommandServoJointResult::SUCCESS:
                    break;
                case CommandServoJointResult::SLAVE_MOVE_FAILED:
                    SPDLOG_ERROR(
                        "ExecuteServoTrajectory failed at waypoint "
                        + std::to_string(i)
                        + " of " + std::to_string(traj.size())
                    );

                    // Stop the force sensing thread to prevent it from running indefinitely.
                    force_limit_exceeded = true;
                    force_sensing_thread.join();

                    // std::string err_description = GetErrorDescription(hr);
                    // SPDLOG_ERROR("Error description: " + err_description);

                    // No need to exit slave mode here because the controller
                    // releases slave mode when an error occurs.

                    return {
                        ExecuteServoTrajectoryError::SLAVE_MOVE_FAILED,
                        ExecuteServoTrajectoryResult::ERROR
                    };
            }

            // Print the joint positions
            // std::string msg = "slvmove (";
            // for (int i=0; i<joint_position.size(); ++i)
            //     msg += std::to_string(joint_position[i]) + ' ';
            // msg += ")";
            // SPDLOG_INFO(msg);
        }

        bool exec_complete = !force_limit_exceeded;
        // Stop the force sensing thread by triggering the stop condition.
        // Don't join the force sensing thread here because it may block this
        // thread for too long, causing command position buffer underflow.
        force_limit_exceeded = true;

        // Close loop servo commands on last waypoint.
        if (exec_complete)
        {
            switch (ClosedLoopCommandServoJoint(traj.trajectory.back()))
            {
                case ClosedLoopCommandServoJointResult::SUCCESS:
                    SPDLOG_INFO("Closed loop servo joint commands successful");
                    break;
                case ClosedLoopCommandServoJointResult::GET_CUR_JNT_FAILED:
                case ClosedLoopCommandServoJointResult::SLAVE_MOVE_FAILED:
                    // The trajectory is essentially complete at this point, so we
                    // don't do anything special if the closed loop commands fail.
                    SPDLOG_ERROR("Closed loop servo joint commands failed");
                    ClearError();
                    break;
            }
        }
        force_sensing_thread.join();

        ExecuteServoTrajectoryResult trajectory_result = (
            exec_complete ?
            ExecuteServoTrajectoryResult::COMPLETE :
            ExecuteServoTrajectoryResult::FORCE_LIMIT_EXCEEDED
        );

        SPDLOG_INFO("Turning off slave mode");
        switch (ExitSlaveMode())
        {
            case ExitSlaveModeResult::SUCCESS:
                SPDLOG_INFO("Slave mode OFF");
                break;
            case ExitSlaveModeResult::EXIT_SLAVE_MODE_FAILED:
                // std::string err_description = GetErrorDescription(hr);
                // SPDLOG_ERROR("Failed to exit b-CAP slave mode." + err_description);
                return {
                    ExecuteServoTrajectoryError::EXIT_SLAVE_MODE_FAILED,
                    trajectory_result
                };
        }

        return {
            ExecuteServoTrajectoryError::SUCCESS,
            trajectory_result
        };
    }

    void DensoController::RunForceSensingLoop(
        const std::optional<double> total_force_limit,
        const std::optional<double> total_torque_limit,
        const std::optional<std::vector<double>> per_axis_force_torque_limits)
    {
        // Exit early if no force limits are specified
        if (!total_force_limit.has_value() && !total_torque_limit.has_value() && !per_axis_force_torque_limits.has_value())
        {
            SPDLOG_INFO("Skip force sensing loop");
            return;
        }

        const int frequency = 50;           // Hz
        const int period = 1000 / frequency; // period in milliseconds

        const int filter_size = 3;
        const int force_value_data_size = 6; // x, y, z, rx, ry, rz
        filter::MeanFilter mean_filter(filter_size, force_value_data_size);

        std::ofstream force_sensing_log("force_sensing_log.txt", std::ios::app);
        if (!force_sensing_log.is_open())
        {
            SPDLOG_ERROR("Failed to open force sensing log file.");
            Stop();
            throw bCapException("Force sensing logging failed.");
        }

        BCAP_HRESULT hr;
        std::vector<double> force_values;
        while (!force_limit_exceeded)
        {
            auto start = std::chrono::steady_clock::now();

            // Check the force threshold
            hr = read_driver.GetForceValue(force_values);
            if (FAILED(hr))
            {
                HandleError(hr, "GetForceValue failed.");
            }
            // SPDLOG_INFO("size of force_values 1: ", std::to_string(force_values.size()));
            mean_filter.AddValue(force_values);
            // SPDLOG_INFO("size of force_values 2: ", std::to_string(force_values.size()));

            // Use the mean of the last filter_size force values
            force_values = mean_filter.GetMean();

            // Write the 6-vector data to the file
            for (size_t i = 0; i < force_values.size(); ++i)
            {
                force_sensing_log << force_values[i];
                if (i < force_values.size() - 1)
                {
                    force_sensing_log << " "; // Separate values with a space
                }
            }
            force_sensing_log << "\n"; // Newline after each vector

            // Check the total force does not exceed the limit
            double total_force = std::sqrt(
                std::pow(force_values[0], 2) + std::pow(force_values[1], 2) + std::pow(force_values[2], 2));
            if (total_force_limit.has_value() && total_force > *total_force_limit)
            {
                SPDLOG_INFO("Force limit exceeded. Total force: " + std::to_string(total_force)
                            + ". Force limit: " + std::to_string(*total_force_limit));
                force_limit_exceeded = true;
                break;
            }

            // Check the total torque does not exceed the limit
            double total_torque = std::sqrt(
                std::pow(force_values[3], 2) + std::pow(force_values[4], 2) + std::pow(force_values[5], 2));
            if (total_torque_limit.has_value() && total_torque > *total_torque_limit)
            {
                SPDLOG_INFO("Torque limit exceeded. Total torque: " + std::to_string(total_torque)
                            + ". Torque limit: " + std::to_string(*total_torque_limit));
                force_limit_exceeded = true;
                break;
            }

            // Check the TCP force/torque does not exceed the limit
            if (per_axis_force_torque_limits.has_value())
            {
                for (int i = 0; i < 6; i++)
                {
                    if (std::abs(force_values[i]) > (*per_axis_force_torque_limits)[i])
                    {
                        SPDLOG_INFO("TCP force/torque limit exceeded. Force/Torque: " + std::to_string(force_values[i]));
                        force_limit_exceeded = true;
                        break;
                    }
                }
            }

            // High-precision sleep to maintain the desired frequency
            auto end = std::chrono::steady_clock::now();
            std::chrono::duration<double, std::milli> elapsed = end - start;
            std::this_thread::sleep_for(std::chrono::milliseconds(period) - elapsed);
        }

        force_sensing_log.close();
    }

    DensoController::ClosedLoopCommandServoJointResult
    DensoController::ClosedLoopCommandServoJoint(const std::vector<double>& last_waypoint)
    {
        SPDLOG_INFO("Closed loop servo joint commands");
        /* Repeat the last servo j command until the robot reaches tolerance or timeout */
        const double CLOSE_LOOP_JOINT_ANGLE_TOLERANCE = 0.0001; // rad
        const int CLOSE_LOOP_MAX_ITERATION = 10;
        std::vector<double> current_jnt_deg;
        BCAP_HRESULT hr = write_driver.GetCurJnt(current_jnt_deg);
        std::vector<double> current_jnt_rad = VDeg2Rad(current_jnt_deg);
        if (FAILED(hr))
        {
            SPDLOG_ERROR("Closed loop servo j commands failed to get initial joint position");
        }

        // Print the initial joint error
        std::vector<double> joint_error;
        for (size_t i = 0; i < last_waypoint.size(); ++i)
        {
            joint_error.push_back(current_jnt_rad[i] - last_waypoint[i]);
        }
        char buffer[256] = {0};
        std::sprintf(buffer, "Before closed-loop servo commands, joint error: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                     joint_error[0], joint_error[1], joint_error[2], joint_error[3], joint_error[4], joint_error[5]);
        SPDLOG_INFO(std::string(buffer));

        int count = 0;
        while (true)
        {
            // Check iteration count
            if (count >= CLOSE_LOOP_MAX_ITERATION)
            {
                break;
            }

            // Check Joint Tolerance.
            // Use the write driver here to prevent a race where this thread
            // and the force sensing loop thread concurrently send commands to
            // the controller using the same connection, causing a controller
            // error.
            hr = write_driver.GetCurJnt(current_jnt_deg);
            if (FAILED(hr))
            {
                SPDLOG_ERROR("Closed loop servo j commands failed to get current joint position");
                return ClosedLoopCommandServoJointResult::GET_CUR_JNT_FAILED;
            }
            current_jnt_rad = VDeg2Rad(current_jnt_deg);

            bool all_within_tolerance = true;
            for (size_t i = 0; i < current_jnt_rad.size(); ++i)
            {
                if (std::abs(current_jnt_rad[i] - last_waypoint[i]) > CLOSE_LOOP_JOINT_ANGLE_TOLERANCE)
                {
                    all_within_tolerance = false;
                    break;
                }
            }
            if (all_within_tolerance)
            {
                break;
            }

            // Command the last waypoint again
            auto result = CommandServoJoint(last_waypoint);
            if (result != CommandServoJointResult::SUCCESS)
            {
                SPDLOG_ERROR("Closed loop servo j commands failed to command servo joint");
                return ClosedLoopCommandServoJointResult::SLAVE_MOVE_FAILED;
            }

            count++;
        }

        // Print the joint remaining joint error
        joint_error.clear();
        for (size_t i = 0; i < last_waypoint.size(); ++i)
        {
            joint_error.push_back(current_jnt_rad[i] - last_waypoint[i]);
        }
        std::memset(buffer, 0, sizeof(buffer));
        std::sprintf(buffer, "After %d closed-loop servo commands, joint error: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                     count, joint_error[0], joint_error[1], joint_error[2], joint_error[3], joint_error[4], joint_error[5]);
        SPDLOG_DEBUG(std::string(buffer));

        return ClosedLoopCommandServoJointResult::SUCCESS;
    }

    void DensoController::HandleError(BCAP_HRESULT error_code, const char *error_description)
    {
        SPDLOG_ERROR("Error code: " + std::to_string(error_code));
        SPDLOG_ERROR("Error description: " + std::string(error_description));
        std::string controller_error = read_driver.GetErrorDescription(error_code);
        SPDLOG_ERROR("Controller error message: " + controller_error);

        Stop();
        throw bCapException(
            "Error code: " + std::to_string(error_code) + ". Error description: " + error_description + ". Controller error message: " + controller_error);
    }

    DensoController::EnterSlaveModeResult
    DensoController::EnterSlaveMode() {
        BCAP_HRESULT hr = write_driver.SlvChangeMode(SERVO_MODE_ON);
        if (FAILED(hr))
        {
            SPDLOG_ERROR("Failed to enter b-CAP slave mode.");
            return EnterSlaveModeResult::ENTER_SLAVE_MODE_FAILED;
        }
        return EnterSlaveModeResult::SUCCESS;
    }

    DensoController::ExitSlaveModeResult
    DensoController::ExitSlaveMode() {
        BCAP_HRESULT hr = write_driver.SlvChangeMode(SERVO_MODE_OFF);
        if (FAILED(hr))
        {
            SPDLOG_ERROR("Failed to exit b-CAP slave mode.");
            return ExitSlaveModeResult::EXIT_SLAVE_MODE_FAILED;
        }
        return ExitSlaveModeResult::SUCCESS;
    }

    DensoController::CommandServoJointResult
    DensoController::CommandServoJoint(const std::vector<double> &joint_position)
    {
        BCAP_VARIANT vntPose = VNTFromRadVector(joint_position);
        BCAP_VARIANT vntReturn;
        BCAP_HRESULT hr;
        for (size_t attempt = 0; attempt < 3; ++attempt)
        {
            hr = write_driver.SlvMove(&vntPose, &vntReturn);
            if (SUCCEEDED(hr))
            {
                return CommandServoJointResult::SUCCESS;
            }

            // Retry the command for certain valid errors.
            bool is_valid_error = false;
            for (auto error : VALID_SLVMOVE_ERRORS) {
                if (hr == error) {
                    is_valid_error = true;
                    SPDLOG_WARN("Matched buffer underflow error: " + std::to_string(error));
                    break;
                }
            }
            if (is_valid_error) {
                SPDLOG_WARN("Failed to command servo joint, attempt " + std::to_string(attempt));
                ClearError();
                write_driver.ManualReset();
                write_driver.Motor(true);
                EnterSlaveMode();  // If this call fails, it will be caught in the next iteration
            } else {
                SPDLOG_ERROR("Command servo joint failed with code "
                             + std::to_string(hr)
                             + ". Not retrying.");
                return CommandServoJointResult::SLAVE_MOVE_FAILED;
            }
        }

        if (FAILED(hr))
        {
            SPDLOG_ERROR("Failed to command servo joint.");
            return CommandServoJointResult::SLAVE_MOVE_FAILED;
        }
        return CommandServoJointResult::SUCCESS;
    }

    ////////////////////////////// Utilities //////////////////////////////

    const char *DensoController::CommandFromVector(std::vector<double> q)
    {
        std::vector<double> tmp;
        tmp = VRad2Deg(q);
        std::string commandstring;
        commandstring = "J(" + std::to_string(tmp[0]) + ", " + std::to_string(tmp[1]) + ", " + std::to_string(tmp[2]) + ", " + std::to_string(tmp[3]) + ", " + std::to_string(tmp[4]) + ", " + std::to_string(tmp[5]) + ")";
        return commandstring.c_str(); // convert string -> const shar*
    }

    std::vector<double> DensoController::VectorFromVNT(BCAP_VARIANT vnt0)
    {
        std::vector<double> vect;
        vect.resize(0);
        for (int i = 0; i < 8; i++)
        {
            vect.push_back(vnt0.Value.DoubleArray[i]);
        }
        return vect;
    }

    BCAP_VARIANT DensoController::VNTFromVector(std::vector<double> vect0)
    {
        assert(vect0.size() == 6 || vect0.size() == 8);
        BCAP_VARIANT vnt;
        vnt.Type = VT_R8 | VT_ARRAY;
        vnt.Arrays = 8;

        for (int i = 0; i < int(vect0.size()); i++)
        {
            vnt.Value.DoubleArray[i] = vect0[i];
        }
        return vnt;
    }

    std::vector<double> DensoController::RadVectorFromVNT(BCAP_VARIANT vnt0)
    {
        std::vector<double> vect;
        vect.resize(0);
        for (int i = 0; i < 8; i++)
        {
            vect.push_back(Deg2Rad(vnt0.Value.DoubleArray[i]));
        }
        return vect;
    }

    BCAP_VARIANT DensoController::VNTFromRadVector(std::vector<double> vect0)
    {
        assert(vect0.size() == 6 || vect0.size() == 8);
        BCAP_VARIANT vnt;
        vnt.Type = VT_R8 | VT_ARRAY;
        vnt.Arrays = 8;

        for (int i = 0; i < int(vect0.size()); i++)
        {
            vnt.Value.DoubleArray[i] = Rad2Deg(vect0[i]);
        }
        return vnt;
    }

    uint32_t DensoController::Timestamp()
    {
        return read_driver.Timestamp();
    }

    std::vector<double> VRad2Deg(std::vector<double> vect0)
    {
        std::vector<double> resvect;
        resvect.resize(0);
        for (int i = 0; i < int(vect0.size()); i++)
        {
            resvect.push_back(Rad2Deg(vect0[i]));
        }
        return resvect;
    }

    std::vector<double> VDeg2Rad(std::vector<double> vect0)
    {
        std::vector<double> resvect;
        resvect.resize(0);
        for (int i = 0; i < int(vect0.size()); i++)
        {
            resvect.push_back(Deg2Rad(vect0[i]));
        }
        return resvect;
    }

    double Rad2Deg(double x)
    {
        double res = x * 180.0 / PI;
        if (res >= 0)
        {
            return fmod(res, 360.0);
        }
        else
        {
            return -1.0 * fmod(-1.0 * res, 360.0);
        }
    }

    double Deg2Rad(double x)
    {
        // double res;
        if (x >= 0)
        {
            return fmod(x, 360.0) * PI / 180.0;
        }
        else
        {
            return -1.0 * fmod(-1.0 * x, 360.0) * PI / 180;
        }
    }

} // close namespace denso_controller
