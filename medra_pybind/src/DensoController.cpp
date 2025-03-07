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
    DensoArmMutex::DensoArmMutex(DensoReadWriteDriver &driver) : driver(driver) {
        uuid = std::to_string(rand());
    }

    bool DensoArmMutex::Claim()
    {
        SPDLOG_INFO("DensoArmMutex " + uuid + " Claim.");

        BCAP_HRESULT hr = driver.TakeArm();
        if (FAILED(hr)) {
            SPDLOG_ERROR(
                "Failed to take arm. Error code: " + std::to_string(hr)
            );
            return false;
        }

        hr = driver.Motor(true);
        if (FAILED(hr)) {
            SPDLOG_ERROR(
                "Failed to turn motor on. Error code: " + std::to_string(hr)
            );
            return false;
        }

        return true;
    }

    DensoArmMutex::~DensoArmMutex()
    {
        SPDLOG_INFO("DensoArmMutex " + uuid + " Release.");
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

    BCAP_HRESULT DensoReadDriver::GetCurJnt(JointPosition &joint_positions)
    {
        double dJnt[8];
        BCAP_HRESULT hr = bCap_RobotExecute(iSockFD, lhRobot, "CurJnt", "", &dJnt);
        if (FAILED(hr))
        {
            SPDLOG_ERROR("Failed to get current joint values. Error code: " + std::to_string(hr));
            return hr;
        }

        for (size_t i = 0; i < joint_positions.size(); i++)
        {
            joint_positions[i] = dJnt[i];
        }
        return hr;
    }

    BCAP_HRESULT DensoReadDriver::GetForceValue(ForceTorque &force_values)
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
        }
        if (FAILED(hr))
        {
            SPDLOG_ERROR("Failed to get current force values. Error code: " + std::to_string(hr));
            return hr;
        }

        for (size_t i = 0; i < force_values.size(); ++i)
        {
            force_values[i] = dForce[i];
        }
        return hr;
    }

    std::tuple<BCAP_HRESULT, std::vector<double>>
    DensoReadDriver::GetMountingCalib(const char *work_coordinate)
    {
        double work_def[8];
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
        SPDLOG_INFO("TakeArm()");
        long lResult;
        return bCap_RobotExecute(iSockFD, lhRobot, "TakeArm", "", &lResult);
    }

    BCAP_HRESULT DensoReadWriteDriver::GiveArm()
    {
        SPDLOG_INFO("GiveArm()");
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
        arm_mutex.Claim();

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
        return bCap_RobotExecute(iSockFD, lhRobot, "slvChangeMode", mode, &lResult);
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
            SPDLOG_ERROR("Failed to execute ForceSensor. Error code: " + std::to_string(hr));
        }
        return hr;
    }

    DensoController::DensoController() : read_driver(DensoReadDriver()),
                                         write_driver(DensoReadWriteDriver())
    {
        current_waypoint_index = 0;
        atomic_trajectory_execution_enabled = true;
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
        arm_mutex.Claim();

        current_waypoint_index = 0;
    }

    void DensoController::Stop()
    {
        atomic_force_limit_exceeded = true;

        read_driver.Stop();
        write_driver.Stop();
    }

    BCAP_HRESULT DensoController::Motor(bool command)
    {
        return write_driver.Motor(command);
    }

    BCAP_HRESULT DensoController::ManualReset()
    {
        return write_driver.ManualReset();
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

    std::tuple<BCAP_HRESULT, JointPosition>
    DensoController::GetJointPositions()
    {
        JointPosition joint_positions;
        BCAP_HRESULT hr = read_driver.GetCurJnt(joint_positions);

        // Convert joint positions to radians
        for (size_t i = 0; i < joint_positions.size(); i++)
        {
            joint_positions[i] = Deg2Rad(joint_positions[i]);
        }
        return {hr, joint_positions};
    }

    bool DensoController::GetTrajectoryExecutionEnabled()
    {
        return atomic_trajectory_execution_enabled;
    }

    void DensoController::SetTrajectoryExecutionEnabled(bool enabled)
    {
        SPDLOG_INFO("Setting trajectory execution enabled to " + std::to_string(enabled));
        atomic_trajectory_execution_enabled = enabled;
    }

    TrajectoryExecutionResult
    DensoController::ExecuteServoTrajectory(
        const RobotTrajectory &traj,
        const std::optional<double> total_force_limit,
        const std::optional<double> total_torque_limit,
        const std::optional<ForceTorque> per_axis_force_torque_limits)
    {
        current_waypoint_index = 0;

        auto arm_mutex = DensoArmMutex(write_driver);
        arm_mutex.Claim();

        if (total_force_limit.has_value()
            || total_torque_limit.has_value()
            || per_axis_force_torque_limits.has_value()
        ) {
            // Wait some time for the arm and sensor to settle.
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            // Reset the force sensor to prevent drift in the force readings.
            // Do it here, instead of in the force sensing thread, because this call
            // requires the arm mutex.
            BCAP_HRESULT hr = write_driver.ForceSensor("0");
            if (FAILED(hr))
            {
                return {
                    ExecuteServoTrajectoryError::FORCE_SENSOR_RESET_FAILED,
                    ExecuteServoTrajectoryResult::ERROR,
                    TimestampedTrajectory(),
                    TimestampedForceSequence()
                };
            }
        }

        // Start a thread to stream force sensor data
        atomic_force_limit_exceeded = false;
        // Pre-allocate memory for the force-torque values to avoid
        // reallocation during real-time execution. In practice we should only
        // need to store around
        //   FORCE_SENSING_FREQUENCY_HZ / SERVO_COMMAND_FREQUENCY_HZ
        // values, but allocate more to be safe.
        const size_t FT_VECTOR_RESERVE_SIZE_FACTOR = 3;
        const size_t total_ft_values = (
            FT_VECTOR_RESERVE_SIZE_FACTOR
            * traj.size()
            * FORCE_SENSING_FREQUENCY_HZ
            / SERVO_COMMAND_FREQUENCY_HZ
        );
        TimestampedForceSequence force_torque_values(total_ft_values);
        std::thread force_sensing_thread(
            &DensoController::RunForceSensingLoop,
            this,
            total_force_limit,
            total_torque_limit,
            per_axis_force_torque_limits,
            std::ref(force_torque_values)
        );

        // Enter slave mode
        SPDLOG_INFO("Entering slave mode");
        switch (EnterSlaveMode())
        {
            case EnterSlaveModeResult::SUCCESS:
                SPDLOG_INFO("Slave mode ON");
                break;
            case EnterSlaveModeResult::ENTER_SLAVE_MODE_FAILED:
                SPDLOG_ERROR("Failed to enter b-CAP slave mode.");
                atomic_force_limit_exceeded = true;
                force_sensing_thread.join();
                return {
                    ExecuteServoTrajectoryError::ENTER_SLAVE_MODE_FAILED,
                    ExecuteServoTrajectoryResult::ERROR,
                    TimestampedTrajectory(),
                    TimestampedForceSequence()
                };
        }

        // Execute the trajectory
        CommandServoJointResult result;
        bool force_limit_exceeded = false;
        bool trajectory_stopped_early = false;

        // Reserve memory for the joint positions here to avoid
        // potentially expensive reallocations during real-time execution.
        TimestampedTrajectory joint_positions(traj.size());

        for (size_t i = 0; i < traj.size(); i++)
        {
            // Stop if the force exceedance condition is true
            if (i > 4 && atomic_force_limit_exceeded)
            {
                SPDLOG_INFO("Force limit exceeded after "
                            + std::to_string(i)
                            + " of " + std::to_string(traj.size())
                            + " waypoints. Stopping trajectory execution.");
                force_limit_exceeded = true;
                break;
            }
            if (i > 4 && !atomic_trajectory_execution_enabled)
            {
                SPDLOG_INFO("Trajectory execution has been disabled at waypoint index "
                            + std::to_string(i) + " of "
                            + std::to_string(traj.size())
                            + ". Stopping trajectory execution early.");
                trajectory_stopped_early = true;
                SetTrajectoryExecutionEnabled(true);  // Reset the flag for the next trajectory.
                break;
            }

            current_waypoint_index = i;
            const auto &joint_position = traj[i];
            result = CommandServoJoint(joint_position);

            joint_positions[i].time = std::chrono::system_clock::now();
            joint_positions[i].joint_position = joint_position;

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
                    atomic_force_limit_exceeded = true;
                    force_sensing_thread.join();

                    joint_positions.resize(current_waypoint_index + 1);

                    // No need to exit slave mode here because the controller
                    // releases slave mode when an error occurs.
                    return {
                        ExecuteServoTrajectoryError::SLAVE_MOVE_FAILED,
                        ExecuteServoTrajectoryResult::ERROR,
                        joint_positions,
                        force_torque_values
                    };
            }
        }

        bool exec_complete = !force_limit_exceeded && !trajectory_stopped_early;
        // Stop the force sensing thread by triggering the stop condition.
        // Don't join the force sensing thread here because it may block this
        // thread for too long, causing command position buffer underflow.
        atomic_force_limit_exceeded = true;

        // Close loop servo commands on last waypoint.
        if (exec_complete)
        {
            switch (ClosedLoopCommandServoJoint(traj[current_waypoint_index]))
            {
                case ClosedLoopCommandServoJointResult::SUCCESS:
                    SPDLOG_INFO("Closed loop servo joint commands successful");
                    break;
                case ClosedLoopCommandServoJointResult::GET_CUR_JNT_FAILED:
                case ClosedLoopCommandServoJointResult::SLAVE_MOVE_FAILED:
                    // The trajectory is essentially complete at this point, so we
                    // don't do anything special if the closed loop commands fail.
                    SPDLOG_ERROR("Closed loop servo joint commands failed");
                    break;
            }
        }
        force_sensing_thread.join();

        ExecuteServoTrajectoryResult trajectory_result;
        if (exec_complete) {
            trajectory_result = ExecuteServoTrajectoryResult::COMPLETE;
        } else if (force_limit_exceeded) {
            trajectory_result = ExecuteServoTrajectoryResult::FORCE_LIMIT_EXCEEDED;
        } else {  // trajectory_stopped_early
            trajectory_result = ExecuteServoTrajectoryResult::EARLY_STOP_REQUESTED;
        }

        SPDLOG_INFO("Turning off slave mode");
        switch (ExitSlaveMode())
        {
            case ExitSlaveModeResult::SUCCESS:
                SPDLOG_INFO("Slave mode OFF");
                break;
            case ExitSlaveModeResult::EXIT_SLAVE_MODE_FAILED:
                SPDLOG_ERROR("Failed to exit b-CAP slave mode.");
                joint_positions.resize(current_waypoint_index + 1);
                return {
                    ExecuteServoTrajectoryError::EXIT_SLAVE_MODE_FAILED,
                    trajectory_result,
                    joint_positions,
                    force_torque_values
                };
        }

        joint_positions.resize(current_waypoint_index + 1);
        return {
            ExecuteServoTrajectoryError::SUCCESS,
            trajectory_result,
            joint_positions,
            force_torque_values
        };
    }

    void DensoController::RunForceSensingLoop(
        const std::optional<double> total_force_limit,
        const std::optional<double> total_torque_limit,
        const std::optional<ForceTorque> per_axis_force_torque_limits,
        TimestampedForceSequence& force_torque_sequence
    )
    {
        // Exit early if no force limits are specified
        if (!total_force_limit.has_value() && !total_torque_limit.has_value() && !per_axis_force_torque_limits.has_value())
        {
            SPDLOG_INFO("Skip force sensing loop");
            return;
        }

        const size_t total_ft_values = force_torque_sequence.size();
        const int period = 1000 / FORCE_SENSING_FREQUENCY_HZ; // period in milliseconds
        const int filter_size = 3;
        filter::MeanFilter<FORCE_TORQUE_DOF> mean_filter(filter_size);

        BCAP_HRESULT hr;
        ForceTorque ft_values;
        size_t ft_sequence_index = 0;
        while (!atomic_force_limit_exceeded)
        {
            if (ft_sequence_index >= total_ft_values)
            {
                SPDLOG_ERROR("Force-torque value vector out of space. Size: "
                                + std::to_string(total_ft_values));
                break;
            }

            auto start = std::chrono::steady_clock::now();

            // Check the force threshold
            hr = read_driver.GetForceValue(ft_values);
            if (FAILED(hr))
            {
                HandleError(hr, "GetForceValue failed.");
            }
            mean_filter.AddValue(ft_values);

            // Use the mean of the last filter_size force values
            ft_values = mean_filter.GetMean();

            // Update the log of force-torque readings
            force_torque_sequence[ft_sequence_index].time = std::chrono::system_clock::now();
            force_torque_sequence[ft_sequence_index].force_torque_values = ft_values;
            ft_sequence_index++;

            // Check the total force does not exceed the limit
            double total_force = std::sqrt(
                std::pow(ft_values[0], 2) + std::pow(ft_values[1], 2) + std::pow(ft_values[2], 2));
            if (total_force_limit.has_value() && total_force > *total_force_limit)
            {
                SPDLOG_INFO("Force limit exceeded. Total force: " + std::to_string(total_force)
                            + ". Force limit: " + std::to_string(*total_force_limit));
                atomic_force_limit_exceeded = true;
                break;
            }

            // Check the total torque does not exceed the limit
            double total_torque = std::sqrt(
                std::pow(ft_values[3], 2) + std::pow(ft_values[4], 2) + std::pow(ft_values[5], 2));
            if (total_torque_limit.has_value() && total_torque > *total_torque_limit)
            {
                SPDLOG_INFO("Torque limit exceeded. Total torque: " + std::to_string(total_torque)
                            + ". Torque limit: " + std::to_string(*total_torque_limit));
                atomic_force_limit_exceeded = true;
                break;
            }

            // Check the TCP force/torque does not exceed the limit
            if (per_axis_force_torque_limits.has_value())
            {
                for (int i = 0; i < per_axis_force_torque_limits->size(); i++)
                {
                    if (std::abs(ft_values[i]) > (*per_axis_force_torque_limits)[i])
                    {
                        SPDLOG_INFO("TCP force/torque limit exceeded. Force/Torque: " + std::to_string(ft_values[i]));
                        atomic_force_limit_exceeded = true;
                        break;
                    }
                }
            }

            // High-precision sleep to maintain the desired frequency
            auto end = std::chrono::steady_clock::now();
            std::chrono::duration<double, std::milli> elapsed = end - start;
            std::this_thread::sleep_for(std::chrono::milliseconds(period) - elapsed);
        }

        // Truncate to the actual size
        SPDLOG_INFO("Force-torque reading populated "
                        + std::to_string(ft_sequence_index) + " of "
                        + std::to_string(total_ft_values) + " reserved values.");
        force_torque_sequence.resize(ft_sequence_index);
    }

    DensoController::ClosedLoopCommandServoJointResult
    DensoController::ClosedLoopCommandServoJoint(const JointPosition& last_waypoint)
    {
        SPDLOG_INFO("Closed loop servo joint commands");
        /* Repeat the last servo j command until the robot reaches tolerance or timeout */
        const double CLOSE_LOOP_JOINT_ANGLE_TOLERANCE = 0.0001; // rad
        const int CLOSE_LOOP_MAX_ITERATION = 10;
        JointPosition current_jnt_deg;
        BCAP_HRESULT hr = write_driver.GetCurJnt(current_jnt_deg);
        JointPosition current_jnt_rad = VDeg2Rad(current_jnt_deg);
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
    DensoController::CommandServoJoint(const JointPosition &joint_position)
    {
        BCAP_VARIANT vntPose = VNTFromRadVector(joint_position);
        BCAP_VARIANT vntReturn;
        BCAP_HRESULT hr;
        hr = write_driver.SlvMove(&vntPose, &vntReturn);
        if (SUCCEEDED(hr))
        {
            return CommandServoJointResult::SUCCESS;
        }
        SPDLOG_ERROR("Failed to command servo joint. Resetting robot. Error code: " + std::to_string(hr));

        write_driver.SlvChangeMode(SERVO_MODE_OFF);;
        SPDLOG_INFO("Cleared errors after failed servo command.");

        return CommandServoJointResult::SLAVE_MOVE_FAILED;
    }

    ////////////////////////////// Utilities //////////////////////////////

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

    BCAP_VARIANT DensoController::VNTFromRadVector(const JointPosition &vect0)
    {
        BCAP_VARIANT vnt;
        vnt.Type = VT_R8 | VT_ARRAY;
        vnt.Arrays = 8;
        for (int i = 0; i < int(vect0.size()); i++)
        {
            vnt.Value.DoubleArray[i] = Rad2Deg(vect0[i]);
        }
        return vnt;
    }

    JointPosition VRad2Deg(const JointPosition &vect0)
    {
        JointPosition resvect;
        for (size_t i = 0; i < vect0.size(); ++i)
        {
            resvect[i] = Rad2Deg(vect0[i]);
        }
        return resvect;
    }

    JointPosition VDeg2Rad(const JointPosition &vect0)
    {
        JointPosition resvect;
        for (size_t i = 0; i < vect0.size(); ++i)
        {
            resvect[i] = Deg2Rad(vect0[i]);
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
