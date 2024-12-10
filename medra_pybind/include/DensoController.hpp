/**
 * C++ Wrapper around Denso b-CAP library
 * Based on https://github.com/quangounet/denso/blob/master/cpp/src/DensoController.h
 */ 

#ifndef DensoController_hpp
#define DensoController_hpp

#include "b-Cap.h"
#include <exception>
#include <string>
#include <cassert>
#include <vector>
#include <tuple>
#include <atomic>
#include <optional>
#include <chrono>

#define DEFAULT_SERVER_IP_ADDRESS    "192.168.0.1"
#define DEFAULT_SERVER_PORT_NUM      5007

#define E_BUF_FULL           0x83201483
#define S_BUF_FULL           0x0F200501

#define PI 3.1415926535897932
#define nSEC_PER_SECOND 1E9
#define dReal float

#define SERVO_MODE_ON "514"  // Mode 2 J-type
#define SERVO_MODE_OFF "0"   // Servo mode off

const size_t JOINT_DOF = 6;
const size_t FORCE_TORQUE_DOF = 6;
const size_t SERVO_COMMAND_FREQUENCY_HZ = 100;
const size_t FORCE_SENSING_FREQUENCY_HZ = 50;

namespace denso_controller {

// Joint position type without auxiliary axes
using JointPosition = std::array<double, JOINT_DOF>;
using ForceTorque = std::array<double, FORCE_TORQUE_DOF>;
using RobotTrajectory = std::vector<JointPosition>;

struct TimestampedWaypoint {
    std::chrono::system_clock::time_point time;
    JointPosition joint_position;
};
using TimestampedTrajectory = std::vector<TimestampedWaypoint>;

struct TimestampedForceReading {
    std::chrono::system_clock::time_point time;
    ForceTorque force_torque_values;
};
using TimestampedForceSequence = std::vector<TimestampedForceReading>;

enum class ExecuteServoTrajectoryError {
    SUCCESS,
    ENTER_SLAVE_MODE_FAILED,
    SLAVE_MOVE_FAILED,
    EXIT_SLAVE_MODE_FAILED,
    FORCE_SENSOR_RESET_FAILED
};
enum class ExecuteServoTrajectoryResult {
    COMPLETE,
    FORCE_LIMIT_EXCEEDED,
    ERROR,
    EARLY_STOP_REQUESTED
};

// Log data for a single trajectory execution.
struct TrajectoryExecutionResult {
    ExecuteServoTrajectoryError error_code;
    ExecuteServoTrajectoryResult result_code;
    // Log of times and joint positions for each waypoint in the trajectory.
    TimestampedTrajectory joint_positions;
    // Log of times and force/torque values for each waypoint in the trajectory.
    TimestampedForceSequence force_torque_values;
};

class bCapException : public std::exception {
public:
    bCapException(std::string msg) {
        _err_msg = msg;
    }

    bCapException(std::string msg, std::string err_description) {
        _err_msg = msg + " Description: " + err_description;
    }

    const char* what() const noexcept override {
        return _err_msg.c_str();
    }

private:
    std::string _err_msg;
};

// Custom exceptions for ExecuteServoTrajectory
class SlaveMoveException : public std::exception {
public:
    SlaveMoveException(std::string msg="") {
        _err_msg = "Failed to Execute SlaveMove " + msg;
    }

    const char* what() const noexcept override {
        return _err_msg.c_str();
    }

private:
    std::string _err_msg;
};
class EnterSlaveModeException : public std::exception {
public:
    EnterSlaveModeException(std::string msg="") {
        _err_msg = "Failed to ENTER slave mode " + msg;
    }

    const char* what() const noexcept override {
        return _err_msg.c_str();
    }
private:
    std::string _err_msg;
};
class ExitSlaveModeException : public std::exception {
public:
    ExitSlaveModeException(std::string msg="") {
        _err_msg = "Failed to EXIT slave mode " + msg;
    }

    const char* what() const noexcept override {
        return _err_msg.c_str();
    }
private:
    std::string _err_msg;
};

// Maintains a connection to a Denso RC9 controller and provides wrapper
// functions for common "read-only" operations, which do not change the
// state of the controller.
class DensoReadDriver {
public:
    DensoReadDriver();

    // Setup and teardown functions
    void Start();
    void Stop();

    BCAP_HRESULT ClearError();

    //// Read functions
    // Populates joint_positions with the current joint values.
    // joint_positions is mutated into a vector of 8 doubles.
    // The first 6 values are the joint angles in degrees, and the last 2
    // values are the positions of the auxiliary axes, or 0 if they are not
    // used.
    BCAP_HRESULT GetCurJnt(JointPosition& joint_positions);
    // Populates force_values with the current force values.
    // force_values is mutated into a vector of 6 doubles.
    // The first 3 values are the force values in Newtons, and the last 3
    // values are the torque values in Newton-meters.
    BCAP_HRESULT GetForceValue(ForceTorque& force_values);
    // TODO: Change this function signature to match the Denso b-CAP API.
    std::tuple<BCAP_HRESULT, std::vector<double>> GetMountingCalib(const char* work_coordinate);
    std::string GetErrorDescription(BCAP_HRESULT error_code);

protected:
    const char* session_name;
    const char* server_ip_address;
    int server_port_num;
    int iSockFD;
    uint32_t lhController;
    uint32_t lhRobot;

    // Setup and teardown functions
    void bCapOpen();
    void bCapClose();
    void bCapServiceStart();
    void bCapServiceStop();
    void bCapControllerConnect();
    void bCapControllerDisconnect();
    void bCapGetRobot();
    void bCapReleaseRobot();
};

// Inherits from DensoReadDriver and adds wrapper functions for "write"
// operations, which may change the state of the controller.
class DensoReadWriteDriver : public DensoReadDriver {
public:
    DensoReadWriteDriver();

    BCAP_HRESULT ManualReset();
    BCAP_HRESULT Motor(bool command);
    BCAP_HRESULT TakeArm();
    BCAP_HRESULT GiveArm();
    BCAP_HRESULT SetExtSpeed(const char* speed);
    // TODO: Change this function signature to match the Denso b-CAP API.
    BCAP_HRESULT SetTcpLoad(const int32_t tool_value);

    // Executes a command on the robot controller. Use for commands with string args.
    // BCAP_HRESULT RobotExecute(const char* command, const char* option);

    // Movement commands
    BCAP_HRESULT SlvChangeMode(const char* mode);
    BCAP_HRESULT SlvMove(BCAP_VARIANT* pose, BCAP_VARIANT* result);

    // Resets the force sensor.
    BCAP_HRESULT ForceSensor(const char* mode);
};

// RAII-style mutex for the Denso arm mutex for axis control.
// See "TakeArm" and "GiveArm" in the Cobotta Pro manual for more information
// about the Denso arm mutex.
// Claim() calls "TakeArm" and "Motor(1)", and the destructor calls "GiveArm".
class DensoArmMutex {
public:
    DensoArmMutex(DensoReadWriteDriver &driver);
    ~DensoArmMutex();
    bool Claim();

private:
    DensoReadWriteDriver &driver;

    // Random UUID for logging purposes.
    std::string uuid;
};

class DensoController {
public:
    DensoController();

    void Start();
    void Stop();

    // Error handling
    BCAP_HRESULT Motor(bool command);
    BCAP_HRESULT ManualReset();
    BCAP_HRESULT ClearError();
    std::string GetErrorDescription(BCAP_HRESULT error_code);

    // High level commands
    // Returns a tuple containing the error code and the joint positions in
    // radians.
    std::tuple<BCAP_HRESULT, JointPosition> GetJointPositions();

    // Executes a trajectory of joint angles in radians.
    // total_force_limit, total_torque_limit, and per_axis_force_torque_limits
    // describe stopping criteria for trajectory execution based on force
    // sensing. If any of the following are true, then trajectory execution is
    // stopped early:
    //   1. If the total force exceeds total_force_limit N,
    //   2. If the total torque exceeds total_torque_limit Nm, or
    //   3. If the force or torque on any axis exceeds the corresponding limit
    //     in per_axis_force_torque_limits.
    TrajectoryExecutionResult ExecuteServoTrajectory(
        const RobotTrajectory& traj,
        const std::optional<double> total_force_limit = std::nullopt,
        const std::optional<double> total_torque_limit = std::nullopt,
        const std::optional<ForceTorque> per_axis_force_torque_limits = std::nullopt
    );

    BCAP_HRESULT SetTcpLoad(const int32_t tool_value);
    std::tuple<BCAP_HRESULT, std::vector<double>> GetMountingCalib(const char* work_coordinate);

    int current_waypoint_index;

    // Disable/enable trajectory execution.
    bool GetTrajectoryExecutionEnabled();
    void SetTrajectoryExecutionEnabled(bool enabled);

private:
    // Denso b-CAP drivers, one for read-only operations and one for read-write
    // operations.
    DensoReadDriver read_driver;
    DensoReadWriteDriver write_driver;

    // Used to request early termination of trajectory execution via
    // StopTrajectoryExecution().
    std::atomic<bool> atomic_trajectory_execution_enabled;

    // The purpose of this variable is two-fold:
    //   1. The RunForceSensingLoop function only runs while this variable is
    //      false.
    //   2. The RunForceSensingLoop function sets this variable to true if the
    //      force limit is exceeded.
    std::atomic<bool> atomic_force_limit_exceeded;

    // Runs force sensing loop with the read-only Denso driver. Should be used
    // in a separate thread.
    // This function runs while force_limit_exceeded is false.
    // If the force limit is exceeded, force_limit_exceeded is set to true.
    // This function populates force_torque_values with time-stamped
    // force-torque readings.
    void RunForceSensingLoop(
        const std::optional<double> total_force_limit,
        const std::optional<double> total_torque_limit,
        const std::optional<ForceTorque> per_axis_force_torque_limits,
        TimestampedForceSequence& force_torque_values
    );

    enum class EnterSlaveModeResult {
        SUCCESS, ENTER_SLAVE_MODE_FAILED
    };
    EnterSlaveModeResult EnterSlaveMode();

    enum class ExitSlaveModeResult {
        SUCCESS, EXIT_SLAVE_MODE_FAILED
    };
    ExitSlaveModeResult ExitSlaveMode();

    enum class CommandServoJointResult {
        SUCCESS, SLAVE_MOVE_FAILED
    };
    // Commands the robot to move to a joint position, in radians, in slave mode.
    CommandServoJointResult CommandServoJoint(const JointPosition& waypoint);

    enum class ClosedLoopCommandServoJointResult {
        SUCCESS,
        GET_CUR_JNT_FAILED,
        SLAVE_MOVE_FAILED
    };
    // Repeatedly commands a joint position in slave mode until the robot's
    // current joint position is within a small tolerance of it.
    ClosedLoopCommandServoJointResult ClosedLoopCommandServoJoint(const JointPosition& waypoint);

    // Error handling
    void HandleError(BCAP_HRESULT error_code, const char* error_description);

    // Utility functions
    std::vector<double> VectorFromVNT(BCAP_VARIANT vnt0);
    std::vector<double> RadVectorFromVNT(BCAP_VARIANT vnt0);
    BCAP_VARIANT VNTFromVector(std::vector<double> vect0);
    BCAP_VARIANT VNTFromRadVector(const JointPosition &vect0);
};


////////////////////////////// Utilities //////////////////////////////
JointPosition VRad2Deg(const JointPosition &vect0);
JointPosition VDeg2Rad(const JointPosition &vect0);

double Rad2Deg(double x);
double Deg2Rad(double x);

} // close namespace denso_controller

#endif // DensoController_hpp
