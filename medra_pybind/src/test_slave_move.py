import numpy as np

from medra_robotics.arm_control.denso_controller import (
    DENSO_SERVO_MODE,
    DensoController,
)
from medra_robotics.utils.typing import JointPosition


def set_realtime_priority():
    import os
    import psutil
    import ctypes

    try:
        # Set process priority to real-time
        pid = os.getpid()
        p = psutil.Process(pid)

        # Set nice value to -175 (requires root privileges)
        os.nice(-175)

        # Set scheduler to FIFO
        param = ctypes.c_int(1)  # SCHED_FIFO
        result = ctypes.cdll.LoadLibrary("libc.so.6").sched_setscheduler(
            pid, param, None
        )

        if result != 0:
            print("Failed to set scheduler. This might require root privileges.")

        # Set real-time priority (requires root privileges)
        p.nice(psutil.REALTIME_PRIORITY_CLASS)

        print(f"Process priority set to real-time with nice value: {os.nice(0)}")
        print(f"Scheduler set to FIFO")
    except Exception as e:
        print(f"Error setting priority: {e}")
        print(
            "This script may require root privileges to set real-time priority and FIFO scheduler."
        )


def test_slave_move(controller: DensoController):
    """oscillate j6 back and forth"""
    controller._robot_control.robot_execute(  # type: ignore
        controller._robot,  # type: ignore
        "slvChangeMode",
        DENSO_SERVO_MODE,
    )
    current_position = controller.get_joint_positions()
    for i in range(10000):
        print(current_position)
        current_position = JointPosition(
            [
                current_position[0],
                current_position[1],
                current_position[2],
                current_position[3],
                current_position[4],
                current_position[5] + 0.001 * np.sin(2 * np.pi * i / 100),
                0.0,
                0.0,
            ]
        )
        print(current_position)
        controller._command_servo_j(current_position)  # type: ignore
    controller._command_servo_j(current_position)  # type: ignore
    controller._command_servo_j(current_position)  # type: ignore
    controller._command_servo_j(current_position)  # type: ignore
    controller._command_servo_j(current_position)  # type: ignore
    controller._command_servo_j(current_position)  # type: ignore
    controller._command_servo_j(current_position)  # type: ignore
    controller._robot_control.robot_execute(  # type: ignore
        controller._robot,  # type: ignore
        "slvChangeMode",
        0x000,
    )


def main():
    # set_realtime_priority()
    controller = DensoController()
    controller.start()
    with controller._arm_mutex:  # type: ignore
        test_slave_move(controller)
    controller.stop()


if __name__ == "__main__":
    main()
