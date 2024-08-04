# Test the denso_pybind module

import copy
import medra_bcap

import numpy as np

def test_trajectory(curr_joint_angles=[0, 0, 0, 0, 0, 0]):
    # Create a test trajectory
    wave_traj = [curr_joint_angles]
    period = 100

    for i in range(1000):
        curr_joint_angles = wave_traj[-1]
        wave_value = 0.001* np.sin(2 * np.pi * i/period)
        new_joint_angles = curr_joint_angles
        new_joint_angles[5] += float(wave_value)
        wave_traj.append(copy.deepcopy(new_joint_angles))

    # print("Test trajectory: ", wave_traj)
    traj = medra_bcap.RobotTrajectory()
    traj.trajectory = wave_traj

    return traj

def main():
    print("Test the denso_pybind module")
    controller = medra_bcap.DensoController()

    controller.bCapEnterProcess()
    print("Entered bCap process")

    # Get the current joint angles
    curr_joint_angles = controller.GetCurJnt()
    curr_joint_angles = np.radians(curr_joint_angles[:6])
    print("Current joint angles: ", curr_joint_angles)

    # Execute a trajectory
    traj = test_trajectory(curr_joint_angles)
    print("Start Executing trajectory")
    controller.executeServoTrajectory(traj)
    print("Finish executing trajectory")

    controller.bCapExitProcess()
    print("Exited bCap process")





if __name__ == "__main__":
    main()