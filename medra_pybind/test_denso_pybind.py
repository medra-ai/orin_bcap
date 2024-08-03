# Test the denso_pybind module

import build.denso_pybind as denso

import numpy as np

def test_trajectory(curr_joint_angles=[0, 0, 0, 0, 0, 0]):
    # Create a test trajectory
    curr_joint_angles

    wave_traj = []
    amplitude = 100
    period = 100

    for i in range(100):
        wave_value = amplitude * np.sin(2 * np.pi * i/period)
        new_joint_angles = curr_joint_angles
        new_joint_angles[5] += wave_value.item()
        wave_traj.append(new_joint_angles)

    print("Test trajectory: ", wave_traj)
    traj = denso.RobotTrajectory()
    traj.trajectory = wave_traj

    return traj

def main():
    print("Test the denso_pybind module")
    # denso.bCapEnterProcess()
    print("Entered bCap process")

    # Get the current joint angles
    # curr_joint_angles = denso.GetCurJnt()
    # print("Current joint angles: ", joint_angles)

    # Execute a trajectory
    # traj = test_trajectory(curr_joint_angles)
    traj = test_trajectory()
    print("Start Executing trajectory")
    # denso.executeServoTrajectory(traj)
    print("Finish executing trajectory")

    # denso.bCapExitProcess()
    print("Exited bCap process")





if __name__ == "__main__":
    main()