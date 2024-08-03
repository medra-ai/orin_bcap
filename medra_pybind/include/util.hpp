#include <vector>
#include <cstddef>

const size_t _DOF = 6;

struct RobotTrajectory {
  // Number of joints per trajectory entry
  const size_t dimension = _DOF;

  // Trajectory of joint angles in radians
  std::vector<std::vector<double>> trajectory;

  // Length of the trajectory
  size_t size() const {
    return trajectory.size();
  }
};