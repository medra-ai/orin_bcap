#include <vector>
#include <cstddef>

const size_t _DOF = 6;

struct RobotTrajectory {
  const size_t dimension = _DOF;
  std::vector<std::vector<double>> trajectory;

  size_t size() const {
    return trajectory.size();
  }
};