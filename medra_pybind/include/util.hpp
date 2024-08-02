#include <vector>

const int _DOF = 6;

struct RobotTrajectory {
  const int dimension = _DOF;
  std::vector<std::vector<double>> trajectory;

  size_t size() const {
    return trajectory.size();
  }
};