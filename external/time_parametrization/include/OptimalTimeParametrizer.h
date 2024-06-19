#include <Eigen/Dense>
#include <vector>

namespace OptimalTimeParametrizer{
    Eigen::MatrixXd calculateTrajectory(const std::vector<Eigen::Vector3d>& waypoints, const std::vector<Eigen::Vector3d> preWaypoints, const double v_max, const double a_max, const double startTimeOffset, const double samplingInterval, const double maxDivergence);
}