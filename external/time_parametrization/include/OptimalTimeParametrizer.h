#include <Eigen/Dense>
#include <vector>

namespace OptimalTimeParametrizer{
    
    /// @brief Calculate trajectory from list of waypoints using optimal time parametrization
    /// @param waypoints Waypoints the trajectory should pass through
    /// @param preWaypoints As we cannot define non-zero initial conditions. Append sufficient pre-waypoints sucht that trajectory matches the initial conditions when waypoints are reached
    /// @param v_max 
    /// @param a_max 
    /// @param startTimeOffset Each time step in the trajectory is offseted by this ammount to make trajectory compatible with local time
    /// @param samplingInterval 
    /// @param maxDivergence How much divergence is maximal acceptable during smoothin of waypoints. When an already smooth path is given this value can be set small
    /// @return 
    Eigen::MatrixXd calculateTrajectory(const std::vector<Eigen::Vector3d>& waypoints, const std::vector<Eigen::Vector3d> preWaypoints, const double v_max, const double a_max, const double startTimeOffset, const double samplingInterval, const double maxDivergence);
}