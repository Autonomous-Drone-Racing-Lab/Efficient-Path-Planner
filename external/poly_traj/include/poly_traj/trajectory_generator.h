#pragma once
#include <Eigen/Dense>
#include <vector>

namespace poly_traj{

/**
 * Generates a trajectory based on the given waypoints and motion constraints.
 *
 * @param waypoints The waypoints that define the desired path.
 * @param v_max The maximum velocity allowed for the trajectory.
 * @param a_max The maximum acceleration allowed for the trajectory.
 * @param sampling_intervall The time interval between consecutive samples on the trajectory.
 * @param startTimeOffset The offset in time from the start of the trajectory.
 * @param initialVel The initial velocity of the trajectory.
 * @param initialAcc The initial acceleration of the trajectory.
 * @param result The resulting trajectory matrix.
 * @return True if the trajectory generation is successful, false otherwise.
 */
bool generateTrajectory(const std::vector<Eigen::Vector3d> &waypoints, double v_max, double a_max, double sampling_intervall, const double startTimeOffset, const Eigen::Vector3d& initialVel, const Eigen::Vector3d& initialAcc, Eigen::MatrixXd &result);
} // namespace poly_traj