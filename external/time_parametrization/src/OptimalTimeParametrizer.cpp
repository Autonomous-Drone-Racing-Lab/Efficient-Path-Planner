#include <Eigen/Dense>
#include <vector>
#include "OptimalTimeParametrizer.h"
#include "Trajectory.h"
#include "Path.h"
#include <list>
#include <iostream>
#include <stdexcept>
namespace OptimalTimeParametrizer
{
    Eigen::MatrixXd calculateTrajectory(const std::vector<Eigen::Vector3d> &waypoints, const std::vector<Eigen::Vector3d> preWaypoints, const double v_max, const double a_max, const double startTimeOffset, const double samplingInterval, const double maxDivergence)
    {
        std::list<Eigen::VectorXd> waypointsList(waypoints.begin(), waypoints.end());
        std::list<Eigen::VectorXd> preWaypointsList(preWaypoints.begin(), preWaypoints.end());
        std::list<Eigen::VectorXd> allWaypoints;
        allWaypoints.insert(allWaypoints.end(), preWaypointsList.begin(), preWaypointsList.end());
        allWaypoints.insert(allWaypoints.end(), waypointsList.begin(), waypointsList.end());

        const Path path = Path(allWaypoints, maxDivergence);
        const Eigen::Vector3d vMaxVec(v_max, v_max, v_max);
        const Eigen::Vector3d aMaxVec(a_max, a_max, a_max);
        const Trajectory traj(path, vMaxVec, aMaxVec);

        if (!traj.isValid())
        {
            throw std::runtime_error("Trajectory is not valid");
        }

        const double duration = traj.getDuration();
        const int noWaypoints = duration / samplingInterval;

        const Eigen::Vector3d trueStart = waypoints[0];
        int offsetIdx = 0;
        double minDistance = 1000;
        for (int i = 0; i < noWaypoints; ++i)
        {
            const double t = i * samplingInterval;
            const Eigen::Vector3d pos = traj.getPosition(t);
            const double dist = (pos - trueStart).norm();
            // std::cout << "Distance " << dist << std::endl;
            if (dist < minDistance)
            {
                offsetIdx = i;
                minDistance = dist;
            }
        }
        Eigen::MatrixXd result(noWaypoints - offsetIdx, 11);
        for (int i = 0; i < noWaypoints - offsetIdx; ++i)
        {
            const double t = (i + offsetIdx) * samplingInterval;
            if (t >= duration)
            {
                std::cerr << "Sampling larger than duration" << std::endl;
            }
            const Eigen::Vector3d pos = traj.getPosition(t);
            const Eigen::Vector3d vel = traj.getVelocity(t);
            const Eigen::Vector3d acc = traj.getAcceleration(t);

            // convert vel to yaw in radians
            double yaw;
            const double vel_x = vel(0);
            const double vel_y = vel(1);
            if (vel_x == 0 && vel_y == 0)
            {
                yaw = 0;
            }
            else if (vel_x == 0)
            {
                if (vel_y > 0)
                {
                    yaw = M_PI / 2;
                }
                else
                {
                    yaw = -M_PI / 2;
                }
            }
            else if (vel_y == 0)
            {
                if (vel_x > 0)
                {
                    yaw = 0;
                }
                else
                {
                    yaw = M_PI;
                }
            }
            else
            {
                yaw = atan2(vel_y, vel_x);
            }

            // std::cout << "Pos " << pos.transpose() << std::endl;

            result(i, 0) = pos(0);
            result(i, 1) = vel(0);
            result(i, 2) = acc(0);
            result(i, 3) = pos(1);
            result(i, 4) = vel(1);
            result(i, 5) = acc(1);
            result(i, 6) = pos(2);
            result(i, 7) = vel(2);
            result(i, 8) = acc(2);
            result(i, 9) = yaw;
            result(i, 10) = (i * samplingInterval) + startTimeOffset;
        }
        return result;
    }
}