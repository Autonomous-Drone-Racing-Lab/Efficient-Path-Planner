#pragma once

#include "World.h"
#include "ConfigParser.h"
#include <Eigen/Dense>
#include "Types.h"
#include <ompl-1.6/ompl/base/StateSpace.h>
#include <ompl-1.6/ompl/base/SpaceInformation.h>
#include <ompl-1.6/ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <memory>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/Planner.h>


class PathPlanner{
    public: 
    PathPlanner(const Eigen::MatrixXd& nominalGatePositionAndType, const Eigen::MatrixXd& nominalObstaclePosition, std::shared_ptr<ConfigParser> configParser);

    bool planPath(const Eigen::Vector3d& start, const Eigen::Vector3d& goal, const double timeLimit, std::vector<Eigen::Vector3d>& resultPath) const;
    void updateGatePos(const int gateId, const Eigen::Vector3d& newPose, const bool subtractHeight);
    
    std::set<int> checkTrajectoryValidityAndReturnCollisionIdx(const Eigen::MatrixXd &trajectoryPoints, const std::vector<Eigen::Vector3d> &prunedPath, int number_check_next_samples=-1) const;
    bool checkTrajectoryValidity(const Eigen::MatrixXd& trajectory, const double minDistance) const;
    std::vector<Eigen::Vector3d> includeGates(const std::vector<std::vector<Eigen::Vector3d>> &waypoints) const;
    std::vector<Eigen::Vector3d> includeGates2(std::vector<std::vector<Eigen::Vector3d>> waypoints) const;

    private:
    std::vector<Eigen::Vector3d> pruneWaypoints(const std::vector<Eigen::Vector3d> &waypoints) const;

    ompl::base::StateSpacePtr space;
    ompl::base::SpaceInformationPtr si;
    std::shared_ptr<World> worldPtr;
    std::shared_ptr<ConfigParser> configParser;

    ompl::base::OptimizationObjectivePtr getStraightLineObjective(const ompl::base::SpaceInformationPtr& si, const Eigen::Vector3d& start, const Eigen::Vector3d& goal, const double optimalityThresholdPercentage);
};