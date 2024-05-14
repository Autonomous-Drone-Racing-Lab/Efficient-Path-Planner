#pragma once

#include "PathPlanner.h"
#include "ConfigParser.h"
#include "Types.h"
#include <Eigen/Dense>
#include <memory>

class OnlineTrajGenerator
{

public:
    OnlineTrajGenerator(const Eigen::Vector3d start, const Eigen::Vector3d goal, const Eigen::MatrixXd &nominalGatePositionAndType, const Eigen::MatrixXd &nominalObstaclePosition, const std::string &configPath);

    void preComputeTraj(const double takeoffTime);
    bool updateGatePos(const int gateId, const Eigen::VectorXd &newPose,  const Eigen::Vector3d& dronePos, const bool nextGateWithinRange, const double flightTime);

    Eigen::VectorXd sampleTraj(const double currentTime) const;
    double getTrajEndTime() const;
    Eigen::MatrixXd getPlannedTraj() const;


private:
    std::shared_ptr<ConfigParser> configParser;
    PathPlanner pathPlanner;

    Eigen::MatrixXd nominalGatePositionAndType;
    std::vector<Eigen::Vector3d> checkpoints;
    std::set<int> gatesObservedWithinRange;
    std::map<int, float> outstandingGateUpdates;
    std::vector<Eigen::MatrixXd> pathSegments;
    Eigen::MatrixXd plannedTraj;

    bool getGateCenterAndNormal(const Eigen::VectorXd &gatePostAndType, const bool subtractHeight, Eigen::Vector3d &center, Eigen::Vector3d &normal);
};