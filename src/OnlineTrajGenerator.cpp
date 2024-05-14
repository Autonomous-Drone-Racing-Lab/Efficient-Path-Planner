#include "OnlineTrajGenerator.h"
#include <iostream>
#include "poly_traj/trajectory_generator.h"

OnlineTrajGenerator::OnlineTrajGenerator(const Eigen::Vector3d start, const Eigen::Vector3d goal, const Eigen::MatrixXd &nominalGatePositionAndType, const Eigen::MatrixXd &nominalObstaclePosition, const std::string &configPath)
    : configParser(std::make_shared<ConfigParser>(configPath)),
      pathPlanner(PathPlanner(nominalGatePositionAndType, nominalObstaclePosition, configParser)),
      nominalGatePositionAndType(nominalGatePositionAndType)
{
    // parse checkpoints
    checkpoints.push_back(start);
    for (int i = 0; i < nominalGatePositionAndType.rows(); i++)
    {
        const Eigen::VectorXd &gate = nominalGatePositionAndType.row(i);
        Eigen::Vector3d center, normal;
        getGateCenterAndNormal(gate, center, normal);

        const Eigen::VectorXd earlyCheckpoint = center - checkpointOffset * normal;
        const Eigen::VectorXd lateCheckpoint = center + checkpointOffset * normal;

        checkpoints.push_back(earlyCheckpoint);
        checkpoints.push_back(lateCheckpoint);
    }
    checkpoints.push_back(goal);
}

bool OnlineTrajGenerator::getGateCenterAndNormal(const Eigen::VectorXd &gatePostAndType, Eigen::Vector3d &center, Eigen::Vector3d &normal)
{
    const Eigen::Vector3d &gatePosition = gatePostAndType.head(3);
    const Eigen::Vector3d &gateRotation = gatePostAndType.segment(3, 3);
    const int gateType = gatePostAndType(6);

    // For now only allow simple rotation around z axis
    if (gateRotation(0) != 0 || gateRotation(1) != 0)
    {
        std::cerr << "Only simple rotation around z axis is supported" << std::endl;
        return false;
    }
    const double gateHeight = configParser->getObjectPropertiesByTypeId(gateType).height;
    center = gatePosition + Eigen::Vector3d(0, 0, gateHeight);
    normal << -sin(gateRotation(2)), cos(gateRotation(2)), 0;
    normal.normalize();

    std::cout << "Gate position: " << gatePosition << std::endl;
    std::cout << "Gate rotation: " << gateRotation << std::endl;
    std::cout << "Center: " << center << std::endl;
    std::cout << "Normal: " << normal << std::endl;

    return true;
}

void OnlineTrajGenerator::preComputeTraj(const double takeoffTime)
{
    const double timeLimit = configParser->getPathPlannerProperties().timeLimitOffline;
    for (int i = 0; i < checkpoints.size() - 1; i += 2)
    {
        const Eigen::Vector3d &start = checkpoints[i];
        const Eigen::Vector3d &goal = checkpoints[i + 1];

        std::cout << "Planning path from " << start.transpose() << " to " << goal.transpose() << std::endl;

        const Eigen::MatrixXd path = pathPlanner.planPath(start, goal, timeLimit);
        pathSegments.push_back(path);
    }

    const double v_max = configParser->getTrajectoryGeneratorProperties().maxVelocity;
    const double a_max = configParser->getTrajectoryGeneratorProperties().maxAcceleration;
    const double samplingInterval = configParser->getTrajectoryGeneratorProperties().samplingInterval;

    const Eigen::Vector3d initialVel = Eigen::Vector3d::Zero();
    const Eigen::Vector3d initialAcc = Eigen::Vector3d::Zero();
    poly_traj::generateTrajectory(pathSegments, v_max, a_max, samplingInterval, takeoffTime, initialVel, initialAcc, plannedTraj);
}

void OnlineTrajGenerator::updateGatePos(const int gateId, const Eigen::Vector3d &newPose, const Eigen::Vector3d &dronePos, const bool nextGateWithinRange, const double flightTime)
{
    if (nextGateWithinRange)
    {
        gatesObservedWithinRange.insert(gateId);
    }

    // ignore if gate was already seen close up and is now no longer
    if (gatesObservedWithinRange.find(gateId) != gatesObservedWithinRange.end() && !nextGateWithinRange)
    {
        return;
    }

    const Eigen::Vector3d &newPos = newPose.head(3);
    const Eigen::Vector3d &oldPos = nominalGatePositionAndType.row(gateId).head(3);
    const Eigen::Vector3d &newRot = newPose.segment(3, 3);
    const Eigen::Vector3d &oldRot = nominalGatePositionAndType.row(gateId).segment(3, 3);

    const double posInsinificanceThreshold = 0.05;
    const double rotInsinificanceThreshold = 0.05;
    if ((newPos - oldPos).norm() < posInsinificanceThreshold && (newRot - oldRot).norm() < rotInsinificanceThreshold)
    {
        return;
    }

    // update nominal gate position
    nominalGatePositionAndType.row(gateId).head(6) = newPose;
    pathPlanner.updateGatePos(gateId, nominalGatePositionAndType.row(gateId), nextGateWithinRange); // if next gate within range, the view of the gate changes and we must subtract its height

    // recompute segment of relevance
    const int segmentIdPre = gateId;
    const int segmentIdPost = gateId + 1;
    const int checkpointIdPre = 2 *gateId + 1 ;
    const int checkpointIdPost = 2 * gateId + 2;
    const int checkpointIdNextGate = 2 * gateId + 3;

    // update checkpoints
    Eigen::Vector3d center, normal;
    getGateCenterAndNormal(nominalGatePositionAndType.row(segmentIdPre), center, normal);
    const Eigen::VectorXd earlyCheckpoint = center - checkpointOffset * normal;
    const Eigen::VectorXd lateCheckpoint = center + checkpointOffset * normal;
    checkpoints[checkpointIdPre] = earlyCheckpoint;
    checkpoints[checkpointIdPost] = lateCheckpoint;

    // Recompute first segment, from drone to first checkpoint
    const Eigen::MatrixXd preSegmentPath = pathPlanner.planPath(dronePos, earlyCheckpoint, configParser->getPathPlannerProperties().timeLimitOnline);
    pathSegments[segmentIdPre] = preSegmentPath;

    // Recompute second segment, from first checkpoint to second checkpoint
    const Eigen::MatrixXd postSegmentPath = pathPlanner.planPath(lateCheckpoint, checkpoints[checkpointIdNextGate], configParser->getPathPlannerProperties().timeLimitOnline);
    pathSegments[segmentIdPost] = postSegmentPath;

    // Recompute trajectory
    const Eigen::VectorXd referenceState = sampleTraj(flightTime);
    Eigen::Vector3d refVel, refAcc;
    refVel << referenceState(1), referenceState(4), referenceState(7);
    refAcc << referenceState(2), referenceState(5), referenceState(8);

    std::vector<Eigen::MatrixXd> pathSegmentsSlice;
    for(int i = segmentIdPre; i < pathSegments.size(); i++)
    {
        pathSegmentsSlice.push_back(pathSegments[i]);
    }

    const double v_max = configParser->getTrajectoryGeneratorProperties().maxVelocity;
    const double a_max = configParser->getTrajectoryGeneratorProperties().maxAcceleration;
    const double samplingInterval = configParser->getTrajectoryGeneratorProperties().samplingInterval;
    poly_traj::generateTrajectory(pathSegmentsSlice, v_max, a_max, samplingInterval, flightTime, refVel, refAcc, plannedTraj);
}

Eigen::VectorXd OnlineTrajGenerator::sampleTraj(const double currentTime)
{
    // Trajectory points are provided as array of shape [[x, x_dot, x_ddot, y, y_dot, y_ddot, z, z_dot, z_ddot, time], 
    // this function samples trajectory points at a given time and writes them to the result matrix

    if (plannedTraj.rows() == 0)
    {
        throw std::runtime_error("No trajectory data available.");
    }
    const int lastColIdx = plannedTraj.cols() - 1;
    const Eigen::VectorXd &timeColumn = plannedTraj.col(lastColIdx);
    const Eigen::VectorXd timeDifferences = (timeColumn.array() - currentTime).abs();
    Eigen::Index minIndex;
    const double minTimeDifference = timeDifferences.minCoeff(&minIndex);

    return plannedTraj.row(minIndex);
}