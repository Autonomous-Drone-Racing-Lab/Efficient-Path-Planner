#include "OnlineTrajGenerator.h"
#include <iostream>
#include "poly_traj/trajectory_generator.h"
#include <fstream>

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
        getGateCenterAndNormal(gate, false, center, normal);
        const double checkpointOffset = configParser->getPathPlannerProperties().checkpointGateOffset;
        const Eigen::VectorXd earlyCheckpoint = center - checkpointOffset * normal;
        const Eigen::VectorXd lateCheckpoint = center + checkpointOffset * normal;

        checkpoints.push_back(earlyCheckpoint);
        checkpoints.push_back(lateCheckpoint);
    }
    checkpoints.push_back(goal);
}

bool OnlineTrajGenerator::getGateCenterAndNormal(const Eigen::VectorXd &gatePostAndType,  const bool subtractHeight, Eigen::Vector3d &center, Eigen::Vector3d &normal)
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

    if (subtractHeight)
    {
        center = gatePosition;
    }
    else
    {
        const double gateHeight = configParser->getObjectPropertiesByTypeId(gateType).height;
        center = gatePosition + Eigen::Vector3d(0, 0, gateHeight);
    }

    normal << -sin(gateRotation(2)), cos(gateRotation(2)), 0;
    normal.normalize();

    return true;
}

void OnlineTrajGenerator::preComputeTraj(const double takeoffTime)
{
    const double timeLimit = configParser->getPathPlannerProperties().timeLimitOffline;
    for (int i = 0; i < checkpoints.size() - 1; i += 2)
    {
        const Eigen::Vector3d &start = checkpoints[i];
        const Eigen::Vector3d &goal = checkpoints[i + 1];
        std::vector<Eigen::Vector3d> path;
        const bool pathFound = pathPlanner.planPath(start, goal, timeLimit, path);
        if (!pathFound)
        {
            throw std::runtime_error("Path not found");
        }

        pathSegments.push_back(path);
    }

    // print path segments
    // for(int i = 0; i < pathSegments.size(); i++){
    //     std::cout << "Path segment " << i << std::endl;
    //     for(const auto &waypoint : pathSegments[i]){
    //         std::cout << waypoint.transpose() << std::endl;
    //     }
    // }


    const std::vector<Eigen::Vector3d> prunedPath = pathPlanner.includeGates(pathSegments);
    pathWriter.writePath(prunedPath);

    const double v_max = configParser->getTrajectoryGeneratorProperties().maxVelocity;
    const double a_max = configParser->getTrajectoryGeneratorProperties().maxAcceleration;
    const double samplingInterval = configParser->getTrajectoryGeneratorProperties().samplingInterval;

    const Eigen::Vector3d initialVel = Eigen::Vector3d::Zero();
    const Eigen::Vector3d initialAcc = Eigen::Vector3d::Zero();
    poly_traj::generateTrajectory(prunedPath, v_max, a_max, samplingInterval, takeoffTime, initialVel, initialAcc, plannedTraj);
}

bool OnlineTrajGenerator::updateGatePos(const int gateId, const Eigen::VectorXd &newPose, const Eigen::Vector3d &dronePos, const bool nextGateWithinRange, const double flightTime)
{   
    bool gateHasOutstandingUpdated = false;
    if (outstandingGateUpdates.find(gateId) != outstandingGateUpdates.end())
    {
        const float timeSinceUpdate = flightTime - outstandingGateUpdates[gateId];
        if (timeSinceUpdate > 0.2)
        {
            gateHasOutstandingUpdated = true;
        }
    }

    // data relevant for all
    const int segmentIdPre = gateId;
    const int segmentIdPost = gateId + 1;
    const int checkpointIdPre = 2 * gateId + 1;
    const int checkpointIdPost = 2 * gateId + 2;
    const int checkpointIdNextGate = 2 * gateId + 3;

   // if(!gateHasOutstandingUpdated){
    if (nextGateWithinRange)
    {
        gatesObservedWithinRange.insert(gateId);
    }

    // ignore if gate was already seen close up and is now no longer
    if (gatesObservedWithinRange.find(gateId) != gatesObservedWithinRange.end() && !nextGateWithinRange)
    {
        return false;
    }

    const Eigen::Vector3d &newPos = newPose.head(3);
    const Eigen::Vector3d &oldPos = nominalGatePositionAndType.row(gateId).head(3);
    const Eigen::Vector3d &newRot = newPose.segment(3, 3);
    const Eigen::Vector3d &oldRot = nominalGatePositionAndType.row(gateId).segment(3, 3);

    const double posInsinificanceThreshold = 0.05;
    const double rotInsinificanceThreshold = 0.05;
    if ((newPos - oldPos).norm() < posInsinificanceThreshold && (newRot - oldRot).norm() < rotInsinificanceThreshold)
    {
        return false;
    }

    // update nominal gate position
    nominalGatePositionAndType.row(gateId).head(6) = newPose;
    std::cout << "updaging gate pos, subtract height: " << nextGateWithinRange << std::endl;
    pathPlanner.updateGatePos(gateId, nominalGatePositionAndType.row(gateId), nextGateWithinRange); // if next gate within range, the view of the gate changes and we must subtract its height

    // update checkpoints
    Eigen::Vector3d center, normal;
    getGateCenterAndNormal(nominalGatePositionAndType.row(segmentIdPre), nextGateWithinRange, center, normal);
    double checkpointOffset = configParser->getPathPlannerProperties().checkpointGateOffset;
    const Eigen::VectorXd earlyCheckpoint = center - checkpointOffset * normal;
    const Eigen::VectorXd lateCheckpoint = center + checkpointOffset * normal;
    checkpoints[checkpointIdPre] = earlyCheckpoint;
    checkpoints[checkpointIdPost] = lateCheckpoint;
    //}
    
    // Recompute first segment, from drone to first checkpoint
    std::vector<Eigen::Vector3d> preSegmentPath;
    std::cout << "Recomputing path from drone to firsty checkpoint, i.e. from" << dronePos.transpose() << " to " << checkpoints[checkpointIdPre].transpose() << std::endl;
    const bool preSegmentResult = pathPlanner.planPath(dronePos, checkpoints[checkpointIdPre], configParser->getPathPlannerProperties().timeLimitOnline, preSegmentPath);
    if (!preSegmentResult)
    {
        std::cout << "Path not found, trying again later" << std::endl;
        outstandingGateUpdates[gateId] = flightTime;
        return false;
    }
    else
    {
        outstandingGateUpdates.erase(gateId);
    }
    pathSegments[segmentIdPre] = preSegmentPath;

    // Recompute second segment, from post checkpoint to next gate
    std::vector<Eigen::Vector3d> postSegmentPath;
    std::cout << "Recomputing path from post checkpoint to next gate, i.e. from" << checkpoints[checkpointIdPost].transpose() << " to " << checkpoints[checkpointIdNextGate].transpose() << std::endl;
    const bool postSegmentResult = pathPlanner.planPath(checkpoints[checkpointIdPost], checkpoints[checkpointIdNextGate], configParser->getPathPlannerProperties().timeLimitOnline, postSegmentPath);
    if (!postSegmentResult)
    {
        std::cout << "Post segment path not found" << std::endl;
    }
    else
    {
        pathSegments[segmentIdPost] = postSegmentPath;
    }

    // Recompute trajectory
    const Eigen::VectorXd referenceState = sampleTraj(flightTime);
    Eigen::Vector3d refVel, refAcc;
    refVel << referenceState(1), referenceState(4), referenceState(7);
    refAcc << referenceState(2), referenceState(5), referenceState(8);

    std::vector<std::vector<Eigen::Vector3d>> pathSegmentsSlice;
    for (int i = segmentIdPre; i < pathSegments.size(); i++)
    {
        pathSegmentsSlice.push_back(pathSegments[i]);
    }

    std::vector<Eigen::Vector3d> prunedWaypoints = pathPlanner.includeGates(pathSegmentsSlice);
    pathWriter.writePath(prunedWaypoints);

    const double v_max = configParser->getTrajectoryGeneratorProperties().maxVelocity;
    const double a_max = configParser->getTrajectoryGeneratorProperties().maxAcceleration;
    const double samplingInterval = configParser->getTrajectoryGeneratorProperties().samplingInterval;
    Eigen::MatrixXd traj;
    poly_traj::generateTrajectory(prunedWaypoints, v_max, a_max, samplingInterval, flightTime, refVel, refAcc, traj);
    std::set<int> insertionIndice;
    // do{
    //     int no_check_next_samples = 1.2 / configParser->getTrajectoryGeneratorProperties().samplingInterval;
    //     insertionIndice = pathPlanner.checkTrajectoryValidityAndReturnCollisionIdx(traj, prunedWaypoints, no_check_next_samples);

    //     std::cout << "Trajectory invalid, inserting " << insertionIndice.size() << " waypoints" << std::endl;

    //     for(const auto &idx: insertionIndice){
    //         const Eigen::Vector3d& preWaypoint = prunedWaypoints[idx];
    //         const Eigen::Vector3d& postWaypoint = prunedWaypoints[idx + 1];
    //         const Eigen::Vector3d insertedWaypoint = (preWaypoint + postWaypoint) / 2;
            
    //         prunedWaypoints.insert(prunedWaypoints.begin() + idx + 1, insertedWaypoint);
    //     }

    //     // prune waypoints to close to one another
    //     std::vector<Eigen::Vector3d> waypointsDuplicatesRemoved;
    //     waypointsDuplicatesRemoved.push_back(prunedWaypoints[0]);

    //     Eigen::Vector3d lastWaypoint = prunedWaypoints[0];
    //     for(int i = 1; i < prunedWaypoints.size(); i++){
    //         const Eigen::Vector3d& currentWaypoint = prunedWaypoints[i];
    //         if((currentWaypoint - lastWaypoint).norm() > 0.01){
    //             waypointsDuplicatesRemoved.push_back(currentWaypoint);
    //             lastWaypoint = currentWaypoint;
    //         }
    //     }

    //     prunedWaypoints = waypointsDuplicatesRemoved;
    //     pathWriter.writePath(prunedWaypoints);

    //     poly_traj::generateTrajectory(prunedWaypoints, v_max, a_max, samplingInterval, flightTime, refVel, refAcc, traj);

    // }while(insertionIndice.size() > 0);

    plannedTraj = traj;

    return true;
}

Eigen::VectorXd OnlineTrajGenerator::sampleTraj(const double currentTime) const
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

double OnlineTrajGenerator::getTrajEndTime() const
{
    if (plannedTraj.rows() == 0)
    {
        throw std::runtime_error("No trajectory data available.");
    }

    const int timeColIdx = plannedTraj.cols() - 1;
    return plannedTraj(plannedTraj.rows() - 1, timeColIdx);
}

Eigen::MatrixXd OnlineTrajGenerator::getPlannedTraj() const
{
    if (plannedTraj.rows() == 0)
    {
        throw std::runtime_error("No trajectory data available.");
    }

    return plannedTraj;
}

// void OnlineTrajGenerator::storePathSegmentsToFile(const std::string &filename, const int startSegment)
// {
//  std::ofstream file(filename);
//      if (!file.is_open())
//     {
//         std::cerr << "Failed to open file for writing: " << filename << std::endl;
//         return;
//     }

//     for (int i = startSegment; i < pathSegments.size(); i++)
//     {
//         const std::vector<Eigen::Vector3d> &segment = pathSegments[i];
//         for (const auto& waypoint: segment)
//         {
//             for (int k = 0; k < 3; k++)
//             {
//                 file << waypoint(k) << " ";
//             }
//             file << std::endl;
//         }
//     }
//     file.close();
//     std::cout << "Stored path segments to " << filename << std::endl;
// }
