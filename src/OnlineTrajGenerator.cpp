#include "OnlineTrajGenerator.h"
#include <iostream>
#include "poly_traj/trajectory_generator.h"
#include <thread>
#include <fstream>
#include <future>

OnlineTrajGenerator::OnlineTrajGenerator(const Eigen::Vector3d start, const Eigen::Vector3d goal, const Eigen::MatrixXd &nominalGatePositionAndType, const Eigen::MatrixXd &nominalObstaclePosition, const std::string &configPath)
    : configParser(std::make_shared<ConfigParser>(configPath)),
      pathPlanner(PathPlanner(nominalGatePositionAndType, nominalObstaclePosition, configParser)),
      nominalGatePositionAndType(nominalGatePositionAndType)
{

    // write nominal gate positions and types to file
    for(int i = 0; i < nominalGatePositionAndType.rows(); ++i){
        pathWriter.updateGatePos(i, nominalGatePositionAndType.row(i));
    }

    // write obstacle pos to file
    for(int i = 0; i<nominalObstaclePosition.rows(); i++){
        pathWriter.updateObstaclePos(i, nominalObstaclePosition.row(i));
    }

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


    const std::vector<Eigen::Vector3d> prunedPath = pathPlanner.includeGates2(pathSegments);
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
    
    // when we are not in range we also dont have new information
    if(!nextGateWithinRange){
        return false;
    }
    // ignore if gate was already seen close up and is now no longer
    if (gatesObservedWithinRange.find(gateId) != gatesObservedWithinRange.end())
    {
        return false;
    }
    //ignore if we canno construct path from where we are currently
    if(!pathPlanner.worldPtr->checkPointValidity(dronePos, false))
    {
        return false;
    }
    gatesObservedWithinRange.insert(gateId);

    // update nominal gate position
    nominalGatePositionAndType.row(gateId).head(6) = newPose;
    pathPlanner.updateGatePos(gateId, nominalGatePositionAndType.row(gateId), nextGateWithinRange); // if next gate within range, the view of the gate changes and we must subtract its height
    pathWriter.updateGatePos(gateId, nominalGatePositionAndType.row(gateId));

    // no longer necessary, as we do proper check whether trajectory collides later
    // const Eigen::Vector3d &newPos = newPose.head(3);
    // const Eigen::Vector3d &oldPos = nominalGatePositionAndType.row(gateId).head(3);
    // const Eigen::Vector3d &newRot = newPose.segment(3, 3);
    // const Eigen::Vector3d &oldRot = nominalGatePositionAndType.row(gateId).segment(3, 3);

    // const double posInsinificanceThreshold = configParser -> getPathPlannerProperties().posDivergenceRecalculate;
    // const double rotInsinificanceThreshold = configParser -> getPathPlannerProperties().rotDivergenceRecalculate;

    // const double deltaPos = (newPos.topRows(2) - oldPos.topRows(2)).norm();
    // const double deltaRot = (newRot - oldRot).norm();

    // if (deltaPos < posInsinificanceThreshold && deltaRot < rotInsinificanceThreshold)
    // {
    //     return false;
    // }

    // // find start idxr
    const int lastColIdx = plannedTraj.cols() - 1;
    const Eigen::VectorXd &timeColumn = plannedTraj.col(lastColIdx);
    const Eigen::VectorXd timeDifferences = (timeColumn.array() - flightTime).abs();
    Eigen::Index startIdx;
    const double minTimeDifference = timeDifferences.minCoeff(&startIdx);

    // find next gate
    const int checkpointIdNextGate = 2*gateId + 3 < checkpoints.size() ? 2*gateId+3: checkpoints.size() - 1;
    const Eigen::Vector3d& nextCheckpoint = checkpoints[checkpointIdNextGate];
    std::cout << "Checkpoint pose" << nextCheckpoint.transpose() << std::endl;
    Eigen::MatrixXd trajPos(plannedTraj.rows(), 3);
    trajPos << plannedTraj.col(0), plannedTraj.col(3), plannedTraj.col(6);
    const Eigen::VectorXd posDifferences = (trajPos.rowwise() - nextCheckpoint.transpose()).rowwise().norm();
    Eigen::Index endIdx;
    const double minDistance = posDifferences.minCoeff(&endIdx);
    const Eigen::MatrixXd& lookaheadTrajSegment = plannedTraj.block(startIdx, 0, endIdx - startIdx, plannedTraj.cols());

    std::cout << "Current drone pose" <<dronePos.transpose() << std::endl;
    std::cout<< "Lookadhead trajectory start" << lookaheadTrajSegment.topRows(1) << std::endl;
    std::cout << "Lookadhead trajectory end" << lookaheadTrajSegment.bottomRows(1) << std::endl;

    // check that next gate pos is reasonable well passed
    Eigen::Vector3d nextGatePos = newPose.head(3);
    std::cout << "Next gate pos " << nextGatePos.transpose() << std::endl;
    const Eigen::VectorXd posDifferencesNextGate = (trajPos.rowwise() - nextGatePos.transpose()).rowwise().norm();
    const double minDistanceGate = posDifferencesNextGate.minCoeff();
    bool trajectoryPassingNextGate;
    if(minDistanceGate > configParser->getPathPlannerProperties().minDistCheckTrajPassedGate){
        std::cout << "Current trajectory not passing next gate. Must be recomputed" << std::endl;
        trajectoryPassingNextGate = false;
    }else{
        std::cout << "Current trajectory is passing next gate" << std::endl;
        trajectoryPassingNextGate = true;
    }
    
    const double minDistanceCollision = configParser->getPathPlannerProperties().minDistCheckTrajCollision;
    std::cout << "Min distance collision" << minDistanceCollision << std::endl;
    bool trajectoryValid = pathPlanner.checkTrajectoryValidity(lookaheadTrajSegment, minDistanceCollision);
    if(trajectoryValid){
        std::cout << "Checked trajectory. It is not colliding, no need to recompute" << std::endl;
    }else{
        std::cout << "Checked trajectory. It is colliding, recomputing trajectory" << std::endl;
    }

    if(trajectoryValid && trajectoryPassingNextGate){
        return false;
    }
    
    if(trajectoryCurrentlyUpdating){
        std::cerr << "Call to update trajectory, while previous update is still going on";
        exit(1);
    }

    trajectoryCurrentlyUpdating = true;
    const bool recalculateOnline = configParser->getPathPlannerProperties().recalculateOnline;
    if(recalculateOnline){
        std::thread(&OnlineTrajGenerator::recomputeTraj, this, gateId, newPose, dronePos, nextGateWithinRange, flightTime).detach();
    }
    else{
        recomputeTraj(gateId, newPose, dronePos, nextGateWithinRange, flightTime);
    }
   
    return true;
}

void OnlineTrajGenerator::recomputeTraj(const int gateId, const Eigen::VectorXd& newPose, const Eigen::Vector3d& dronePos, const bool nextGateWithinRange, const double flightTime){
        
    // Generally relevant idxs
    const int segmentIdPre = gateId;
    const int segmentIdPost = gateId + 1;
    const int checkpointIdPre = 2 * gateId + 1;
    const int checkpointIdPost = 2 * gateId + 2;
    const int checkpointIdNextGate = 2 * gateId + 3;
    

    // update checkpoints
    Eigen::Vector3d center, normal;
    getGateCenterAndNormal(nominalGatePositionAndType.row(segmentIdPre), nextGateWithinRange, center, normal);
    double checkpointOffset = configParser->getPathPlannerProperties().checkpointGateOffset;
    const Eigen::VectorXd earlyCheckpoint = center - checkpointOffset * normal;
    const Eigen::VectorXd lateCheckpoint = center + checkpointOffset * normal;
    checkpoints[checkpointIdPre] = earlyCheckpoint;
    checkpoints[checkpointIdPost] = lateCheckpoint;

    // advance trajectory by delta t. Approximately accounting for time it takes us to recomputeTraj
    const double advanceTime = configParser->getPathPlannerProperties().timeLimitOnline + 0.05; // 50 ms added for general computation overhead
    const double advancedTime = flightTime + advanceTime;
    int startIdxAdvanced;
    // we could only include time from, now however, we include full trajectory to preetify printing
    for(startIdxAdvanced = 0; startIdxAdvanced < plannedTraj.rows(); startIdxAdvanced++){
        const Eigen::MatrixXd &state = plannedTraj.row(startIdxAdvanced);
        const int timeIdx = state.cols() - 1;

        if(state(timeIdx) > advancedTime){
            break;
        }
    }

    const Eigen::MatrixXd& trajectoryPartPre = plannedTraj.topRows(startIdxAdvanced);
    const Eigen::MatrixXd& advancedStartState = plannedTraj.row(startIdxAdvanced + 1);
    const Eigen::Vector3d dronePosAdvanced = Eigen::Vector3d(advancedStartState(0), advancedStartState(3), advancedStartState(6));
    const Eigen::Vector3d droneVelAdvanced = Eigen::Vector3d(advancedStartState(1), advancedStartState(4), advancedStartState(7));
    const Eigen::Vector3d droneAccAdvanced = Eigen::Vector3d(advancedStartState(2), advancedStartState(5), advancedStartState(8));

    const bool advancedStartStateValid = pathPlanner.worldPtr->checkPointValidity(dronePosAdvanced, configParser->getPathPlannerProperties().canPassGate);
    if(!advancedStartStateValid){
        std::cerr << "Advanced trajectory does not end at valid position. No recomputation and hope for best" << std::endl;
        trajectoryCurrentlyUpdating = false;
        return;
    }


    // Utilize two threads
    std::promise<std::vector<Eigen::Vector3d>> preSegmentPromise;
    std::promise<std::vector<Eigen::Vector3d>> postSegmentPromise;
    std::promise<bool> preSegmentResultPromise;
    std::promise<bool> postSegmentResultPromise;

    std::future<std::vector<Eigen::Vector3d>> preSegmentFuture = preSegmentPromise.get_future();
    std::future<std::vector<Eigen::Vector3d>> postSegmentFuture = postSegmentPromise.get_future();
    std::future<bool> preSegmentResultFuture = preSegmentResultPromise.get_future();
    std::future<bool> postSegmentResultFuture = postSegmentResultPromise.get_future();

    // first segment drone to gate
    std::thread preThread([&]() {
        std::vector<Eigen::Vector3d> preSegmentPath;
        bool preSegmentResult = pathPlanner.planPath(dronePosAdvanced, checkpoints[checkpointIdPre], configParser->getPathPlannerProperties().timeLimitOnline, preSegmentPath);
        preSegmentPromise.set_value(preSegmentPath);
        preSegmentResultPromise.set_value(preSegmentResult);
    });

    // second segment gate to next gate
    std::thread postThread([&]() {
        std::vector<Eigen::Vector3d> postSegmentPath;
        bool postSegmentResult = pathPlanner.planPath(checkpoints[checkpointIdPost], checkpoints[checkpointIdNextGate], configParser->getPathPlannerProperties().timeLimitOnline, postSegmentPath);
        postSegmentPromise.set_value(postSegmentPath);
        postSegmentResultPromise.set_value(postSegmentResult);
    });

    preThread.join();
    postThread.join();

    std::vector<Eigen::Vector3d> preSegmentPath = preSegmentFuture.get();
    std::vector<Eigen::Vector3d> postSegmentPath = postSegmentFuture.get();
    bool preSegmentResult = preSegmentResultFuture.get();
    bool postSegmentResult = postSegmentResultFuture.get();

    if (!preSegmentResult) {
        std::cerr << "Pre path not found. Exiting" << std::endl;
        exit(1);
    }
    pathSegments[segmentIdPre] = preSegmentPath;

    if (!postSegmentResult) {
        std::cerr << "Post segment path not found. Exiting" << std::endl;
        exit(1);
    }
    pathSegments[segmentIdPost] = postSegmentPath;

    // Recompute trajectory for second part
    std::vector<std::vector<Eigen::Vector3d>> pathSegmentsSlice;
    for (int i = segmentIdPre; i < pathSegments.size(); i++)
    {
        pathSegmentsSlice.push_back(pathSegments[i]);
    }

    std::vector<Eigen::Vector3d> filledWaypoints = pathPlanner.includeGates2(pathSegmentsSlice);
    pathWriter.writePath(filledWaypoints);

    const double v_max = configParser->getTrajectoryGeneratorProperties().maxVelocity;
    const double a_max = configParser->getTrajectoryGeneratorProperties().maxAcceleration;
    const double samplingInterval = configParser->getTrajectoryGeneratorProperties().samplingInterval;
    Eigen::MatrixXd trajPartPost;
    poly_traj::generateTrajectory(filledWaypoints, v_max, a_max, samplingInterval, flightTime, droneVelAdvanced, droneAccAdvanced, trajPartPost);
    
    // merge trajectories together, pre computed and current
    Eigen::MatrixXd newTraj(trajPartPost.rows() + trajectoryPartPre.rows(), trajPartPost.cols());
    newTraj << trajectoryPartPre, trajPartPost;
    plannedTraj = newTraj;
    trajectoryCurrentlyUpdating = false;
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
