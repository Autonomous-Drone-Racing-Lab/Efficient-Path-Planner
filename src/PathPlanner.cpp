#include "PathPlanner.h"

#include <ompl-1.6/ompl/base/StateSpace.h>
#include <ompl-1.6/ompl/base/SpaceInformation.h>
#include <ompl-1.6/ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/OptimizationObjective.h>
#include "World.h"
#include "ConfigParserYAML.h"
#include <Eigen/Dense>
#include "Types.h"
#include <memory>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/pRRT.h>
#include "MotionValidator.h"
#include "StateValidator.h"
#include "memory"
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTXstatic.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/fmt/FMT.h>

namespace ob = ompl::base;
PathPlanner::PathPlanner(const Eigen::MatrixXd &nominalGatePositionAndType, const Eigen::MatrixXd &nominalObstaclePosition, std::shared_ptr<ConfigParser> configParser)
    : configParser(configParser)
{

    worldPtr = std::make_shared<World>(configParser);

        // parse nomial gates
    parseGatesAndObstacles(nominalGatePositionAndType, nominalObstaclePosition);

    // Todo, potential memory leak
    space = ob::StateSpacePtr(new ob::RealVectorStateSpace(3));
    ob::RealVectorBounds bounds(3);
    for (int i = 0; i < 3; i++)
    {
        bounds.setLow(i, configParser->getWorldProperties().lowerBound(i));
        bounds.setHigh(i, configParser->getWorldProperties().upperBound(i));
    }
    space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

    // SI cannot pass gates (normal state space)
    const bool canPassGates = configParser->getPathPlannerProperties().canPassGate; 
    si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));
    si->setStateValidityChecker(std::make_shared<StateValidator>(si, worldPtr, canPassGates));
    si->setMotionValidator(std::make_shared<MotionValidator>(si, worldPtr, canPassGates));
    si->setup();

    // si2 can pass gates
    // si2 = ob::SpaceInformationPtr(new ob::SpaceInformation(space));
    // si2->setStateValidityChecker(std::make_shared<StateValidator>(si2, worldPtr, true));
    // si2->setMotionValidator(std::make_shared<MotionValidator>(si2, worldPtr, true));
    // si2->setup();
}

void PathPlanner::parseGatesAndObstacles(const Eigen::MatrixXd &nominalGatePositionAndType, const Eigen::MatrixXd &nominalObstaclePosition)
{
    
    worldPtr->resetWorld();
    // parse nomial gates
    for (int i = 0; i < nominalGatePositionAndType.rows(); i++)
    {
        Eigen::VectorXd gate = nominalGatePositionAndType.row(i);
        gate(2) = 0.0; // put all gate to cround
        worldPtr->addGate(i, gate);
    }

    // parse nominal obstacles
    for (int i = 0; i < nominalObstaclePosition.rows(); i++)
    {
        Eigen::VectorXd obstacle = nominalObstaclePosition.row(i);
        worldPtr->addObstacle(i, obstacle);
    }
}

bool PathPlanner::planPath(const Eigen::Vector3d &start, const Eigen::Vector3d &goal, const double timeLimit, std::vector<Eigen::Vector3d> &resultPath) const
{
    // assert result path is empty
    if (!resultPath.empty())
    {
        resultPath.clear();
        std::cerr << "Result path not empty, clearing it" << std::endl;
    }

    // set start and goal
    ob::ScopedState<> startState(space);
    startState->as<ob::RealVectorStateSpace::StateType>()->values[0] = start(0);
    startState->as<ob::RealVectorStateSpace::StateType>()->values[1] = start(1);
    startState->as<ob::RealVectorStateSpace::StateType>()->values[2] = start(2);

    ob::ScopedState<> goalState(space);
    goalState->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal(0);
    goalState->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal(1);
    goalState->as<ob::RealVectorStateSpace::StateType>()->values[2] = goal(2);

    // create problem
    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
    pdef->setStartAndGoalStates(startState, goalState);

    // create planner
    ob::PlannerPtr planner;
    if(configParser->getPathPlannerProperties().planner == "rrt"){
        std::shared_ptr<ompl::geometric::RRTstar> algo = std::make_shared<ompl::geometric::RRTstar>(si);
        algo->setRange(configParser->getPathPlannerProperties().range);
        planner = ob::PlannerPtr(algo);
    }else if(configParser->getPathPlannerProperties().planner == "fmt"){
        std::shared_ptr<ompl::geometric::FMT> algo = std::make_shared<ompl::geometric::FMT>(si);
        planner = ob::PlannerPtr(algo);
    }
    else {
        std::cerr << "Unknown planner" << std::endl;
        throw std::runtime_error("Unknown planner");
    }

    planner->setProblemDefinition(pdef);
    planner->setup();

    ob::PlannerStatus solved = planner->solve(timeLimit);

    // now convert back to Eigen::MatrixXd, i.e. [[x1, y1, z1], [x2, y2, z2], ...]
    if (solved)
    {
        // Get the path as geometric Path
        auto path = pdef->getSolutionPath()->as<ompl::geometric::PathGeometric>();
        ompl::geometric::PathSimplifier pathSimplifier(si);
        pathSimplifier.reduceVertices(*path);
        // pathSimplifier.smoothBSpline(*path, 5);

        // Convert the path to an Eigen::MatrixXd
        resultPath.resize(path->getStateCount()); // Dimensions: number of points x space dimensions (3D)
        for (std::size_t i = 0; i < path->getStateCount(); ++i)
        {
            const auto *state = path->getState(i)->as<ob::RealVectorStateSpace::StateType>();
            Eigen::Vector3d point{state->values[0], state->values[1], state->values[2]};
            resultPath[i] = point;
        }

        return true;
    }
    else
    {
        // Return an empty matrix if planning failed
        return false;
    }
}

ompl::base::OptimizationObjectivePtr PathPlanner::getStraightLineObjective(const ob::SpaceInformationPtr &si, const Eigen::Vector3d &start, const Eigen::Vector3d &goal, const double optimalityThresholdPercentage)
{
    const double straightLineDistance = (goal - start).norm();
    const double optimalityThreshold = straightLineDistance * optimalityThresholdPercentage;

    ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
    obj->setCostThreshold(ob::Cost(optimalityThreshold));
    return obj;
}

void PathPlanner::updateGatePos(const int gateId, const Eigen::Vector3d &newPose)
{
    worldPtr->updateGatePosition(gateId, newPose);
}

std::vector<Eigen::Vector3d> PathPlanner::includeGates2(std::vector<std::vector<Eigen::Vector3d>> waypoints) const
{
     // step 1: inlcude gate positions
    std::vector<Eigen::Vector3d> gateCenters;
    for (int firstSegmentIdx = 0; firstSegmentIdx < waypoints.size() - 1; firstSegmentIdx++)
    {
        const Eigen::Vector3d& firstSegmentEnd = waypoints[firstSegmentIdx][waypoints[firstSegmentIdx].size() - 1];
        const Eigen::Vector3d& secondSegmentStart = waypoints[firstSegmentIdx + 1][0];
        gateCenters.push_back((firstSegmentEnd + secondSegmentStart) / 2);
    }

    // step 2: include gate centers
    for(int i = 0; i < gateCenters.size(); i++){
        waypoints[i].push_back(gateCenters[i]);
        if(i < gateCenters.size()){
            waypoints[i + 1].insert(waypoints[i + 1].begin(), gateCenters[i]);
        }
    }

    std::vector<Eigen::Vector3d> waypointsFlattened;
    for(const auto& segment : waypoints){
        const std::string simplificationMethod = configParser->getPathPlannerProperties().pathSimplification;
        std::vector<Eigen::Vector3d> prunedWaypoints;
        if(simplificationMethod == "none"){
            prunedWaypoints = segment;
        }else if(simplificationMethod == "ompl"){
            prunedWaypoints = omplPrunePathAndInterpolate(segment);
        }else if(simplificationMethod == "custom"){
            prunedWaypoints = pruneWaypoints(segment);
        }
        else{
            std::cerr << "Unknown pruning method" << std::endl;
            throw std::runtime_error("Unknown pruning method");
        }

       for(const auto& waypoint : prunedWaypoints){
            // do not include duplicates
            if(waypointsFlattened.size() > 0 && (waypointsFlattened.back() - waypoint).norm() < 0.05){
                continue;
            }
            waypointsFlattened.push_back(waypoint);
        }
    }
    return waypointsFlattened;
}

std::vector<Eigen::Vector3d> PathPlanner::pruneWaypoints(const std::vector<Eigen::Vector3d> &waypoints) const
{
    if (waypoints.size() < 3)
    {
        return waypoints;
    }

    std::vector<Eigen::Vector3d> prunedWaypoints;
    prunedWaypoints.push_back(waypoints[0]); // add first waypoint

    int referenceWaypointIdx = 0;
    int waypointIdx = 2;
    while (waypointIdx < waypoints.size())
    {
        const Eigen::Vector3d referenceWaypoint = waypoints[referenceWaypointIdx];
        const Eigen::Vector3d currentWaypoint = waypoints[waypointIdx];
        if (!worldPtr->checkRayValid(referenceWaypoint, currentWaypoint, true))
        {
            const Eigen::Vector3d lastValidWaypoint = waypoints[waypointIdx - 1];
            prunedWaypoints.push_back(lastValidWaypoint);
            referenceWaypointIdx = waypointIdx - 1;
        }
        waypointIdx++;
    }

    int waypintsBefore = waypoints.size();
    int waypointsAfter = prunedWaypoints.size();
    std::cout << "Pruned " << waypintsBefore - waypointsAfter << " waypoints from " << waypintsBefore << " to " << waypointsAfter << std::endl;

    // adding last waypoint necessary as path must end at goal. No risk of duplication as last waypoint never added in loop from before
    prunedWaypoints.push_back(waypoints[waypoints.size() - 1]); // add last waypoint

    return prunedWaypoints;
}


bool PathPlanner::checkTrajectoryValidity(const Eigen::MatrixXd &trajectory, const double minDistance) const
{
    for (int i = 0; i < trajectory.rows(); i++)
    {   
        const Eigen::MatrixXd trajRow = trajectory.row(i);
        Eigen::Vector3d trajectoryPoint;
        trajectoryPoint << trajRow(0), trajRow(3), trajRow(6);
        if (!worldPtr->checkPointValidity(trajectoryPoint, minDistance))
        {
            return false;
        }
    }
    return true;
}


std::vector<Eigen::Vector3d> PathPlanner::omplPrunePathAndInterpolate(std::vector<Eigen::Vector3d> waypoints) const{

    ompl::geometric::PathGeometric path(si);

       for (const auto& point : waypoints) {
        // Todo check for memory leakage
        ob::State* state = space->allocState();
        state->as<ob::RealVectorStateSpace::StateType>()->values[0] = point(0);
        state->as<ob::RealVectorStateSpace::StateType>()->values[1] = point(1);
        state->as<ob::RealVectorStateSpace::StateType>()->values[2] = point(2);

        path.append(state);
    }

    ompl::geometric::PathSimplifier pathSimplifier(si);
    //pathSimplifier.reduceVertices(path);
    //pathSimplifier.shortcutPath(path);
    pathSimplifier.smoothBSpline(path);

    // convert back to eigen vector<Eigen::Vector3d>
        std::vector<Eigen::Vector3d> result;
    for (std::size_t i = 0; i < path.getStateCount(); ++i) {
        const auto* state = path.getState(i)->as<ob::RealVectorStateSpace::StateType>();
        Eigen::Vector3d point(state->values[0], state->values[1], state->values[2]);
        result.push_back(point);
        //delete state;
    }

    return result;
}