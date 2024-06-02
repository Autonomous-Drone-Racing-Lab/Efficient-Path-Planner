#include "PathPlanner.h"

#include <ompl-1.6/ompl/base/StateSpace.h>
#include <ompl-1.6/ompl/base/SpaceInformation.h>
#include <ompl-1.6/ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/OptimizationObjective.h>
#include "World.h"
#include "ConfigParser.h"
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

namespace ob = ompl::base;
PathPlanner::PathPlanner(const Eigen::MatrixXd &nominalGatePositionAndType, const Eigen::MatrixXd &nominalObstaclePosition, std::shared_ptr<ConfigParser> configParser)
    : configParser(configParser)
{

    worldPtr = std::make_shared<World>(configParser);

    // Todo, potential memory leak
    space = ob::StateSpacePtr(new ob::RealVectorStateSpace(3));
    ob::RealVectorBounds bounds(3);
    for (int i = 0; i < 3; i++)
    {
        bounds.setLow(i, configParser->getWorldProperties().lowerBound(i));
        bounds.setHigh(i, configParser->getWorldProperties().upperBound(i));
    }
    space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

    // Todo, potential memory leak
    si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));

    // parse nomial gates
    for (int i = 0; i < nominalGatePositionAndType.rows(); i++)
    {
        std::cout << "Adding gate " << i << std::endl;
        Eigen::VectorXd gate = nominalGatePositionAndType.row(i);
        worldPtr->addGate(i, gate);
    }

    // parse nominal obstacles
    for (int i = 0; i < nominalObstaclePosition.rows(); i++)
    {
        Eigen::VectorXd obstacle = nominalObstaclePosition.row(i);
        worldPtr->addObstacle(i, obstacle);
    }

    si->setStateValidityChecker(std::make_shared<StateValidator>(si, worldPtr));
    si->setMotionValidator(std::make_shared<MotionValidator>(si, worldPtr));

    si->setup();
}

bool PathPlanner::planPath(const Eigen::Vector3d &start, const Eigen::Vector3d &goal, const double timeLimit, std::vector<Eigen::Vector3d> &resultPath)
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
    // ob::PlannerPtr planner(new ompl::geometric::RRTstar(si));
    std::shared_ptr<ompl::geometric::RRTstar> algo = std::make_shared<ompl::geometric::RRTstar>(si);
    algo->setRange(configParser->getPathPlannerProperties().range);
    ob::PlannerPtr planner(algo);

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

void PathPlanner::updateGatePos(const int gateId, const Eigen::Vector3d &newPose, const bool subtractHeight)
{
    worldPtr->updateGatePosition(gateId, newPose, subtractHeight);
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
        if(i < gateCenters.size() - 1){
            waypoints[i + 1].insert(waypoints[i + 1].begin(), gateCenters[i]);
        }
    }

    std::vector<Eigen::Vector3d> waypointsFlattened;
    for(const auto& segment : waypoints){
        std::vector<Eigen::Vector3d> prunedWaypoints = pruneWaypoints(segment);
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
    //std::cout << "Pruned " << waypintsBefore - waypointsAfter << " waypoints from " << waypintsBefore << " to " << waypointsAfter << std::endl;

    // adding last waypoint necessary as path must end at goal. No risk of duplication as last waypoint never added in loop from before
    prunedWaypoints.push_back(waypoints[waypoints.size() - 1]); // add last waypoint

    return prunedWaypoints;
}

std::vector<Eigen::Vector3d> PathPlanner::includeGates(const std::vector<std::vector<Eigen::Vector3d>> &waypoints) const
{
    std::vector<Eigen::Vector3d> waypointsFlattened;
    for (int i = 0; i < waypoints.size(); i++)
    {   
        // ignore last waypoint, directly skip to center
        for (int j = 0; j < waypoints[i].size() - 1; j++)
        {
            waypointsFlattened.push_back(waypoints[i][j]);
        }

        // add gate
        if (i < waypoints.size() - 1)
        {
            const Eigen::Vector3d gate = (waypoints[i][waypoints[i].size() - 1] + waypoints[i + 1][0]) / 2;
            waypointsFlattened.push_back(gate);
        }
    }

    return waypointsFlattened;
}

std::set<int> PathPlanner::checkTrajectoryValidityAndReturnCollisionIdx(const Eigen::MatrixXd &trajectoryPoints, const std::vector<Eigen::Vector3d> &prunedPath, int number_check_next_samples) const
{
    // set to make sure same index is only inserted once
    std::set<int> insertionIndice;

    int numberSamples = trajectoryPoints.rows();
    if (number_check_next_samples != -1)
    {
        numberSamples = std::min(numberSamples, number_check_next_samples);
    }

    for (int rowIdx = 0; rowIdx < numberSamples; rowIdx++)
    {
        const Eigen::Vector3d trajectoryPoint = trajectoryPoints.row(rowIdx);
        if (worldPtr->checkPointValidity(trajectoryPoint, 0.7, true))
        {
            continue;
        }

        // find the closest point in the pruned path
        double minDistance = std::numeric_limits<double>::max();
        int closestIdx = -1;
        for (int i = 0; i < prunedPath.size(); i++)
        {
            const double distance = (trajectoryPoint - prunedPath[i]).norm();
            if (distance < minDistance)
            {
                minDistance = distance;
                closestIdx = i;
            }
        }

        // check if the collision comes after or before point
        if (closestIdx == 0)
        {
            insertionIndice.insert(0);
            continue;
        }
        if (closestIdx == prunedPath.size() - 1)
        {
            insertionIndice.insert(prunedPath.size() - 1);
            continue;
        }

        const double preDistance = (trajectoryPoint - prunedPath[closestIdx - 1]).norm();
        const double postDistance = (trajectoryPoint - prunedPath[closestIdx + 1]).norm();
        // Todo this check is not ideal and probably not 100 percent correct
        // case a point is close to previous than to next, implying point is between previous and current point
        if (preDistance < postDistance)
        {
            insertionIndice.insert(closestIdx);
        }
        // case a point is close to next than to previous, implying point is between current and next point
        else
        {
            insertionIndice.insert(closestIdx + 1);
        }
    }

    return insertionIndice;
}