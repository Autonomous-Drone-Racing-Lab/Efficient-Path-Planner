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

bool PathPlanner::planPath(const Eigen::Vector3d &start, const Eigen::Vector3d &goal, const double timeLimit, std::vector<Eigen::Vector3d>& resultPath)
{
    // assert result path is empty
    if(!resultPath.empty()){
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
     ob::PlannerPtr planner(new ompl::geometric::InformedRRTstar(si));
   
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
        //pathSimplifier.smoothBSpline(*path, 3);

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

std::vector<Eigen::Vector3d> PathPlanner::pruneWaypoints(const std::vector<std::vector<Eigen::Vector3d>> &waypoints){
    // prune waypoints that are not necessary, because it could be skipped by connecting to the next waypoint in a straight line
    // Todo hanle edge cases, e.g. length is one, ...
    
    std::vector<Eigen::Vector3d> waypointsFlattened;
    for(const auto &segment : waypoints){
        for(const auto &waypoint : segment){
            waypointsFlattened.push_back(waypoint);
        }
    }

    if(waypointsFlattened.size() < 3){
        return waypointsFlattened;
    }

    std::vector<Eigen::Vector3d> prunedWaypoints;
    prunedWaypoints.push_back(waypointsFlattened[0]); // add first waypoint
    std::cout << "Adding first waypoint to pruned waypoints" << waypointsFlattened[0].transpose() << std::endl;
    
    int referenceWaypointIdx = 0;
    int waypointIdx = 1;
    while(waypointIdx < waypointsFlattened.size()){
        const Eigen::Vector3d referenceWaypoint = waypointsFlattened[referenceWaypointIdx];
        const Eigen::Vector3d currentWaypoint = waypointsFlattened[waypointIdx];
        std::cout << "checkin ray from index " << referenceWaypointIdx << " to " << waypointIdx << " i.e., " << referenceWaypoint.transpose() << " to " << currentWaypoint.transpose() << std::endl;
        if(!worldPtr->checkRayValid(referenceWaypoint, currentWaypoint, true)){
           std::cout << "Ray not valid, adding last valid waypoin to pruned waypoints" << std::endl;
            const Eigen::Vector3d lastValidWaypoint = waypointsFlattened[waypointIdx - 1];
            prunedWaypoints.push_back(lastValidWaypoint);
            referenceWaypointIdx = waypointIdx - 1;
        }
        waypointIdx++;
    }

    int waypintsBefore = waypointsFlattened.size();
    int waypointsAfter = prunedWaypoints.size();
    std::cout << "Pruned "<< waypintsBefore - waypointsAfter <<" waypoints from " << waypintsBefore << " to " << waypointsAfter << std::endl;

    // adding last waypoint necessary as path must end at goal. No risk of duplication as last waypoint never added in loop from before
    prunedWaypoints.push_back(waypointsFlattened[waypointsFlattened.size() - 1]); // add last waypoint

    return prunedWaypoints;
}