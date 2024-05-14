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

Eigen::MatrixXd PathPlanner::planPath(const Eigen::Vector3d &start, const Eigen::Vector3d &goal, const double timeLimit)
{
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

        // Convert the path to an Eigen::MatrixXd
        Eigen::MatrixXd outputPath(path->getStateCount(), 3); // Dimensions: number of points x space dimensions (3D)
        for (std::size_t i = 0; i < path->getStateCount(); ++i)
        {
            const auto *state = path->getState(i)->as<ob::RealVectorStateSpace::StateType>();
            outputPath(i, 0) = state->values[0];
            outputPath(i, 1) = state->values[1];
            outputPath(i, 2) = state->values[2];
        }

        return outputPath;
    }
    else
    {
        // Return an empty matrix if planning failed
        return Eigen::MatrixXd(0, 3);
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