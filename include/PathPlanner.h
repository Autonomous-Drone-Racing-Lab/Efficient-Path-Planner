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
    PathPlanner(const Eigen::MatrixXf& nominalGatePositionAndType, const Eigen::MatrixXf& nominalObstaclePosition, const Eigen::Vector3f& lowerBound, const Eigen::Vector3f& upperBound, const std::string& configPath);


    Eigen::MatrixXf planPath(const Eigen::Vector3f& start, const Eigen::Vector3f& goal, const float timeLimit);


    private:
    ompl::base::StateSpacePtr space;
    ompl::base::SpaceInformationPtr si;
    std::shared_ptr<World> worldPtr;
    ConfigParser configParser;

    ompl::base::OptimizationObjectivePtr getStraightLineObjective(const ompl::base::SpaceInformationPtr& si, const Eigen::Vector3f& start, const Eigen::Vector3f& goal, const float optimalityThresholdPercentage);
};