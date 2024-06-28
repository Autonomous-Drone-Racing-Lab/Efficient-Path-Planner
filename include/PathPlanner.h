#pragma once

#include "World.h"
#include "ConfigParserYAML.h"
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
    /**
     * @brief Constructs a PathPlanner object.
     *
     * This constructor initializes a PathPlanner object with the given parameters. 
     * 
     * The path planner encapsulates the whole world representation and is responsible for keeping it up to date.
     * Under the hood it uses the OMPL Planning library for path planning.
     *
     * @param nominalGatePositionAndType The matrix containing the nominal gate positions and types (initialization)
     * @param nominalObstaclePosition The matrix containing the nominal obstacle positions (initialization)
     * @param configParser A shared pointer to a ConfigParser object.
     */
    PathPlanner(const Eigen::MatrixXd& nominalGatePositionAndType, const Eigen::MatrixXd& nominalObstaclePosition, std::shared_ptr<ConfigParser> configParser);

    /**
     * Parses the given nominal gate positions and obstacle positions into the world representation.
     *
     * @param nominalGatePositionAndType The matrix containing the nominal gate positions and types.
     * @param nominalObstaclePosition The matrix containing the nominal obstacle positions.
     */
    void parseGatesAndObstacles(const Eigen::MatrixXd& nominalGatePositionAndType, const Eigen::MatrixXd& nominalObstaclePosition);

    /**
     * Plans a path from the given start point to the goal point within the specified time limit.
     *
     * @param start The starting point of the path.
     * @param goal The goal point of the path.
     * @param timeLimit The maximum time allowed for path planning.
     * @param resultPath The resulting path from start to goal.
     * @return True if a valid path is found within the time limit, false otherwise.
     */
    bool planPath(const Eigen::Vector3d& start, const Eigen::Vector3d& goal, const double timeLimit, std::vector<Eigen::Vector3d>& resultPath) const;
    
    
    /**
     * Updates the position of a gate identified by its ID with a new pose.
     * 
     * @param gateId The ID of the gate to update.
     * @param newPose The new pose of the gate.
     */
    void updateGatePos(const int gateId, const Eigen::Vector3d& newPose);
    
    /**
     * Checks the validity of a trajectory.
     * 
     * Typically called after updating the world to check if validy remains.
     * 
     * @param trajectory The trajectory to check.
     * @param minDistance The minimum distance to keep from obstacles.
     * @return True if the trajectory is valid, false otherwise.
    */
    bool checkTrajectoryValidity(const Eigen::MatrixXd& trajectory, const double minDistance) const;
    
    /**
     * Given a path described by segments does two things. First it incorporates the gate center i.e. the point between the
     * end of a segment and start of the next one into the path. Second it merges all segmens together to form one path
     * Within the merging operation a smoothing operation can be define via the config.yaml
     * 
     * @param waypoints The waypoints of the path.
     * @return The smoothed path.
    */
    std::vector<Eigen::Vector3d> includeGates2(std::vector<std::vector<Eigen::Vector3d>> waypoints) const;

    
    
    std::shared_ptr<World> worldPtr;    
    private:
    
    /**
     * Implement custom pruning logic to remove unnecessary waypoints from the path.
     * 
     * In short we take one waypoint then we traverse next waypoints as far as possible until connection has a collision.
     * To not miss gate centers this method should only be applied to segments
    */
    std::vector<Eigen::Vector3d> pruneWaypoints(const std::vector<Eigen::Vector3d> &waypoints) const;

    
    /**
     * Path smoothing action utilizing the ompl planning library. Uniecessary waypoints are removed and the path is
     * smoothed using spline interpolation. The validy of the path is guaranteed even after smoothing
     * 
     * @param waypoints The waypoints of the path.
     * @return The smoothed path.
    */
    std::vector<Eigen::Vector3d> omplPrunePathAndInterpolate(std::vector<Eigen::Vector3d> waypoints) const;
    
    /**
     * Create stopping criteria for the path planning. The stopping criteria is based on the straight-linde distance between 
     * the start and goal increased by the optimality threshold
     * 
     * @param si The space information for which the optimization objective is created.
     * @param start The start position of the path.
     * @param goal The goal position of the path.
     * @param optimalityThresholdPercentage The optimality threshold percentage.
    */
    ompl::base::OptimizationObjectivePtr getStraightLineObjective(const ompl::base::SpaceInformationPtr& si, const Eigen::Vector3d& start, const Eigen::Vector3d& goal, const double optimalityThresholdPercentage);
    
    ompl::base::StateSpacePtr space;
    ompl::base::SpaceInformationPtr si;
    ompl::base::SpaceInformationPtr si2;
    std::shared_ptr<ConfigParser> configParser;
};