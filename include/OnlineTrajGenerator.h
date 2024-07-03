#pragma once

#include "PathPlanner.h"
#include "ConfigParserYAML.h"
#include "Types.h"
#include <Eigen/Dense>
#include <memory>
#include "PathWriter.h"
#include "TrajInterpolation.h"

class OnlineTrajGenerator
{

public:
    /**
     * @brief Constructs an OnlineTrajGenerator object. This is the main runner of our code
     *
     * This constructor initializes an OnlineTrajGenerator object with the given parameters.
     *
     * @param start The start position of the trajectory.
     * @param goal The goal position of the trajectory.
     * @param nominalGatePositionAndType The matrix containing the nominal gate positions and types.
     * @param nominalObstaclePosition The matrix containing the nominal obstacle positions.
     * @param configPath The path to the configuration file.
     */
    OnlineTrajGenerator(const Eigen::Vector3d start, const Eigen::Vector3d goal, const Eigen::MatrixXd &nominalGatePositionAndType, const Eigen::MatrixXd &nominalObstaclePosition, const std::string &configPath);

    /**
     * Pre-computes the trajectory using the nominal gate positions provided at the beginning.
     * During runtime this trajectory is updated whenever new information is available
     *
     * @param takeoffTime The takeoff time of the drone, to align trajectory time with global time
     */
    void preComputeTraj(const double takeoffTime);
    
    
    /**
     * @brief Updates the position of a gate.
     *
     * This function updates the position of a gate identified by `gateId` with the new pose specified by `newPose`. It checks whetther the trajectory remains
     * valid after updating the gate position. If not it recomputes relevant segments in an asychronous fashiong
     *
     * @param gateId The ID of the gate to update.
     * @param newPose The new pose of the gate.
     * @param dronePos The current position of the drone.
     * @param nextGateWithinRange Indicates whether the next gate is within range.
     * @param flightTime The flight time (to align trajectory time with global time)
     * @return True if a recomputation of the trajectory is needed, false otherwise.
     */
    bool updateGatePos(const int gateId, const Eigen::VectorXd &newPose,  const Eigen::Vector3d& dronePos, const bool nextGateWithinRange, const double flightTime);


    /**
     * @brief Samples a trajectory at a given time.
     *
     * This function generates a trajectory sample at the specified current time.
     *
     * @param currentTime The current time at which to sample the trajectory.
     * @return The sampled trajectory as a `VectorXd` object. [x,x_dot, x_ddot, y, y_dot, y_ddot, z, z_dot, z_ddot, t]
     */
    Eigen::VectorXd sampleTraj(const double currentTime) const;
    

    /**
     * @brief Get the end time of the trajectory.
     *
     * This function returns the end time of the trajectory.
     *
     * @return The end time of the trajectory.
     */
    double getTrajEndTime() const;
    
    /**
     * @brief Retrieves the planned trajectory.
     *
     * This function returns the planned trajectory as a `MatrixXd` object.
     *
     * @return The planned trajectory as a `MatrixXd` object. Each row represents one sampe of the trajectory [x,x_dot, x_ddot, y, y_dot, y_ddot, z, z_dot, z_ddot, t]
     */
    Eigen::MatrixXd getPlannedTraj() const;


private:
    
    /**
     * @brief Recomputes the trjaectory for a given gate specified by id. 
     * 
     * This function recomputes two segments of trajectory. From the current drone position to the next gate and from the next gate to the one after that.
     * The function is called when a gate is updated and the trajectory is no longer valid.
     * The function takes into account the recomputation time of the trajectory and smoothly merges previous trajectory with recomputed one
     * 
     * @param gateId The ID of the gate.
     * @param newPose The new pose of the drone.
     * @param dronePos The position of the drone.
     * @param flightTime The flight time (for aligning trajectory time and global time)
     */

    void recomputeTraj(const int gateId, const Eigen::VectorXd& newPose, const Eigen::Vector3d& dronePos, const double flightTime);
    
    /**
     * @brief Retrieves the center and normal vectors of a gate.
     *
     * This function takes a gatePostAndType vector as input and returns the center and normal vectors of the gate.
     * This function is especially relevant for placing waypoints before and after each gate
     *
     * @param gatePostAndType The gatePostAndType vector containing information about the gate.
     * @param center[out] The center vector of the gate.
     * @param normal[out] The normal vector of the gate.
     * @return True if the center and normal vectors were successfully retrieved, false otherwise.
     */
    bool getGateCenterAndNormal(const Eigen::VectorXd &gatePostAndType, Eigen::Vector3d &center, Eigen::Vector3d &normal) const;
    
    
    /**
     * @brief Checks if a gate has been passed between two positions.
     *
     * This function takes in two positions, `pos1` and `pos2`, and an integer `gateId`.
     * It determines whether the gate with the given `gateId` has been passed between the two positions.
     * This is achieved by projecting both positions inot the local frame of the gate and checking whether the y-coordinate jumps in sign
     *
     * @param pos1 The first position.
     * @param pos2 The second position.
     * @param gateId The ID of the gate to check.
     * @return True if the gate has been passed, false otherwise.
     */
    bool checkGatePassed(const Eigen::Vector3d& pos1, const Eigen::Vector3d& pos2, const int gateId) const;


    bool trajectoryCurrentlyUpdating = false;
    std::shared_ptr<ConfigParser> configParser;
    PathPlanner pathPlanner;
    Eigen::MatrixXd nominalGatePositionAndType;
    Eigen::MatrixXd nominalObstaclePosition;
    std::vector<Eigen::Vector3d> checkpoints;
    std::set<int> gatesObservedWithinRange;
    std::map<int, float> outstandingGateUpdates;
    std::vector<std::vector<Eigen::Vector3d>> pathSegments;
    Eigen::MatrixXd plannedTraj;
    PathWriter pathWriter = PathWriter("path_segments");
    TrajInterpolation trajInterpolator;

   
    
   // void storePathSegmentsToFile(const std::string &path, const int startSegment);
};