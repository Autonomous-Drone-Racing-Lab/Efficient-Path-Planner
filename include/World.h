#pragma once

#include <Eigen/Dense>
#include "OBB.h"
#include "Object.h"
#include "ConfigParserYAML.h"
#include "Types.h"
#include <ompl-1.6/ompl/base/StateValidityChecker.h>
#include <ompl-1.6/ompl/base/SpaceInformation.h>
#include <memory>


/**
 * @class World
 * Class represnting the world. It keeps track of all objects and their positions in the world and provides utility to query for collisions.
*/
class World {


public:
    World(std::shared_ptr<ConfigParser> configParser): configParser(configParser)
    {
        inflateSizeGate = configParser->getWorldProperties().inflateRadius.at("gate");
        inflateSizeObstacle = configParser->getWorldProperties().inflateRadius.at("obstacle");
    }

    /**
     * Reset world, removing all objects and gates.
    */
    void resetWorld(){
        index.clear();
        obbs = std::map<std::string, OBB>();
    }

    /**
     * Add a gate to the world.
     * 
     * @param gateId The ID of the gate to add.
     * @param coordinates The coordinates of the gate.
    */
    void addGate(const int gateId, const Eigen::VectorXd& coordinates);

    /**
     * Update position of existing gate in world
     * 
     * @param gateId The ID of the gate to update.
     * @param coordinates The new coordinates of the gate.
    */
    void updateGatePosition(const int gateId, const Eigen::VectorXd& coordinates);

    /**
     * Add an obstacle to the world.
     * 
     * @param obstacleId The ID of the obstacle to add.
     * @param coordinates The coordinates of the obstacle.
    */
    void addObstacle(const int obstacleId, const Eigen::VectorXd& coordinates);

    /**
     * Check validity of a point in the world.
     * 
     * We first check fast for possible collision susing the r-tree and converting all OOBs to AABBs.
     * All posible collisions are checked again to filter false positives
     * 
     * @param point The point to check.
     * @param canPassGate Flag indicating whether the drone can pass thorugh a gate (has effect for points within the gate)
    */
    bool checkPointValidity(const Eigen::Vector3d& point, const bool canPassGate);

    /**
     * Check validity of a point in the world.
     * 
     * We first check fast for possible collisions using the r-tree and converting all OOBs to AABBs.
     * All posible collisions are checked again to filter false positives
     * 
     * @param point The ppint in the world
     * @param minDistance The minimum distance to the point. This is useful for overwrtiting the default inflate size
    */
    bool checkPointValidity(const Eigen::Vector3d& point, const double minDistance) const;

    /**
     * Check validity of a ray in the world.
     * 
     * Check for potential collisions of ray transforming OOBs into AABBs.
     * All possible collisions are checked again to filter false positives
     * 
     * @param start The start of the ray.
     * @param end The end of the ray.
     * @param canPassGate Flag indicating whether the drone can pass thorugh a gate (has effect for points within the gate)
     * 
    */
    bool checkRayValid(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const bool canPassGate=false);

    /**
     * Printing function for debug purpose
    */
    void printObbs() const{
        for (const auto& obb : obbs){
            std::cout << "Obb: " << obb.first << std::endl;
            std::cout << "Obb center: " << obb.second.center.transpose() << std::endl;
        }
    }
    

private:
    std::shared_ptr<ConfigParser> configParser;

    rtree index = rtree();
    double inflateSizeGate;
    double inflateSizeObstacle;
    std::map<std::string, OBB> obbs;

    /**
     * Utility function utilized by add gate and add obstacle to add object to the world
    */
    void addObject(const Object& obj, const int id, const std::string &type, const double inflateSize);
    /**
     * RUtility function to remove object from R-Tree represneting world in easy manner
    */
    void removeObject(const int id, const std::string &type, const int noOfObbs, const double inflateSize);

    /**
     * Utility function utlizied by add and update gate position to add a gate to a world, and update position if necessary
    */
    void addGatePrivateOperation(const int gateId, const Eigen::VectorXd& coordinates, const bool isUpdate);
};