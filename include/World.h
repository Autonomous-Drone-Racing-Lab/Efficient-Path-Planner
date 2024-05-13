#pragma once

#include <Eigen/Dense>
#include "OBB.h"
#include "Object.h"
#include "ConfigParser.h"
#include "Types.h"
#include <ompl-1.6/ompl/base/StateValidityChecker.h>
#include <ompl-1.6/ompl/base/SpaceInformation.h>



class World {
public:
    World(const ConfigParser& configParser): configParser(configParser)
    {
        //inflateSizeGate = configParser.getObstacleInflateSize();
        //inflateSizeObstacle = configParser.getObstacleInflateSize();
    }


    void addGate(const int gateId, const Eigen::VectorXf& coordinates);
    void updateGatePosition(const int gateId, const Eigen::VectorXf& coordinates, const bool subtractGateHeight);
    void addObstacle(const int obstacleId, const Eigen::VectorXf& coordinates);

    bool checkPointValidity(const Eigen::Vector3f& point);
    bool checkRayValid(const Eigen::Vector3f& start, const Eigen::Vector3f& end);
    
    rtree index = rtree();
    float inflateSizeGate = 0.0;
    float inflateSizeObstacle= 0.0;
    std::map<std::string, OBB> obbs;

private:
    float minHeight = 0;
    float maxHeight = 2;
    ConfigParser configParser;


    void addObject(const Object& obj, const int id, const std::string &type, const float inflateSize);
    void removeObject(const int id, const std::string &type, const int noOfObbs, const float inflateSize);
    void addGatePrivateOperation(const int gateId, const Eigen::VectorXf& coordinates, const bool subtractGateHeight, const bool isUpdate);
};