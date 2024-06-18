#pragma once

#include <Eigen/Dense>
#include "OBB.h"
#include "Object.h"
#include "ConfigParser.h"
#include "Types.h"
#include <ompl-1.6/ompl/base/StateValidityChecker.h>
#include <ompl-1.6/ompl/base/SpaceInformation.h>
#include <memory>



class World {
public:
    World(std::shared_ptr<ConfigParser> configParser): configParser(configParser)
    {
        inflateSizeGate = configParser->getWorldProperties().inflateRadius.at("gate");
        inflateSizeObstacle = configParser->getWorldProperties().inflateRadius.at("obstacle");
    }


    void resetWorld(){
        index.clear();
        obbs = std::map<std::string, OBB>();
    }

    void addGate(const int gateId, const Eigen::VectorXd& coordinates);
    void updateGatePosition(const int gateId, const Eigen::VectorXd& coordinates);
    void addObstacle(const int obstacleId, const Eigen::VectorXd& coordinates);

    bool checkPointValidity(const Eigen::Vector3d& point, const bool canPassGate);
    bool checkPointValidity(const Eigen::Vector3d& point, const double minDistance) const;
    bool checkRayValid(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const bool canPassGate=false);

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


    void addObject(const Object& obj, const int id, const std::string &type, const double inflateSize);
    void removeObject(const int id, const std::string &type, const int noOfObbs, const double inflateSize);
    void addGatePrivateOperation(const int gateId, const Eigen::VectorXd& coordinates, const bool isUpdate);
};