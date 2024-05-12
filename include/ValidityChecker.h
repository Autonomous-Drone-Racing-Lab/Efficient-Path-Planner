#pragma once

#include <Eigen/Dense>
#include "OBB.h"
#include "Object.h"
#include "ConfigParser.h"
#include "Types.h"
#include <ompl-1.6/ompl/base/StateValidityChecker.h>
#include <ompl-1.6/ompl/base/SpaceInformation.h>



class ValidityChecker: public ompl::base::StateValidityChecker
{
public:
    ValidityChecker(const ompl::base::SpaceInformationPtr &si, const ConfigParser& configParser): ompl::base::StateValidityChecker(si), configParser(configParser)
    {
        //inflateSizeGate = configParser.getObstacleInflateSize();
        //inflateSizeObstacle = configParser.getObstacleInflateSize();
    }

    virtual bool isValid(const ompl::base::State *state) const override;

    void addGate(const int gateId, const Eigen::VectorXf& coordinates);
    void updateGatePosition(const int gateId, const Eigen::VectorXf& coordinates, const bool subtractGateHeight);
    void addObstacle(const int obstacleId, const Eigen::VectorXf& coordinates);


private:
    std::map<std::string, OBB> obbs;
    rtree index = rtree();
    float minHeight = 0;
    float maxHeight = 2;
    ConfigParser configParser;
    float inflateSizeGate = 0.3;
    float inflateSizeObstacle= 1;

    void addObject(const Object& obj, const int id, const std::string &type, const float inflateSize);
    void removeObject(const int id, const std::string &type, const int noOfObbs, const float inflateSize);
    void addGatePrivateOperation(const int gateId, const Eigen::VectorXf& coordinates, const bool subtractGateHeight, const bool isUpdate);
};