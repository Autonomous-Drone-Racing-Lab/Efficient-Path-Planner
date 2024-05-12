#include "ValidityChecker.h"

#include <Eigen/Dense>
#include "OBB.h"
#include "Object.h"
#include "ConfigParser.h"
#include "Types.h"
#include <ompl-1.6/ompl/base/StateValidityChecker.h>
#include <ompl-1.6/ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <boost/geometry/index/rtree.hpp>

namespace ob = ompl::base;
void ValidityChecker::addGatePrivateOperation(const int gateId, const Eigen::VectorXf &coordinates, const bool subtractGateHeight, const bool isUpdate)
{
    Eigen::Vector3f pos = coordinates.head(3);
    Eigen::Vector3f rot = coordinates.segment(3, 3);
    int type = coordinates(6);
    const std::vector<OBBDescription> &obbDescriptions = configParser.getGateGeometryByTypeId(type);
    const ObjectProperties &objectProperties = configParser.getObjectPropertiesByTypeId(type);

    if (subtractGateHeight)
    {
        pos(2) -= objectProperties.height;
    }

    // create object
    Object obj = Object::createFromDescription(pos, rot, obbDescriptions);
    // on update remove old OBBs
    if (isUpdate)
    {
        removeObject(gateId, "gate", obj.obbs.size(), inflateSizeGate);
    }
    // add new OBBs
    addObject(obj, gateId, "gate", inflateSizeGate);
}

void ValidityChecker::addGate(int gateId, const Eigen::VectorXf &coordinates)
{
    addGatePrivateOperation(gateId, coordinates, false, false);
}

void ValidityChecker::updateGatePosition(const int gateId, const Eigen::VectorXf &coordinates, const bool subtractGateHeight)
{
    addGatePrivateOperation(gateId, coordinates, subtractGateHeight, true);
}

void ValidityChecker::addObstacle(const int obstacleId, const Eigen::VectorXf &coordinates)
{
    Eigen::Vector3f pos = coordinates.head(3);
    Eigen::Vector3f rot = coordinates.segment(3, 3);
    const std::vector<OBBDescription> &obbDescriptions = configParser.getObstacleGeometry();

    Object obj = Object::createFromDescription(pos, rot, obbDescriptions);
    addObject(obj, obstacleId, "obstacle", inflateSizeObstacle);
}

void ValidityChecker::addObject(const Object &obj, const int id, const std::string &type, const float inflateSize)
{

    // create bounding boxes to insert in r-tree
    std::vector<box> aabbs = obj.getAABBs(inflateSize);
    for (int i = 0; i < aabbs.size(); i++)
    {
        std::string index_string = std::string() + type + "_" + std::to_string(id) + "_obb_" + std::to_string(i);
        obbs.insert(std::make_pair(index_string, obj.obbs[i]));
        index.insert(std::make_pair(aabbs[i], index_string));
    }
}

void ValidityChecker::removeObject(const int id, const std::string& type, const int noOfObbs, const float inflateSize)
{
    for (int i = 0; i < noOfObbs; i++)
    {
        std::string index_string = std::string() + type + "_" + std::to_string(id) + "_obb_" + std::to_string(i);
        OBB obbToDelete = obbs.at(index_string);
        index.remove(std::make_pair(obbToDelete.getAABB(inflateSize), index_string));
        obbs.erase(index_string);
    }
}

bool ValidityChecker::isValid(const ob::State *state) const
{
    const ob::RealVectorStateSpace::StateType *state3D =
        state->as<ob::RealVectorStateSpace::StateType>();
    Eigen::Vector3f pos(state3D->values[0], state3D->values[1], state3D->values[2]);
    // check where pos is in the r-tree
    std::vector<value> potentialHits;
    index.query(boost::geometry::index::contains(pos), std::back_inserter(potentialHits));

    for (const value &v : potentialHits)
    {
        OBB obb = obbs.at(v.second);
        // ToDo, handle proper inflateSize
        if(obb.checkCollisionWithPoint(pos, inflateSizeGate)){
            return false;
        }
    }

    return true;
}