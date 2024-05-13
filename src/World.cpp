#include "World.h"
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
void World::addGatePrivateOperation(const int gateId, const Eigen::VectorXf &coordinates, const bool subtractGateHeight, const bool isUpdate)
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

void World::addGate(int gateId, const Eigen::VectorXf &coordinates)
{
    addGatePrivateOperation(gateId, coordinates, false, false);
}

void World::updateGatePosition(const int gateId, const Eigen::VectorXf &coordinates, const bool subtractGateHeight)
{
    addGatePrivateOperation(gateId, coordinates, subtractGateHeight, true);
}

void World::addObstacle(const int obstacleId, const Eigen::VectorXf &coordinates)
{
    Eigen::Vector3f pos = coordinates.head(3);
    Eigen::Vector3f rot = coordinates.segment(3, 3);
    const std::vector<OBBDescription> &obbDescriptions = configParser.getObstacleGeometry();

    Object obj = Object::createFromDescription(pos, rot, obbDescriptions);
    addObject(obj, obstacleId, "obstacle", inflateSizeObstacle);
}

void World::addObject(const Object &obj, const int id, const std::string &type, const float inflateSize)
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

void World::removeObject(const int id, const std::string &type, const int noOfObbs, const float inflateSize)
{
    for (int i = 0; i < noOfObbs; i++)
    {
        std::string index_string = std::string() + type + "_" + std::to_string(id) + "_obb_" + std::to_string(i);
        OBB obbToDelete = obbs.at(index_string);
        index.remove(std::make_pair(obbToDelete.getAABB(inflateSize), index_string));
        obbs.erase(index_string);
    }
}

bool World::checkPointValidity(const Eigen::Vector3f &point)
{
    std::vector<value> potentialHits;
    index.query(boost::geometry::index::contains(point), std::back_inserter(potentialHits));

    for (const value &v : potentialHits)
    {
        OBB obb = obbs.at(v.second);

        bool isGate = v.second.find("gate") != std::string::npos;
        // ToDo, handle proper inflateSize
        if (obb.checkCollisionWithPoint(point, isGate ? inflateSizeGate : inflateSizeObstacle))
        {
            return false;
        }
    }

    return true;
}

bool World::checkRayValid(const Eigen::Vector3f &start, const Eigen::Vector3f &end)
{
    // create ray bounding box
    // ToDo how can we make this more efficient
    Eigen::Matrix<float, 3, 2> ray;
    ray.col(0) = start;
    ray.col(1) = end;
    Eigen::Vector3f rayMin = ray.rowwise().minCoeff();
    Eigen::Vector3f rayMax = ray.rowwise().maxCoeff();
    box rayBox(rayMin, rayMax);

    // find potential hits
    std::vector<value> potentialHits;
    index.query(boost::geometry::index::intersects(rayBox), std::back_inserter(potentialHits));

    for (const value &v : potentialHits)
    {
        OBB obb = obbs.at(v.second);
        bool isGate = v.second.find("gate") != std::string::npos;
        // ToDo, handle proper inflateSize
        if (obb.checkCollisionWithRay(start, end, isGate ? inflateSizeGate : inflateSizeObstacle))
        {
            return false;
        }
    }
    return true;
}