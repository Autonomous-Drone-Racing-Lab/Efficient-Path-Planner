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
void World::addGatePrivateOperation(const int gateId, const Eigen::VectorXd &coordinates, const bool isUpdate)
{   
    std::cout << "Add gate private operation" << std::endl;
    Eigen::Vector3d pos = coordinates.head(3);
    Eigen::Vector3d rot = coordinates.segment(3, 3);
    int type = coordinates(6);
    const std::vector<OBBDescription> &obbDescriptions = configParser->getGateGeometryByTypeId(type);
    const ObjectProperties &objectProperties = configParser->getObjectPropertiesByTypeId(type);
    pos(2) = 0.0;
    // create object
    Object obj = Object::createFromDescription(pos, rot, obbDescriptions);
    // on update remove old OBBs
    if (isUpdate)
    {   
        // no of obbs in tree
        const int no_obj_before_delete = index.size();
        removeObject(gateId, "gate", obj.obbs.size(), inflateSizeGate);
        const int no_obj_after_delete = index.size();
        std::cout << "Delted " << no_obj_after_delete - no_obj_before_delete << " obbs from " << no_obj_before_delete << " to " << no_obj_after_delete << std::endl;

    }
    // add new OBBs
    addObject(obj, gateId, "gate", inflateSizeGate);
}

void World::addGate(int gateId, const Eigen::VectorXd &coordinates)
{   
    std::cout  << "Adding gate " << gateId << std::endl;
    addGatePrivateOperation(gateId, coordinates, false);
}

void World::updateGatePosition(const int gateId, const Eigen::VectorXd &coordinates)
{
    addGatePrivateOperation(gateId, coordinates, true);
}

void World::addObstacle(const int obstacleId, const Eigen::VectorXd &coordinates)
{
    Eigen::Vector3d pos = coordinates.head(3);
    Eigen::Vector3d rot = coordinates.segment(3, 3);
    const std::vector<OBBDescription> &obbDescriptions = configParser->getObstacleGeometry();

    Object obj = Object::createFromDescription(pos, rot, obbDescriptions);
    addObject(obj, obstacleId, "obstacle", inflateSizeObstacle);
}

void World::addObject(const Object &obj, const int id, const std::string &type, const double inflateSize)
{
    std::cout << "Adding object " << id << std::endl;
    // create bounding boxes to insert in r-tree
    std::vector<box> aabbs = obj.getAABBs(inflateSize);
    for (int i = 0; i < aabbs.size(); i++)
    {
        std::string index_string = std::string() + type + "_" + std::to_string(id) + "_obb_" + std::to_string(i);
        obbs.insert(std::make_pair(index_string, obj.obbs[i]));
        index.insert(std::make_pair(aabbs[i], index_string));
    }
}

void World::removeObject(const int id, const std::string &type, const int noOfObbs, const double inflateSize)
{
    for (int i = 0; i < noOfObbs; i++)
    {
        std::string index_string = std::string() + type + "_" + std::to_string(id) + "_obb_" + std::to_string(i);
        OBB obbToDelete = obbs.at(index_string);
        index.remove(std::make_pair(obbToDelete.getAABB(inflateSize), index_string));
        obbs.erase(index_string);
    }
}

bool World::checkPointValidity(const Eigen::Vector3d &point,  const bool canPassGate)
{
    std::vector<value> potentialHits;
    index.query(boost::geometry::index::contains(point), std::back_inserter(potentialHits));

    for (const value &v : potentialHits)
    {
        OBB obb = obbs.at(v.second);

        const bool isGate = v.second.find("gate") != std::string::npos;
        const double inflateSize = isGate ? inflateSizeGate : inflateSizeObstacle;
        
        if(obb.type=="filling" and canPassGate){
            continue;
        }

        if (obb.checkCollisionWithPoint(point, inflateSize))
        {   
            return false;
        }
    }

    return true;
}

bool World::checkPointValidity(const Eigen::Vector3d& point, const double minDistance) const{
    std::vector<value> potentialHits;
    index.query(boost::geometry::index::contains(point), std::back_inserter(potentialHits));

    for (const value &v : potentialHits)
    {
        OBB obb = obbs.at(v.second);

        // allowed to pass gates
        if(obb.type=="filling"){
            continue;
        }

        if (obb.checkCollisionWithPoint(point, minDistance))
        {   
            //std::cout << "Collision with " << v.second << std::endl;
            return false;
        }
    }
    return true;
}

bool World::checkRayValid(const Eigen::Vector3d &start, const Eigen::Vector3d &end, const bool canPassGate)
{
    // create ray bounding box
    // ToDo how can we make this more efficient
    Eigen::Matrix<double, 3, 2> ray;
    ray.col(0) = start;
    ray.col(1) = end;
    Eigen::Vector3d rayMin = ray.rowwise().minCoeff();
    Eigen::Vector3d rayMax = ray.rowwise().maxCoeff();
    box rayBox(rayMin, rayMax);

    // find potential hits
    std::vector<value> potentialHits;
    index.query(boost::geometry::index::intersects(rayBox), std::back_inserter(potentialHits));

    for (const value &v : potentialHits)
    {
        OBB obb = obbs.at(v.second);
        bool isGate = v.second.find("gate") != std::string::npos;
        // Allow to pass gates if canPassGate is true
        if(obb.type=="filling" and canPassGate){
            continue;
        }

        double inflateSize = isGate ? inflateSizeGate : inflateSizeObstacle;

        if (obb.checkCollisionWithRay(start, end, inflateSize))
        {
            return false;
        }
    }
    return true;
}