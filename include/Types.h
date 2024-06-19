#pragma once

#include "json.h"
#include <Eigen/Dense>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <map>


// Register Eigen::Vector3d as a point type to Boost Geometry
BOOST_GEOMETRY_REGISTER_POINT_3D(Eigen::Vector3d, double, boost::geometry::cs::cartesian, x(), y(), z())


typedef Eigen::Vector3d point;
typedef boost::geometry::model::box<point> box;
typedef std::pair<box, std::string> value;
typedef boost::geometry::index::rtree<value, boost::geometry::index::quadratic<16>> rtree;

struct OBBDescription
{
    Eigen::Vector3d center;
    Eigen::Vector3d halfSize;
    std::string type;
    std::string name;
};

struct ObjectProperties
{
    double height;
};

struct WorldProperties
{
    Eigen::Vector3d lowerBound;
    Eigen::Vector3d upperBound;
    std::map<std::string, double> inflateRadius;
};

struct PathPlannerProperties
{
    double optimalityThresholdPercentage;
    double timeLimitOnline;
    double timeLimitOffline;
    double checkpointGateOffset;
    double range;
    double minDistCheckTrajCollision;
    std::string pathSimplification;
    bool recalculateOnline;
    bool advanceForCalculation;
    bool canPassGate;
    
};

struct TrajectoryGeneratorProperties{
    double maxVelocity;
    double maxAcceleration;
    double samplingInterval;
    std::string type;
    double maxTime;

    double maxTrajDivergence;
    double prependTrajTime;
};
