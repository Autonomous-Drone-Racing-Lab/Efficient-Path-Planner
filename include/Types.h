#pragma once

#include "json.h"
#include <Eigen/Dense>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/geometries/register/point.hpp>


// Register Eigen::Vector3f as a point type to Boost Geometry
BOOST_GEOMETRY_REGISTER_POINT_3D(Eigen::Vector3f, float, boost::geometry::cs::cartesian, x(), y(), z())


typedef Eigen::Vector3f point;
typedef boost::geometry::model::box<point> box;
typedef std::pair<box, std::string> value;
typedef boost::geometry::index::rtree<value, boost::geometry::index::quadratic<16>> rtree;

struct OBBDescription
{
    Eigen::Vector3f center;
    Eigen::Vector3f halfSize;
    std::string type;
};

struct ObjectProperties
{
    float height;
};