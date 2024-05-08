#include <Eigen/Dense>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include "OBB.h"

// Register Eigen::Vector3f as a point type to Boost Geometry
BOOST_GEOMETRY_REGISTER_POINT_3D(Eigen::Vector3f, float, boost::geometry::cs::cartesian, x(), y(), z())

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef Eigen::Vector3f point;
typedef bg::model::box<point> box;
typedef bgi::rtree<point, bgi::quadratic<16>> rtree;

class Map{
    public:
    Map(double lowerBound, double upperBound, double droneRadius);

    private:
    rtree index = rtree();
    double lowerBound;
    double upperBound;
    double droneRadius;
    std::vector<OBB> obbs;
};
