#include <Eigen/Dense>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include "OBB.h"
#include "Object.h"
#include "ConfigParser.h"

// Register Eigen::Vector3f as a point type to Boost Geometry
BOOST_GEOMETRY_REGISTER_POINT_3D(Eigen::Vector3f, float, boost::geometry::cs::cartesian, x(), y(), z())

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef Eigen::Vector3f point;
typedef bg::model::box<point> box;
typedef std::pair<box, unsigned> value;
typedef bgi::rtree<point, bgi::quadratic<16>> rtree;

class PathPlanner
{
public:
    PathPlanner(float minHeight, float maxHeight);

    void addGate(int gateId, Eigen::VectorXf coordinates, bool subtractGateHeight);
    void removeGate(int gateId);
    void updateGateMidflight(int gateId, Eigen::VectorXf coordinates, bool withinRange);

private:
    std::map<unsigned int, Object> gates;
    rtree index = rtree();
    float minHeight = 0;
    float maxHeight = 2;
    ConfigParser configParser;
};