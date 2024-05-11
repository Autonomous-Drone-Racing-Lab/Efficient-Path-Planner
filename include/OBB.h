#include <Eigen/Dense>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include "OBB.h"
#include "Object.h"

// Register Eigen::Vector3f as a point type to Boost Geometry
BOOST_GEOMETRY_REGISTER_POINT_3D(Eigen::Vector3f, float, boost::geometry::cs::cartesian, x(), y(), z())

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef Eigen::Vector3f point;
typedef bg::model::box<point> box;
typedef std::pair<box, unsigned> value;
typedef bgi::rtree<point, bgi::quadratic<16>> rtree;

class OBB
{
public:
    OBB(Eigen::Vector3f center, Eigen::Vector3f halfSize, Eigen::Matrix3f rotation, char *type);

    bool checkCollisionWithRay(Eigen::Vector3f start, Eigen::Vector3f end, float inflateSize);
    bool checkCollisionWithPoint(Eigen::Vector3f point, float inflateSize);

    Eigen::Vector3f center;
    Eigen::Vector3f halfSize;
    Eigen::Matrix3f rotation;
    std::string type;
    box boundingBox;
};