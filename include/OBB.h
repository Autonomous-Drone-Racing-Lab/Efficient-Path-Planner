#pragma once

#include <Eigen/Dense>
#include "Types.h"
#include "ConfigParser.h"
#include <string>


class OBB
{
public:
    OBB(const Eigen::Vector3d &center, const Eigen::Vector3d &halfSize, const std::string &type);

    bool checkCollisionWithRay(const Eigen::Vector3d &start, const Eigen::Vector3d &end, const double inflateSize) const;
    bool checkCollisionWithPoint(const Eigen::Vector3d &point, const double inflateSize) const;
    box getAABB(const double inflateSize) const;

    Eigen::Vector3d center;
    Eigen::Vector3d halfSize;
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity(); // No rotation on initialization
    std::string type;
    box boundingBox;
};