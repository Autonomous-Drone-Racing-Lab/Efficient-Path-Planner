#pragma once

#include <Eigen/Dense>
#include "Types.h"
#include "ConfigParser.h"
#include <string>


class OBB
{
public:
    OBB(const Eigen::Vector3f &center, const Eigen::Vector3f &halfSize, const std::string &type);

    bool checkCollisionWithRay(const Eigen::Vector3f &start, const Eigen::Vector3f &end, const float inflateSize) const;
    bool checkCollisionWithPoint(const Eigen::Vector3f &point, const float inflateSize) const;
    box getAABB(const float inflateSize) const;

    Eigen::Vector3f center;
    Eigen::Vector3f halfSize;
    Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity(); // No rotation on initialization
    std::string type;
    box boundingBox;
};