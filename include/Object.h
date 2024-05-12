
#pragma once

#include "OBB.h"
#include <Eigen/Dense>
#include "Types.h"
#include <string>
#include <vector>


class Object
{
public:
    Object(const Eigen::Vector3f &globalCenter, const Eigen::Matrix3f &globalRotation, const std::vector<OBB> &obbs);

    static Object createFromDescription(const Eigen::Vector3f &globalCenter, const Eigen::Vector3f &globalRotation, const std::vector<OBBDescription> &obbDescriptions);

    void translate(const Eigen::Vector3f &translation);
    void rotateZ(const float angle, const bool useRadian = true);

    std::vector<box> getAABBs(const float inflateSize) const;

    std::vector<OBB> obbs;
    Eigen::Vector3f globalCenter;
    Eigen::Matrix3f globalRotation;
};