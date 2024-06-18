
#pragma once

#include "OBB.h"
#include <Eigen/Dense>
#include "Types.h"
#include <string>
#include <vector>
#include <iostream>


class Object
{
public:
    Object(const Eigen::Vector3d &center, const double rot_z, const std::vector<OBB> &obbs);

    static Object createFromDescription(const Eigen::Vector3d &globalCenter, const Eigen::Vector3d &globalRotation, const std::vector<OBBDescription> &obbDescriptions);

    void translate(const Eigen::Vector3d &translation);
    void rotateZ(const double angle, const bool useRadian = true);

    std::vector<box> getAABBs(const double inflateSize) const;
    void printObbs() {
        int i = 0;
        for (auto obb : obbs)
        {
            std::cout << "OBB " << i++ << std::endl;
            std::cout << "Center: " << obb.center.transpose() << std::endl;
        }
    }

    std::vector<OBB> obbs;
    Eigen::Vector3d globalCenter;
    Eigen::Matrix3d globalRotation;
};