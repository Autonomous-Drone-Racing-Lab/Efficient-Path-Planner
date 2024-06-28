#include "Object.h"

#include "OBB.h"
#include <Eigen/Dense>
#include "Types.h"
#include <string>
#include <vector>
#include <iostream>
#include <stdexcept>

Object::Object(const Eigen::Vector3d& center, const double rot_z, const std::vector<OBB>& obbs): obbs(obbs){
    this->globalCenter = Eigen::Vector3d(0,0,0);
    this->globalRotation = Eigen::Matrix3d::Identity();
    
    if(center(2) > 1e-6){
        std::cerr << "Center z position must be zero" << std::endl;
        throw std::runtime_error("Center z position must be zero");
    }

    this->translate(center);
    this->rotateZ(rot_z, true);
}

Object Object::createFromDescription(const Eigen::Vector3d& globalCenter, const Eigen::Vector3d& globalRotation,  const std::vector<OBBDescription> &obbDescriptions)
{
    std::vector<OBB> obbs;
    for (const OBBDescription &obbDescription : obbDescriptions)
    {
        obbs.push_back(OBB(obbDescription.center, obbDescription.halfSize, obbDescription.type, obbDescription.name));
    }

    const double rot_theta = globalRotation(0);
    const double rot_psi = globalRotation(1);
    const double rot_phi = globalRotation(2);

    if(abs(rot_theta) > 1e-6){
        std::cerr << "Rotation around x axis is not supported" << std::endl;
        throw std::runtime_error("Rotation around x axis is not supported");
    }
    if (abs(rot_psi) > 1e-6){
        std::cerr << "Rotation around y axis is not supported" << std::endl;
        throw std::runtime_error("Rotation around y axis is not supported");
    }

    return Object(globalCenter, rot_phi, obbs);
}

void Object::translate(const Eigen::Vector3d& translation)
{
    globalCenter += translation;
    for (OBB &obb : obbs)
    {
        obb.center += translation;
    }
}

void Object::rotateZ(const double angle, const bool useRadian)
{   double angleRadian = angle;
    if (!useRadian)
    {
        angleRadian = angleRadian * M_PI / 180;
    }

    double cos_angle = cos(angleRadian);
    double sin_angle = sin(angleRadian);

    Eigen::Matrix3d rotation;
    rotation << cos_angle, -sin_angle, 0,
        sin_angle, cos_angle, 0,
        0, 0, 1;
    globalRotation = rotation * globalRotation;

    // apply rotation for each obb
    for (OBB &obb : obbs)
    {
        Eigen::Vector3d relativeCenter = obb.center - globalCenter;
        obb.center = rotation * relativeCenter + globalCenter;
        obb.rotation = rotation * obb.rotation;
    }
}

std::vector<box> Object::getAABBs(const double inflateSize) const
{
    std::vector<box> aabbs;
    for (OBB obb : obbs)
    {
        aabbs.push_back(obb.getAABB(inflateSize));
    }
    return aabbs;
}