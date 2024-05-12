#include "Object.h"

#include "OBB.h"
#include <Eigen/Dense>
#include "Types.h"
#include <string>
#include <vector>

Object::Object(const Eigen::Vector3f& globalCenter, const Eigen::Matrix3f& globalRotation, const std::vector<OBB>& obbs) :globalCenter(globalCenter), globalRotation(globalRotation){
    // tranform all obbs to global coordinates considereing global center and rotation
    std::vector<OBB> transformedOBBs = obbs;
    for (OBB &obb : transformedOBBs)
    {
        obb.rotation = globalRotation;
        obb.center += globalCenter;
    }
    this->obbs = transformedOBBs;
}

Object Object::createFromDescription(const Eigen::Vector3f& globalCenter, const Eigen::Vector3f& globalRotation,  const std::vector<OBBDescription> &obbDescriptions)
{
    std::vector<OBB> obbs;
    for (const OBBDescription &obbDescription : obbDescriptions)
    {
        obbs.push_back(OBB(obbDescription.center, obbDescription.halfSize, obbDescription.type));
    }

    // create rotation matrix from angles in global rotation (angles in radians)
    float cos_x = cos(globalRotation(0));
    float sin_x = sin(globalRotation(0));
    float cos_y = cos(globalRotation(1));
    float sin_y = sin(globalRotation(1));
    float cos_z = cos(globalRotation(2));
    float sin_z = sin(globalRotation(2));

    Eigen::Matrix3f rotation;
    rotation << cos_y * cos_z, -cos_y * sin_z, sin_y,
        cos_x * sin_z + sin_x * sin_y * cos_z, cos_x * cos_z - sin_x * sin_y * sin_z, -sin_x * cos_y,
        sin_x * sin_z - cos_x * sin_y * cos_z, sin_x * cos_z + cos_x * sin_y * sin_z, cos_x * cos_y;

    return Object(globalCenter, rotation, obbs);
}

void Object::translate(const Eigen::Vector3f& translation)
{
    globalCenter += translation;
    for (OBB &obb : obbs)
    {
        obb.center += translation;
    }
}

void Object::rotateZ(const float angle, const bool useRadian)
{   float angleRadian = angle;
    if (!useRadian)
    {
        angleRadian = angleRadian * M_PI / 180;
    }

    float cos_angle = cos(angleRadian);
    float sin_angle = sin(angleRadian);

    Eigen::Matrix3f rotation;
    rotation << cos_angle, -sin_angle, 0,
        sin_angle, cos_angle, 0,
        0, 0, 1;
    globalRotation = rotation * globalRotation;

    // apply rotation for each obb
    for (OBB &obb : obbs)
    {
        Eigen::Vector3f relativeCenter = obb.center - globalCenter;
        obb.center = rotation * relativeCenter + globalCenter;
        obb.rotation = rotation * obb.rotation;
    }
}

std::vector<box> Object::getAABBs(const float inflateSize) const
{
    std::vector<box> aabbs;
    for (OBB obb : obbs)
    {
        aabbs.push_back(obb.getAABB(inflateSize));
    }
    return aabbs;
}