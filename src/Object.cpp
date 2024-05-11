#include "Object.h"

Object::Object(Eigen::Vector3f globalCenter, Eigen::Matrix3f globalRotation, std::vector<OBB> obbs) : globalCenter(globalCenter), globalRotation(globalRotation), obbs(obbs) {}

Object Object::createFromDescription(Eigen::Vector3f globalCenter, Eigen::Matrix3f globalRotation, std::vector<OBBDescription> &obbDescriptions)
{
    std::vector<OBB> obbs;
    for (OBBDescription obbDescription : obbDescriptions)
    {
        obbs.push_back(OBB(obbDescription.center, obbDescription.halfSize, obbDescription.rotation, obbDescription.type));
    }
    return Object(globalCenter, globalRotation, obbs);
}

void Object::translate(Eigen::Vector3f translation)
{
    globalCenter += translation;
    for (OBB &obb : obbs)
    {
        obb.center += translation;
    }
}

void Object::rotateZ(float angle, bool useRadian)
{
    if (!useRadian)
    {
        angle = angle * M_PI / 180;
    }

    float cos_angle = cos(angle);
    float sin_angle = sin(angle);

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