
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
    /**
     * @brief Constructs an Object (e.g. gate obstacle) with the given center, rotation angle, and list of oriented bounding boxes (OBBs).
     *
     * @param center The center position of the Object as a 3D vector. The z position must always be zero
     * @param rot_z The rotation angle of the Object around the z-axis.
     * @param obbs The list of oriented bounding boxes (OBBs) that define the Object's shape.
     */
    Object(const Eigen::Vector3d &center, const double rot_z, const std::vector<OBB> &obbs);

    /**
     * @brief Creates an Object from the given description.
     *
     * This function creates an Object using the provided global center, global rotation, and a vector of OBB descriptions.
     *
     * @param globalCenter The global center of the Object. The z position must always be zero
     * @param globalRotation The global rotation of the Object.
     * @param obbDescriptions A vector of OBB descriptions.
     * @return The created Object.
     */
    static Object createFromDescription(const Eigen::Vector3d &globalCenter, const Eigen::Vector3d &globalRotation, const std::vector<OBBDescription> &obbDescriptions);

    /**
     * Translates the object by the specified translation vector.
     *
     * @param translation The translation vector to apply to the object.
     */
    void translate(const Eigen::Vector3d &translation);
    
    /**
     * Rotates the object around the Z-axis by the specified angle.
     *
     * @param angle The angle of rotation. If `useRadian` is set to `true`, the angle is in radians. Otherwise, it is in degrees.
     * @param useRadian Specifies whether the angle is in radians or degrees. Default is `true`.
     */
    void rotateZ(const double angle, const bool useRadian = true);

    /**
     * @brief Retrieves the Axis-Aligned Bounding Boxes (AABBs) for the object.
     * 
     * This function returns a vector of AABBs that represent the object's bounding boxes.
     * The `inflateSize` parameter allows you to specify an additional size to inflate the AABBs.
     * 
     * @param inflateSize The amount by which to inflate the AABBs (optional).
     * @return A vector of AABBs representing the object's bounding boxes.
     */
    std::vector<box> getAABBs(const double inflateSize) const;
    
    /**
     * Prints the information of each oriented bounding box (OBB) in the object.
     */
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