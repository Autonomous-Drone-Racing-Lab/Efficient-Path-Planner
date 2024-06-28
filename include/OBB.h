#pragma once

#include <Eigen/Dense>
#include "Types.h"
#include "ConfigParserYAML.h"
#include <string>


class OBB
{
public:
    
    /**
     * @brief Constructs an OBB (Oriented Bounding Box) object.
     *
     * @param center The center point of the OBB.
     * @param halfSize The half-size of the OBB along each axis.
     * @param type The type of the OBB. It has consequences on inflating and collision checking. Supported types are collision and filling
     * @param name The name of the OBB. It is to make it uniquely identifiable
     */
    OBB(const Eigen::Vector3d &center, const Eigen::Vector3d &halfSize, const std::string &type, const std::string &name);


    /**
     * Checks for collision between an oriented bounding box (OBB) and a ray.
     *
     * @param start The starting point of the ray.
     * @param end The ending point of the ray.
     * @param inflateSize The size by which to inflate the OBB for collision checking.
     * @return True if the OBB collides with the ray, false otherwise.
     */
    bool checkCollisionWithRay(const Eigen::Vector3d &start, const Eigen::Vector3d &end, const double inflateSize) const;
    /**
     * Checks if a given point collides with the oriented bounding box (OBB).
     *
     * @param point The point to check for collision.
     * @param inflateSize The size by which to inflate the OBB for collision checking.
     * @return True if the point collides with the inflated OBB, false otherwise.
     */
    bool checkCollisionWithPoint(const Eigen::Vector3d &point, const double inflateSize) const;
    
    /**
     * @brief Calculates the Axis-Aligned Bounding Box (AABB) of the OBB.
     * 
     * @param inflateSize The amount by which to inflate the AABB.
     * @return The inflated AABB of the OBB.
     */
    box getAABB(const double inflateSize) const;

    Eigen::Vector3d center;
    Eigen::Vector3d halfSize;
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity(); // No rotation on initialization
    std::string type;
    std::string name;
    box boundingBox;
    
    bool shouldBeInflated() const{
        return type == "collision";
    }
};