#include "OBB.h"
#include <Eigen/Dense>
#include "Types.h"
#include "ConfigParser.h"
#include <string>
#include <iostream>

OBB::OBB(const Eigen::Vector3f &center, const Eigen::Vector3f &halfSize, const std::string &type) : center(center), halfSize(halfSize), type(type) {}

bool OBB::checkCollisionWithRay(const Eigen::Vector3f& start, const Eigen::Vector3f& end, const float inflateSize)const{
    // Transform the ray into the OBB's local coordinate system
    const Eigen::Vector3f localStart = rotation.transpose() * (start - center);
    const Eigen::Vector3f localEnd = rotation.transpose() * (end - center);
    const Eigen::Vector3f localDirection = (localEnd - localStart);

    float tMin = 0;
    float tMax = 1;

    const Eigen::Vector3f inflatedHalfSize = halfSize + Eigen::Vector3f(inflateSize, inflateSize, inflateSize);
    const Eigen::Vector3f box_min = -inflatedHalfSize;
    const Eigen::Vector3f box_max = inflatedHalfSize;

    for(int i = 0; i < 3; i++){
        if(abs(localDirection(i)) < 0.0001){
            // Rays is parallel to the plane. No hit if the ray is outside the box
            if(localStart(i) < box_min(i) || localStart(i) > box_max(i)){
                return false;
            }
        }else{
            float invD = 1.0f / localDirection(i);
            float t1 = (box_min(i) - localStart(i)) * invD;
            float t2 = (box_max(i) - localStart(i)) * invD;

            float tEntry = std::min(t1, t2);
            float tExit = std::max(t1, t2);

            tMin = std::max(tMin, tEntry);
            tMax = std::min(tMax, tExit);

            if(tMin > tMax){
                return false;
            }
            
        }
    }
    return 0 <= tMin && tMin <= 1 && 0 <= tMax && tMax <= 1;
}

bool OBB::checkCollisionWithPoint(const Eigen::Vector3f& point, const float inflateSize)const{
    // Transform the point into the OBB's local coordinate system
    const Eigen::Vector3f localPoint = rotation.transpose() * (point - center);
    const Eigen::Vector3f inflatedHalfSize = halfSize + Eigen::Vector3f(inflateSize, inflateSize, inflateSize);
    
    // Check if the point is within the inflated bounding box
    return (std::abs(localPoint.x()) <= inflatedHalfSize.x()) &&
               (std::abs(localPoint.y()) <= inflatedHalfSize.y()) &&
               (std::abs(localPoint.z()) <= inflatedHalfSize.z());
}


box OBB::getAABB(const float inflateSize) const {
   // Get world fram AABB by rotating object into world frame and doing worst case estimation
   // Calculate the cornoes of the AABB based on the OBB center, half sizes and rotation
   // For this first create unit cube, scaled by halfSize
    
    Eigen::Matrix3Xf corners(3, 8);
    corners << -1,  1,  1, -1, -1,  1,  1, -1,
               -1, -1,  1,  1, -1, -1,  1,  1,
               -1, -1, -1, -1,  1,  1,  1,  1;
    
    Eigen::Matrix<float, 3, 8> halfSizesStacked = halfSize.replicate(1, 8);
    corners = corners.cwiseProduct(halfSizesStacked);

    // Rotate the corners
    // Todo, with or whithout transpose?
    const Eigen::Matrix<float, 3, 8> centerStacked = center.replicate(1, 8);
    const Eigen::Matrix3Xf cornersGlobalFrame = (rotation * corners)+ centerStacked;
    Eigen::Vector3f minCorner = cornersGlobalFrame.rowwise().minCoeff();
    Eigen::Vector3f maxCorner = cornersGlobalFrame.rowwise().maxCoeff();

    // Inflate the AABB
    const Eigen::Vector3f inflateSizeVec(inflateSize, inflateSize, inflateSize);
    minCorner -= inflateSizeVec;
    maxCorner += inflateSizeVec;

    return box(minCorner, maxCorner);
}