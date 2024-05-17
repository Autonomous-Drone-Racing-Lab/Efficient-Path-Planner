#include "OBB.h"
#include <Eigen/Dense>
#include "Types.h"
#include "ConfigParser.h"
#include <string>
#include <iostream>

OBB::OBB(const Eigen::Vector3d &center, const Eigen::Vector3d &halfSize, const std::string &type, const std::string&name) :  center(center), halfSize(halfSize), type(type), name(name){}

bool OBB::checkCollisionWithRay(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const double inflateSize)const{
    // Transform the ray into the OBB's local coordinate system
    const Eigen::Vector3d localStart = rotation.transpose() * (start - center);
    const Eigen::Vector3d localEnd = rotation.transpose() * (end - center);
    const Eigen::Vector3d localDirection = (localEnd - localStart);

    double tMin = 0;
    double tMax = 1;

    const Eigen::Vector3d inflatedHalfSize = halfSize + Eigen::Vector3d(inflateSize, inflateSize, inflateSize);
    const Eigen::Vector3d box_min = -inflatedHalfSize;
    const Eigen::Vector3d box_max = inflatedHalfSize;

    for(int i = 0; i < 3; i++){
        if(abs(localDirection(i)) < 0.0001){
            // Rays is parallel to the plane. No hit if the ray is outside the box
            if(localStart(i) < box_min(i) || localStart(i) > box_max(i)){
                return false;
            }
        }else{
            double invD = 1.0f / localDirection(i);
            double t1 = (box_min(i) - localStart(i)) * invD;
            double t2 = (box_max(i) - localStart(i)) * invD;

            double tEntry = std::min(t1, t2);
            double tExit = std::max(t1, t2);

            tMin = std::max(tMin, tEntry);
            tMax = std::min(tMax, tExit);

            if(tMin > tMax){
                return false;
            }
            
        }
    }
    return 0 <= tMin && tMin <= 1 && 0 <= tMax && tMax <= 1;
}

bool OBB::checkCollisionWithPoint(const Eigen::Vector3d& point, const double inflateSize) const {
    // Transform the point into the OBB's local coordinate system
    const Eigen::Vector3d localPoint = rotation.transpose() * (point - center);
    Eigen::Vector3d collisionHalfSizes = halfSize;
    
    if(shouldBeInflated()){
       collisionHalfSizes += Eigen::Vector3d(inflateSize, inflateSize, inflateSize);
    }

    
    
    
    // Check if the point is within the inflated bounding box
    const bool isColliding = (std::abs(localPoint.x()) <= collisionHalfSizes.x()) &&
               (std::abs(localPoint.y()) <= collisionHalfSizes.y()) &&
               (std::abs(localPoint.z()) <= collisionHalfSizes.z());

    if(isColliding){
        std::cout << "Collision of object " << name << " with point " << point << std::endl;
        std::cout << "Local point: " << localPoint.transpose() << std::endl;
        std::cout << "Half sizes: " << collisionHalfSizes.transpose() << std::endl;
        std::cout << "Center: " << center.transpose() << std::endl;
        std::cout << "Rotation: \n" << rotation << std::endl;
    }

    return isColliding;
}


box OBB::getAABB(const double inflateSize) const {
   // Get world fram AABB by rotating object into world frame and doing worst case estimation
   // Calculate the cornoes of the AABB based on the OBB center, half sizes and rotation
   // For this first create unit cube, scaled by halfSize
    
    Eigen::Matrix3Xd corners(3, 8);
    corners << -1,  1,  1, -1, -1,  1,  1, -1,
               -1, -1,  1,  1, -1, -1,  1,  1,
               -1, -1, -1, -1,  1,  1,  1,  1;
    
    Eigen::Matrix<double, 3, 8> halfSizesStacked = halfSize.replicate(1, 8);
    corners = corners.cwiseProduct(halfSizesStacked);

    // Rotate the corners
    // Todo, with or whithout transpose?
    const Eigen::Matrix<double, 3, 8> centerStacked = center.replicate(1, 8);
    const Eigen::Matrix3Xd cornersGlobalFrame = rotation * corners + centerStacked;
    Eigen::Vector3d minCorner = cornersGlobalFrame.rowwise().minCoeff();
    Eigen::Vector3d maxCorner = cornersGlobalFrame.rowwise().maxCoeff();

    // Inflate the AABB
    const Eigen::Vector3d inflateSizeVec(inflateSize, inflateSize, inflateSize);
    // conly collision objects are inflated, filling objects are not
    if(shouldBeInflated()){
            minCorner -= inflateSizeVec;
            maxCorner += inflateSizeVec;
    }


    return box(minCorner, maxCorner);
}