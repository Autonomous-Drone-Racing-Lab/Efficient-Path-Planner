#include "OBB.h"

OBB::OBB(Eigen::Vector3f center, Eigen::Vector3f halfSize, Eigen::Matrix3f rotation, char* type) : center(center), halfSize(halfSize), rotation(rotation), type(type) {}

bool OBB::checkCollisionWithRay(Eigen::Vector3f start, Eigen::Vector3f end, float inflateSize){
    // Transform the ray into the OBB's local coordinate system
    Eigen::Vector3f localStart = rotation.transpose() * (start - center);
    Eigen::Vector3f localEnd = rotation.transpose() * (end - center);
    Eigen::Vector3f localDirection = (localEnd - localStart).normalized();

    float tMin = 0;
    float tMax = 1;

    Eigen::Vector3f inflatedHalfSize = halfSize + Eigen::Vector3f(inflateSize, inflateSize, inflateSize);
    Eigen::Vector3f box_min = -inflatedHalfSize;
    Eigen::Vector3f box_max = inflatedHalfSize;

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

            float tMin = std::max(tMin, tEntry);
            float tMax = std::min(tMax, tExit);

            if(tMin > tMax){
                return false;
            }

            return 0 <= tMin && tMin <= 1 && 0 <= tMax && tMax <= 1;
        }
    }
}