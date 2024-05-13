#include "OBB.h"
#include <Eigen/Dense>
#include "Types.h"
#include <iostream>

int main(void){

    Eigen::Vector3f center(0, 0, 0);
    Eigen::Vector3f halfSize(0.5,0.5, 0.5);
    std::string type = "collision";

    OBB obb(center, halfSize, type);


    float inflateSize = 0.0;

    box aabb = obb.getAABB(inflateSize);

    std::cout << "AABB: " << aabb.min_corner() << " " << aabb.max_corner() << std::endl;


    Eigen::Vector3f start(0, 0, 0);
    Eigen::Vector3f end(1, 1, 1);

    bool collision = obb.checkCollisionWithRay(start, end, inflateSize);
    std::cout << "Collision With ray: " << collision << std::endl;

    Eigen::Vector3f point(0.5, 0.5, 0.5);
    collision = obb.checkCollisionWithPoint(point, inflateSize);
    std::cout << "Collision With point: " << collision << std::endl;
}