#include "OBB.h"
#include <Eigen/Dense>
#include "Types.h"
#include "ConfigParser.h"
#include "Object.h"

#include <iostream>

int main(void){

//    // create small gate object
//    std::string configPath = "../config.json";
//    ConfigParser configParser(configPath);

//     std::vector<OBBDescription> obbDescriptions = configParser.getGateGeometryByTypeId(1);
//     Eigen::Vector3d globalCenter(0, 0, 0);
//     Eigen::Vector3d globalRotation(0, 0, 0);
//     Object gate = Object::createFromDescription(globalCenter, globalRotation, obbDescriptions);

//     Eigen::Vector3d collisionCheckpoint(0, 1, 0.525);


//     // print all obbs
//     int i = 0;
//     for (auto obb : gate.obbs)
//     {   std:: cout << "OBB " << i++ << std::endl;
//         std::cout << "Center: " << obb.center.transpose() << std::endl;
//         std::cout << "Half size: " << obb.halfSize.transpose() << std::endl;
//         std::cout << "Type: " << obb.type << std::endl;
//         std::cout << "Rotation: " << obb.rotation << std::endl;

//         // check collision with point
//         bool isColliding = obb.checkCollisionWithPoint(collisionCheckpoint, 0);
    
//     }

    Eigen::Vector3d globalCenter(0, 0, 0);
    Eigen::Vector3d halfSizes(0.5, 0.05, 0.05);

    OBB obb(globalCenter, halfSizes, "obstacle", "test");
    OBBDescription obbDescription;
    obbDescription.center = globalCenter;
    obbDescription.halfSize = halfSizes;
    obbDescription.type = "obstacle";

    Eigen::Vector3d objectPosition(0, 0, 0);
    Eigen::Vector3d objectRotation(0, 0, 0);
    Object object = Object::createFromDescription(objectPosition, objectRotation, {obbDescription});
    object.rotateZ(180, false);

    Eigen::Vector3d collisionCheckpoint(0, 1, 0);

    int i = 0;
    for (auto obb : object.obbs)
    {   std:: cout << "OBB " << i++ << std::endl;
        std::cout << "Center: " << obb.center.transpose() << std::endl;
        std::cout << "Half size: " << obb.halfSize.transpose() << std::endl;
        std::cout << "Type: " << obb.type << std::endl;
        std::cout << "Rotation: " << obb.rotation << std::endl;

        // check collision with point
        bool isColliding = obb.checkCollisionWithPoint(collisionCheckpoint, 0);
    
    }
}