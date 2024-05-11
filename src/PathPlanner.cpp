#include "PathPlanner.h"
#include "Object.h"

void PathPlanner::addGate(int gateId, Eigen::VectorXf coordinates, bool subtractGateHeight)
{
    Eigen::Vector3f pos = coordinates.head(3);
    Eigen::Vector3f rot = coordinates.segment(3, 3);
    int type = coordinates(6);

    // create object
    Object obj = Object::createFromDescription(pos, rot, configParser.getObbDescriptions(type));
}