#include "ConfigParserYAML.h"
#include <string>
#include "Object.h"

int main(void)
{
    std::string configPath = "/home/tim/code/lsy_drone_racing/config.json";
    ConfigParser configParser(configPath);

    std::vector<OBBDescription> obbDescriptions = configParser.getGateGeometryByTypeId(1);
    Eigen::Vector3d globalCenter(0, 0, 0);
    const double rot = 3.14 / 2;
    Eigen::Vector3d globalRotation(0, 0, rot);
    Object gate = Object::createFromDescription(globalCenter, globalRotation, obbDescriptions);
    gate.printObbs();
}