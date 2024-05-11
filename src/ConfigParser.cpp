#include "ConfigParser.h"
#include <fstream>

ConfigParser::ConfigParser(std::string configPath)
{

    std::ifstream f(configPath);
    config = json::parse(f);

    // parse geometries
    for (auto &component : config["component_geometry"].items())
    {
        std::vector<OBBDescription> obbDescriptions;
        for (auto &obb : component.value())
        {
            Eigen::Vector3f position = Eigen::Vector3f(obb["position"][0], obb["position"][1], obb["position"][2]);
            Eigen::Vector3f halfSize = Eigen::Vector3f(obb["halfSize"][0], obb["halfSize"][1], obb["halfSize"][2]);
            std::string type = obb["type"];

            obbDescriptions.push_back({position, halfSize, type});
        }

        std::string componentName = component.key();
        objects.insert({componentName, obbDescriptions});
    }

    // parse object properties
    for (auto &object : config["object_properties"].items())
    {
        std::string key = object.key();
        float height = object.value()["height"];
        objectProperties.insert({key, {height}});
    }
}

std::vector<OBBDescription> ConfigParser::getGateGeometryByTypeId(int typeId)
{
    std::string key = config["gate_id_to_name_mapping"][std::to_string(typeId)];
    return objects[key];
}

ObjectProperties ConfigParser::getObjectPropertiesByTypeId(int typeId)
{
    std::string key = config["object_id_to_name_mapping"][std::to_string(typeId)];
    return objectProperties[key];
}

std::vector<OBBDescription> ConfigParser::getObstacleGeometry()
{
    return objects["obstacle"];
}