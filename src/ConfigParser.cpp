#include "ConfigParser.h"
#include "json.h"
#include "Types.h"
#include <vector>
#include <map>
#include <string>
#include <fstream>
#include <iostream>

ConfigParser::ConfigParser(const std::string& configPath)
{

    std::ifstream f(configPath);
    config = json::parse(f);

    std::cout << "Parsed config: " << std::endl;

    // parse geometries
    for (auto &component : config["component_geometry"].items())
    {
        std::vector<OBBDescription> obbDescriptions;
        for (auto &obb : component.value())
        {
            Eigen::Vector3f position = Eigen::Vector3f(obb["position"][0], obb["position"][1], obb["position"][2]);
            Eigen::Vector3f halfSize = Eigen::Vector3f(obb["size"][0], obb["size"][1], obb["size"][2]);
            std::string type = obb["type"];

            obbDescriptions.push_back({position, halfSize, type});
        }

        std::string componentName = component.key();
        objects.insert({componentName, obbDescriptions});
    }

    std::cout << "Parsed geometries: " << std::endl;

    // parse object properties
    for (auto &object : config["component_properties"].items())
    {
        std::string key = object.key();
        float height = object.value()["height"];
        objectProperties.insert({key, {height}});
    }

    std::cout << "Parsed object properties: " << std::endl;
}

const std::vector<OBBDescription>& ConfigParser::getGateGeometryByTypeId(const int typeId) const
{
    std::string key = config["gate_id_to_name_mapping"][std::to_string(typeId)];
    return objects.at(key);
}

const ObjectProperties& ConfigParser::getObjectPropertiesByTypeId(const int typeId) const
{
    std::string key = config["gate_id_to_name_mapping"][std::to_string(typeId)];
    return objectProperties.at(key);
}

const std::vector<OBBDescription>& ConfigParser::getObstacleGeometry() const
{
    return objects.at("obstacle");
}