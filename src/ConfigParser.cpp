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

    parseGeometries(config);
    std::cout << "Parsing geometries done" << std::endl;
    parseObjectProperties(config);
    std::cout << "Parsing object properties done" << std::endl;
    parseWorldProperties(config);
    std::cout << "Parsing world properties done" << std::endl;
    parsePathPlannerProperties(config);
    std::cout << "Parsing path planner properties done" << std::endl;
    parseTrajectoryGeneratorProperties(config);
    std::cout << "Parsing trajectory generator properties done" << std::endl;
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

const WorldProperties& ConfigParser::getWorldProperties() const
{
return worldProperties;
}

const PathPlannerProperties& ConfigParser::getPathPlannerProperties() const
{
return pathPlannerProperties;
}

const TrajectoryGeneratorProperties& ConfigParser::getTrajectoryGeneratorProperties() const
{
return trajectoryGeneratorProperties;
}

void ConfigParser::parseGeometries(json& config)
{
    for (auto& component : config["component_geometry"].items())
    {
        std::vector<OBBDescription> obbDescriptions;
        for (auto& obb : component.value())
        {
            Eigen::Vector3d position = Eigen::Vector3d(obb["position"][0], obb["position"][1], obb["position"][2]);
            Eigen::Vector3d halfSize = Eigen::Vector3d(obb["size"][0] , obb["size"][1], obb["size"][2]);
            halfSize /= 2;
            std::string type = obb["type"];
            std::string name = obb["name"];

            obbDescriptions.push_back({position, halfSize, type, name});
        }

        std::string componentName = component.key();
        objects.insert({componentName, obbDescriptions});
    }
}

void ConfigParser::parseObjectProperties(json& config)
{
    for (auto& object : config["component_properties"].items())
    {
        std::string key = object.key();
        double height = object.value()["height"];
        objectProperties.insert({key, {height}});
    }
}

void ConfigParser::parseWorldProperties(json& config)
{
    worldProperties.lowerBound = Eigen::Vector3d(config["world_properties"]["lower_bound"][0], config["world_properties"]["lower_bound"][1], config["world_properties"]["lower_bound"][2]);
    worldProperties.upperBound = Eigen::Vector3d(config["world_properties"]["upper_bound"][0], config["world_properties"]["upper_bound"][1], config["world_properties"]["upper_bound"][2]);
    worldProperties.inflateRadius = config["world_properties"]["inflate_radius"];
}

void ConfigParser::parsePathPlannerProperties(json& config)
{
    pathPlannerProperties.optimalityThresholdPercentage = config["path_planner_properties"]["optimality_threshold_percentage"];
    pathPlannerProperties.timeLimitOnline = config["path_planner_properties"]["time_limit_online"];
    pathPlannerProperties.timeLimitOffline = config["path_planner_properties"]["time_limit_offline"];
    pathPlannerProperties.checkpointGateOffset = config["path_planner_properties"]["checkpoint_gate_offset"];
    pathPlannerProperties.range = config["path_planner_properties"]["range"];
    pathPlannerProperties.posDivergenceRecalculate = config["path_planner_properties"]["pos_divergence_recalculate"];
    pathPlannerProperties.rotDivergenceRecalculate = config["path_planner_properties"]["rot_divergence_recalculate"];
}

void ConfigParser::parseTrajectoryGeneratorProperties(json& config)
{
    trajectoryGeneratorProperties.maxVelocity = config["trajectory_generator_properties"]["max_velocity"];
    trajectoryGeneratorProperties.maxAcceleration = config["trajectory_generator_properties"]["max_acceleration"];
    trajectoryGeneratorProperties.samplingInterval = config["trajectory_generator_properties"]["sampling_interval"];
}


