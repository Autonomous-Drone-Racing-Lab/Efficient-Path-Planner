#include "ConfigParserYAML.h"
#include <yaml-cpp/yaml.h>
#include "Types.h"
#include <vector>
#include <map>
#include <string>
#include <fstream>
#include <iostream>

ConfigParser::ConfigParser(const std::string &configPath)
{
    this->config = YAML::LoadFile(configPath);

    parseGeometries(config);
    parseObjectProperties(config);
    parseWorldProperties(config);
    parsePathPlannerProperties(config);
    parseTrajectoryGeneratorProperties(config);
}

const std::vector<OBBDescription> &ConfigParser::getGateGeometryByTypeId(const int typeId) const
{

    std::string key = config["gate_id_to_name_mapping"][std::to_string(typeId)].as<std::string>();
    return objects.at(key);
}

const ObjectProperties &ConfigParser::getObjectPropertiesByTypeId(const int typeId) const
{
    std::string key = config["gate_id_to_name_mapping"][std::to_string(typeId)].as<std::string>();
    return objectProperties.at(key);
}

const std::vector<OBBDescription> &ConfigParser::getObstacleGeometry() const
{
    return objects.at("obstacle");
}

const WorldProperties &ConfigParser::getWorldProperties() const
{
    return worldProperties;
}

const PathPlannerProperties &ConfigParser::getPathPlannerProperties() const
{
    return pathPlannerProperties;
}

const TrajectoryGeneratorProperties &ConfigParser::getTrajectoryGeneratorProperties() const
{
    return trajectoryGeneratorProperties;
}

void ConfigParser::parseGeometries(const YAML::Node &config)
{
    for (auto &component : config["component_geometry"])
    {
        std::vector<OBBDescription> obbDescriptions;
        for (auto &obbi : component.second)
        {
            auto obb = obbi.second;
            Eigen::Vector3d position = Eigen::Vector3d(obb["position"][0].as<double>(), obb["position"][1].as<double>(), obb["position"][2].as<double>());
            Eigen::Vector3d halfSize = Eigen::Vector3d(obb["size"][0].as<double>(), obb["size"][1].as<double>(), obb["size"][2].as<double>()) / 2;
            std::string type = obb["type"].as<std::string>();
            std::string name = obb["name"].as<std::string>();

            obbDescriptions.push_back({position, halfSize, type, name});
        }

        std::string componentName = component.first.as<std::string>();
        objects.insert({componentName, obbDescriptions});
    }
}

void ConfigParser::parseObjectProperties(const YAML::Node &config)
{
    for (auto &object : config["component_properties"])
    {
        std::string key = object.first.as<std::string>();
        double height = object.second["height"].as<double>();
        objectProperties.insert({key, {height}});
    }
}

void ConfigParser::parseWorldProperties(const YAML::Node &config)
{
    worldProperties.lowerBound = Eigen::Vector3d(config["world_properties"]["lower_bound"][0].as<double>(), config["world_properties"]["lower_bound"][1].as<double>(), config["world_properties"]["lower_bound"][2].as<double>());
    worldProperties.upperBound = Eigen::Vector3d(config["world_properties"]["upper_bound"][0].as<double>(), config["world_properties"]["upper_bound"][1].as<double>(), config["world_properties"]["upper_bound"][2].as<double>());
    worldProperties.inflateRadius = std::map<std::string, double>();
    worldProperties.inflateRadius["gate"] = config["world_properties"]["inflate_radius"]["gate"].as<double>();
    worldProperties.inflateRadius["obstacle"] = config["world_properties"]["inflate_radius"]["obstacle"].as<double>();
}

void ConfigParser::parsePathPlannerProperties(const YAML::Node &config)
{
    pathPlannerProperties.optimalityThresholdPercentage = config["path_planner_properties"]["optimality_threshold_percentage"].as<double>();
    pathPlannerProperties.timeLimitOnline = config["path_planner_properties"]["time_limit_online"].as<double>();
    pathPlannerProperties.timeLimitOffline = config["path_planner_properties"]["time_limit_offline"].as<double>();
    pathPlannerProperties.checkpointGateOffset = config["path_planner_properties"]["checkpoint_gate_offset"].as<double>();
    pathPlannerProperties.range = config["path_planner_properties"]["range"].as<double>();
    pathPlannerProperties.minDistCheckTrajCollision = config["path_planner_properties"]["min_dist_check_traj_collision"].as<double>();
    pathPlannerProperties.pathSimplification = config["path_planner_properties"]["path_simplification"].as<std::string>();
    pathPlannerProperties.recalculateOnline = config["path_planner_properties"]["recalculate_online"].as<bool>();
    pathPlannerProperties.canPassGate = config["path_planner_properties"]["can_pass_gate"].as<bool>();
    pathPlannerProperties.advanceForCalculation = config["path_planner_properties"]["advance_for_calculation"].as<bool>();
    pathPlannerProperties.planner = config["path_planner_properties"]["planner"].as<std::string>();
}

void ConfigParser::parseTrajectoryGeneratorProperties(const YAML::Node &config)
{
    trajectoryGeneratorProperties.maxVelocity = config["trajectory_generator_properties"]["max_velocity"].as<double>();
    trajectoryGeneratorProperties.maxAcceleration = config["trajectory_generator_properties"]["max_acceleration"].as<double>();
    trajectoryGeneratorProperties.samplingInterval = config["trajectory_generator_properties"]["sampling_interval"].as<double>();
    trajectoryGeneratorProperties.type = config["trajectory_generator_properties"]["type"].as<std::string>();
    trajectoryGeneratorProperties.maxTime = config["trajectory_generator_properties"]["max_time"].as<double>();
    trajectoryGeneratorProperties.prependTrajTime = config["trajectory_generator_properties"]["prepend_traj_time"].as<double>();
    trajectoryGeneratorProperties.maxTrajDivergence = config["trajectory_generator_properties"]["max_traj_divergence"].as<double>();
    trajectoryGeneratorProperties.samplesFTM = config["trajectory_generator_properties"]["samples_ftm"].as<int>();
}
