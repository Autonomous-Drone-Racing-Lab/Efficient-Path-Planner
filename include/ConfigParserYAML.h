#pragma once

#include <yaml-cpp/yaml.h>
#include "Types.h"
#include <vector>
#include <map>
#include <string>

class ConfigParser
{
public:
    ConfigParser(const std::string& configPath);

    const std::vector<OBBDescription> &getGateGeometryByTypeId(const int typeId) const;
    const ObjectProperties &getObjectPropertiesByTypeId(const int typeId) const;
    const std::vector<OBBDescription> &getObstacleGeometry() const;

    const WorldProperties &getWorldProperties() const;
    const PathPlannerProperties &getPathPlannerProperties() const;
    const TrajectoryGeneratorProperties &getTrajectoryGeneratorProperties() const;

private:
    YAML::Node config;
    std::map<std::string, std::vector<OBBDescription>> objects;
    std::map<std::string, ObjectProperties> objectProperties;
    WorldProperties worldProperties;
    PathPlannerProperties pathPlannerProperties;
    TrajectoryGeneratorProperties trajectoryGeneratorProperties;

    // parsers
    void parseGeometries(const YAML::Node& config);
    void parseObjectProperties(const YAML::Node& config);
    void parseWorldProperties(const YAML::Node& config);
    void parsePathPlannerProperties(const YAML::Node& config);
    void parseTrajectoryGeneratorProperties(const YAML::Node& config);
};
