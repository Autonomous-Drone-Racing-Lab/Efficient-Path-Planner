#pragma once

#include "json.h"
#include "Types.h"
#include <vector>
#include <map>
#include <string>

using json = nlohmann::json;
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
    json config;
    std::map<std::string, std::vector<OBBDescription>> objects;
    std::map<std::string, ObjectProperties> objectProperties;
    WorldProperties worldProperties;
    PathPlannerProperties pathPlannerProperties;
    TrajectoryGeneratorProperties trajectoryGeneratorProperties;

    // parsers
    void parseGeometries(json &config);
    void parseObjectProperties(json &config);
    void parseWorldProperties(json &config);
    void parsePathPlannerProperties(json &config);
    void parseTrajectoryGeneratorProperties(json &config);
};