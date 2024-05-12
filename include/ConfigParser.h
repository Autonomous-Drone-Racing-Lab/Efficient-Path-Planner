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

private:
    json config;
    std::map<std::string, std::vector<OBBDescription>> objects;
    std::map<std::string, ObjectProperties> objectProperties;
};