#pragma once

#include <yaml-cpp/yaml.h>
#include "Types.h"
#include <vector>
#include <map>
#include <string>

/**
 * @class ConfigParser
 * @brief Parses and provides access to configuration data.
 *
 * The `ConfigParser` class is responsible for parsing a configuration file and providing access to various properties and geometries defined in the file.
 * It supports retrieving gate geometries, object properties, world properties, path planner properties, and trajectory generator properties.
 */
class ConfigParser
{
public:
    ConfigParser(const std::string &configPath);

    /**
     * Retrieves the gate geometry description for a given type ID.
     *
     * This function returns a constant reference to a vector of OBBDescription objects
     * that represent the gate geometry for the specified type ID.
     *
     * @param typeId The type ID of the gate.
     * @return A constant reference to a vector of OBBDescription objects representing the gate geometry.
     */
    const std::vector<OBBDescription> &getGateGeometryByTypeId(const int typeId) const;

    /**
     * Retrieves the obstacle geometry.
     *
     * @return A constant reference to a vector of OBBDescription objects representing the obstacle geometry.
     */
    const std::vector<OBBDescription> &getObstacleGeometry() const;

    /**
     * Retrieves the object properties for a given type ID.
     *
     * Object in this case means small gate, large gate, ... and properties are something like height
     *
     * @param typeId The type ID of the object.
     * @return The object properties associated with the given type ID.
     */
    const ObjectProperties &getObjectPropertiesByTypeId(const int typeId) const;

    /**
     * Retrieves the world properties.
     *
     * @return The world properties.
     */
    const WorldProperties &getWorldProperties() const;

    /**
     * Retrieves the path planner properties.
     *
     * @return The path planner properties.
     */
    const PathPlannerProperties &getPathPlannerProperties() const;

    /**
     * Retrieves the trajectory generator properties.
     *
     * @return The trajectory generator properties.
     */
    const TrajectoryGeneratorProperties &getTrajectoryGeneratorProperties() const;

private:
    YAML::Node config;
    std::map<std::string, std::vector<OBBDescription>> objects;
    std::map<std::string, ObjectProperties> objectProperties;
    WorldProperties worldProperties;
    PathPlannerProperties pathPlannerProperties;
    TrajectoryGeneratorProperties trajectoryGeneratorProperties;

    // parsers
    void parseGeometries(const YAML::Node &config);
    void parseObjectProperties(const YAML::Node &config);
    void parseWorldProperties(const YAML::Node &config);
    void parsePathPlannerProperties(const YAML::Node &config);
    void parseTrajectoryGeneratorProperties(const YAML::Node &config);
};
