#include "json.h"
#include "Object.h"

using json = nlohmann::json;

class ConfigParser
{
public:
    ConfigParser(std::string configPath);

    std::vector<OBBDescription> &getGateGeometryByTypeId(int typeId);
    ObjectProperties &getObjectPropertiesByTypeId(int typeId);
    std::vector<OBBDescription> &getObstacleGeometry();

private:
    json config;
    std::map<std::string, std::vector<OBBDescription>> objects;
    std::map<std::string, ObjectProperties> objectProperties;
};