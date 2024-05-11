#include "OBB.h"
#include <Eigen/Dense>

struct OBBDescription
{
    Eigen::Vector3f center;
    Eigen::Vector3f halfSize;
    std::string type;
};

struct ObjectProperties
{
    float height;
};

class Object
{
public:
    Object(Eigen::Vector3f globalCenter, Eigen::Matrix3f globalRotation, std::vector<OBB> obbs);

    static Object createFromDescription(Eigen::Vector3f globalCenter, Eigen::Vector3f globalRotation, std::vector<OBBDescription> &obbDescriptions);
    void translate(Eigen::Vector3f translation);
    void rotateZ(float angle, bool useRadian = true);

private:
    std::vector<OBB> obbs;
    Eigen::Vector3f globalCenter;
    Eigen::Matrix3f globalRotation;
};