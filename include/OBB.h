#include <Eigen/Dense>

class OBB{
    public:
    OBB(Eigen::Vector3f center, Eigen::Vector3f halfSize, Eigen::Matrix3f rotation, char* type);

    bool checkCollisionWithRay(Eigen::Vector3f start, Eigen::Vector3f end, float inflateSize);
    bool checkCollisionWithPoint(Eigen::Vector3f point, float inflateSize);

    private:
    Eigen::Vector3f center;
    Eigen::Vector3f halfSize;
    Eigen::Matrix3f rotation;
    char* type;
};