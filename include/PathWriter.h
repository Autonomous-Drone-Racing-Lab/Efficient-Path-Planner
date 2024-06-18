#include <string>
#include <vector>
#include <Eigen/Dense>

class PathWriter {

    public:
    PathWriter(const std::string& folderPath);
    void writePath(const std::vector<Eigen::Vector3d>& path);
    void updateGatePos(const int gateId, const Eigen::VectorXd& gateInfo);
    void updateObstaclePos(const int obstacleId, const Eigen::VectorXd& pose);
    void writeCheckpoints(const std::vector<Eigen::Vector3d>& checkpoints);
    
    private:
    int write_count = 0;
    const std::string folderPath;

};