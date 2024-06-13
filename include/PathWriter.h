#include <string>
#include <vector>
#include <Eigen/Dense>

class PathWriter {

    public:
    PathWriter(const std::string& folderPath);
    void writePath(const std::vector<Eigen::Vector3d>& path);
    void updateGatePos(const int gateId, const Eigen::VectorXd& pose);
    
    private:
    int write_count = 0;
    const std::string folderPath;

};