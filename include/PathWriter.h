#include <string>
#include <vector>
#include <Eigen/Dense>

/**
 * @brief The PathWriter class is used to write debug logs about trajectories and paths to locations for later plotting.

*/
class PathWriter
{

public:
    PathWriter(const std::string &folderPath);

    /**
     * Writes the given path to a file.
     *
     * @param path The path to be written.
     */
    void writePath(const std::vector<Eigen::Vector3d> &path);

    /**
     * Update the position of a gate identified by its ID with the provided gate information. Updae4d position is written to file
     *
     * @param gateId The ID of the gate to update.
     * @param gateInfo The new position of the gate as a vector.
     */
    void updateGatePos(const int gateId, const Eigen::VectorXd &gateInfo);

    /**
     * Update the position of an obstacle identified by its ID with the provided obstacle information. Updated position is written to file
     *
     * @param obstacleId The ID of the obstacle to update.
     * @param pose The new position of the obstacle as a vector.
     */
    void updateObstaclePos(const int obstacleId, const Eigen::VectorXd &pose);

    /**
     * Writes the given checkpoints to a file.
     *
     * @param checkpoints The checkpoints to be written.
     */
    void writeCheckpoints(const std::vector<Eigen::Vector3d> &checkpoints);

private:
    int write_count = 0;
    const std::string folderPath;
};