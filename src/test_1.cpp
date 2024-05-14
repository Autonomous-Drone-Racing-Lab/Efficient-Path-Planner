#include <iostream>
#include <fstream>
#include "json.h"
#include <Eigen/Dense>
#include "PathPlanner.h"
#include <iostream>
#include "poly_traj/trajectory_generator.h"
#include "OnlineTrajGenerator.h"

using json = nlohmann::json;
using namespace Eigen;

bool parseJsonToMatrices(const std::string &filename, MatrixXd &gatesPosAndType, MatrixXd &obstaclePos, MatrixXd &checkpoints)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return false;
    }

    json j;
    file >> j;

    if (!j.contains("nominal_gates_pos_and_type") || !j.contains("nomial_obstacle_pos") || !j.contains("checkpoints"))
    {
        std::cerr << "JSON does not contain necessary data" << std::endl;
        return false;
    }

    auto parseMatrix = [](const json &jsonArray, MatrixXd &matrix)
    {
        size_t numRows = jsonArray.size();
        size_t numCols = jsonArray.empty() ? 0 : jsonArray[0].size();
        matrix.resize(numRows, numCols);

        for (size_t i = 0; i < numRows; ++i)
        {
            for (size_t j = 0; j < numCols; ++j)
            {
                matrix(i, j) = jsonArray[i][j];
            }
        }
    };

    parseMatrix(j["nominal_gates_pos_and_type"], gatesPosAndType);
    parseMatrix(j["nomial_obstacle_pos"], obstaclePos);
    parseMatrix(j["checkpoints"], checkpoints);

    return true;
}

// Function to write a matrix to a text file
void writePathToFile(const std::vector<Eigen::MatrixXd> &completePath, const std::string &filename)
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
        return;
    }

    // Loop through each row and column of the matrix
    for (const auto &matrix : completePath)
    {
        for (int i = 0; i < matrix.rows(); ++i)
        {
            for (int j = 0; j < matrix.cols(); ++j)
            {
                file << matrix(i, j);
                if (j != matrix.cols() - 1)
                    file << ", "; // Add comma between numbers except the last number
            }
            file << std::endl; // New line for each row
        }
    }

    file.close();
}

int main()
{

    const std::string filename = "../task.json";
    MatrixXd nominalGatesPosAndType;
    MatrixXd nominalObstaclePos;
    MatrixXd checkpoints;
    parseJsonToMatrices(filename, nominalGatesPosAndType, nominalObstaclePos, checkpoints);

    Eigen::Vector3d lowerBound(-2, -2, 0);
    Eigen::Vector3d upperBound(2, 2, 2);

    const std::string configFile = "../config.json";

    const Eigen::Vector3d start = Eigen::Vector3d(1, 1, 0.2);
    const Eigen::Vector3d goal = Eigen::Vector3d(0, -2, 0.5);
    OnlineTrajGenerator onlineTrajGenerator(start, goal, nominalGatesPosAndType, nominalObstaclePos, configFile);

    const double takeoffTime = 0.0;
    onlineTrajGenerator.preComputeTraj(takeoffTime);
    Eigen::MatrixXd trajSample = onlineTrajGenerator.sampleTraj(0.3);

    std::cout << trajSample << std::endl;

    return 0;
}