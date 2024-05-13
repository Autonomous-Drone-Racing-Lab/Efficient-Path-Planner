#include <iostream>
#include <fstream>
#include "json.h"
#include <Eigen/Dense>
#include "PathPlanner.h"
#include <iostream>

using json = nlohmann::json;
using namespace Eigen;

bool parseJsonToMatrices(const std::string &filename, MatrixXf &gatesPosAndType, MatrixXf &obstaclePos, MatrixXf &checkpoints)
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

    auto parseMatrix = [](const json &jsonArray, MatrixXf &matrix)
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
void writePathToFile(const std::vector<Eigen::MatrixXf> &completePath, const std::string &filename)
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
    MatrixXf nominalGatesPosAndType;
    MatrixXf nominalObstaclePos;
    MatrixXf checkpoints;
    parseJsonToMatrices(filename, nominalGatesPosAndType, nominalObstaclePos, checkpoints);

    Eigen::Vector3f lowerBound(-2, -2, 0);
    Eigen::Vector3f upperBound(2, 2, 2);

    const std::string configFile = "../config.json";
    PathPlanner pathPlanner(nominalGatesPosAndType, nominalObstaclePos, lowerBound, upperBound, configFile);

    const float timeLimit = 0.05;
    std::vector<MatrixXf> completePath;
    for (int i = 0; i < checkpoints.rows() - 1; i += 2)
    {
        const Vector3f start(checkpoints(i, 0), checkpoints(i, 1), checkpoints(i, 2));
        const Vector3f goal(checkpoints(i + 1, 0), checkpoints(i + 1, 1), checkpoints(i + 1, 2));

        const float optimalStraightLineCost = (goal - start).norm();
        std::cout << "Optimal straight line cost: " << optimalStraightLineCost << std::endl;

        MatrixXf path = pathPlanner.planPath(start, goal, timeLimit);

        // calculate cost of path
        float pathCost = 0;
        for (int i = 0; i < path.rows() - 1; i++)
        {
            pathCost += (path.row(i + 1) - path.row(i)).norm();
        }

        std::cout << "Path cost: " << pathCost << std::endl;

        completePath.push_back(path);
    }

    std::string outputPath = "../output.txt";
    writePathToFile(completePath, outputPath);

    return 0;
}