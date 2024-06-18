#include "PathWriter.h"
#include <fstream>
#include <iostream>
#include <filesystem>
#include <Eigen/Dense>

PathWriter::PathWriter(const std::string &folderPath)
    : folderPath(folderPath)
{
    // Check if the path exists
    if (!std::filesystem::exists(folderPath))
    {
        // Create the directory since it does not exist
        std::filesystem::create_directories(folderPath);
        std::cout << "Folder created: " << folderPath << std::endl;
    }
    else
    {
        // Check if the directory is empty
        if (!std::filesystem::is_empty(folderPath))
        {
            // Iterate through each item in the directory
            for (const auto &entry : std::filesystem::directory_iterator(folderPath))
            {
                // Check if it's a file and then remove it
                if (std::filesystem::is_regular_file(entry))
                {
                    std::filesystem::remove(entry);
                    std::cout << "Removed file: " << entry.path() << std::endl;
                }
            }
        }
    }
}

void PathWriter::writePath(const std::vector<Eigen::Vector3d> &path)
{
    const std::string fileName = "path_" + std::to_string(write_count) + ".txt";
    const std::string filePath = folderPath + "/" + fileName;

    std::ofstream file(filePath);
    if (!file.is_open())
    {
        std::cerr << "Failed to open file for writing: " << fileName << std::endl;
        return;
    }

    for (const auto &waypoint : path)
    {
        for (int k = 0; k < 3; k++)
        {
            file << waypoint(k) << " ";
        }
        file << std::endl;
    }

    file.close();
    write_count++;
}

void PathWriter::updateGatePos(const int gateId, const Eigen::VectorXd& gateInfo)
{
   const std::string fileName = "gates.txt";
    const std::string filePath = folderPath + "/" + fileName;

    std::ofstream file(filePath, std::ios_base::app);
    if (!file.is_open())
    {
        std::cerr << "Failed to open file for writing: " << fileName << std::endl;
        return;
    }

    file << "id: " << gateId << " info: " << gateInfo.transpose() << std::endl;
}


void PathWriter::updateObstaclePos(const int obstacleId, const Eigen::VectorXd& pose){
    const std::string fileName = "obstacles.txt";
    const std::string filePath = folderPath + "/" + fileName;

    std::ofstream file(filePath, std::ios_base::app);
    if (!file.is_open())
    {
        std::cerr << "Failed to open file for writing: " << fileName << std::endl;
        return;
    }

    file << "id: " << obstacleId << " info: " << pose.transpose() << std::endl;
}

void PathWriter::writeCheckpoints(const std::vector<Eigen::Vector3d>& checkpoints){
    const std::string fileName = "checkpoints.txt";
    const std::string filePath = folderPath + "/" + fileName;

    std::ofstream file(filePath);
    if (!file.is_open())
    {
        std::cerr << "Failed to open file for writing: " << fileName << std::endl;
        return;
    }

    for (const auto &checkpoint : checkpoints)
    {
        for (int k = 0; k < 3; k++)
        {
            file << checkpoint(k) << " ";
        }
        file << std::endl;
    }

    file.close();
}