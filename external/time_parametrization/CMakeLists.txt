cmake_minimum_required(VERSION 3.2)

# Set the project name
project(time_parametrization)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find packages
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# Create a library from the source files in the src/ directory
add_library(${PROJECT_NAME} STATIC
  src/Path.cpp
  src/Trajectory.cpp
  src/OptimalTimeParametrizer.cpp
)

# Include directories
target_include_directories(${PROJECT_NAME} 
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} 
  PRIVATE 
    Eigen3::Eigen
)