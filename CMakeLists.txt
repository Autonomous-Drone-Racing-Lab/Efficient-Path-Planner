cmake_minimum_required(VERSION 3.16)
project(path_planner)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)



# Find yaml-cpp package
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED)

# Specify the directories which contain headers
include_directories(include ${EIGEN3_INCLUDE_DIR})
include_directories(include ${Boost_INCLUDE_DIRS})


# Create a library from the source files in the src/ directory
add_library(${PROJECT_NAME} STATIC
 
)
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)


# Link against yaml-cpp
target_link_libraries (${PROJECT_NAME} Eigen3::Eigen)


add_subdirectory(extern/pybind11)
pybind11_add_module(polynomial_trajectory src/pybind.cpp)

# Link the executable against the library
target_link_libraries(polynomial_trajectory PRIVATE ${PROJECT_NAME})
