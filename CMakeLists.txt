cmake_minimum_required(VERSION 3.22)
project(path_planner)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find required packages
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED)
find_package(ompl REQUIRED)

# Set the include directories
add_subdirectory(external/poly_traj)


# Create a library from the source files in the src/ directory
add_library(${PROJECT_NAME} STATIC
    src/ConfigParser.cpp
    src/PathPlanner.cpp
    src/OBB.cpp
    src/Object.cpp
    src/World.cpp
    src/MotionValidator.cpp
    src/StateValidator.cpp
    src/OnlineTrajGenerator.cpp
    src/PathWriter.cpp
)
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)


# Include directories scoped to the target
target_include_directories(${PROJECT_NAME} PUBLIC 
    include 
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
)


# Link the libraries to the target
target_link_libraries(${PROJECT_NAME} 
    Eigen3::Eigen
    ${Boost_LIBRARIES}
    ${OMPL_LIBRARIES}
    poly_traj
)

#add_subdirectory(extern/pybind11)
#pybind11_add_module(polynomial_trajectory src/pybind.cpp)

# Link the executable against the library
#target_link_libraries(polynomial_trajectory PRIVATE ${PROJECT_NAME})

# Compile test_1.cpp to an executable
# add_executable(test_1 src/test_1.cpp)
# target_link_libraries(test_1 ${PROJECT_NAME})

# # Compile test_2.cpp to an executable
add_executable(test_2 src/test_objects.cpp)
target_link_libraries(test_2 ${PROJECT_NAME})

add_subdirectory(external/pybind11)
pybind11_add_module(online_traj_planner src/pybind.cpp)

# Link the executable against the library
target_link_libraries(online_traj_planner PRIVATE ${PROJECT_NAME})

