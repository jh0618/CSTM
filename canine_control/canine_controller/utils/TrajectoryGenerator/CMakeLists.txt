cmake_minimum_required(VERSION 3.21)
project(TrajectoryGenerator)

find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} STATIC
        CubicTrajectoryGenerator.cpp
        QuadTrajectoryGenerator.cpp
        QuinticTrajectoryGenerator.cpp
        SineTrajectoryGenerator.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        .
)
target_link_libraries(${PROJECT_NAME}
        Eigen3::Eigen
)