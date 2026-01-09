cmake_minimum_required(VERSION 3.21)
project(Utility)

find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} STATIC
        Filter.cpp
        RobotMath.cpp
)

add_library(Robot::Utility ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
        PUBLIC
        .
)

target_link_libraries(${PROJECT_NAME}
        Eigen3::Eigen
        Robot::SharedMemory
)