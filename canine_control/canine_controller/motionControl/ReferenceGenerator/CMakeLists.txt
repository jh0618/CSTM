cmake_minimum_required(VERSION 3.21)
project(QuadMotionGenerator)
##
add_library(${PROJECT_NAME} STATIC
        src/BodyTrajectoryGen.cpp
        src/HomeLegPDMotionGen.cpp
        src/LegTrajectoryGen.cpp
        src/QuadMotionGenerator.cpp
)

add_library(Controller::QuadMotionGenerator ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
        PUBLIC
        include
)

target_link_libraries(${PROJECT_NAME}
        Robot::Utility
        TrajectoryGenerator
        SharedMemory
        ControlUtils
)
