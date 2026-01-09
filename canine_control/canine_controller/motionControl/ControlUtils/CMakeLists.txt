cmake_minimum_required(VERSION 3.21)
project(ControlUtils)

add_library(${PROJECT_NAME} STATIC
        src/CubicReplanning.cpp
        src/CubicSwingLeg.cpp
        src/CubicSwingLegLocal.cpp
        src/Gait.cpp
        src/SwingLeg.cpp
        src/SwingLegLinear.cpp
        src/SwingLegReplanning.cpp
)

add_library(canine_controller::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
        PUBLIC
        include
)
target_link_libraries(${PROJECT_NAME}
        Robot::Utility
        TrajectoryGenerator
        SharedMemory
)