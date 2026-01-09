cmake_minimum_required(VERSION 3.21)
project(ControlMain)

add_library(${PROJECT_NAME} STATIC
        src/LowControlMain.cpp
        src/HighControlMain.cpp
)

add_library(canine_controller::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
        PUBLIC
        include
)

target_link_libraries(${PROJECT_NAME}
        Robot::Utility
        ControlUtils
        WBController
        PDTorqueGenerator
        SharedMemory
        Controller::QuadMotionGenerator
)
