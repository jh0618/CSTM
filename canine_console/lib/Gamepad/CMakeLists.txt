cmake_minimum_required(VERSION 3.21)
project(Gamepad)

add_library(${PROJECT_NAME}
        Gamepad.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)