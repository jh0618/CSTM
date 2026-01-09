cmake_minimum_required(VERSION 3.21)
project(DataAnalysis)

ADD_COMPILE_DEFINITIONS (LOG_DIR="${CMAKE_SOURCE_DIR}/log/")

add_library(${PROJECT_NAME} STATIC
        DataAnalysis.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        .
)
target_link_libraries(${PROJECT_NAME}
        Robot::SharedMemory
        stdc++fs
)