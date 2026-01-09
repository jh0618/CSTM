cmake_minimum_required(VERSION 3.21)
project(communication)

find_package(Eigen3 REQUIRED)
find_package(Qt5 COMPONENTS Network Widgets REQUIRED)

add_library(${PROJECT_NAME} STATIC
        src/ConsoleCommunication.cpp
)
target_include_directories(${PROJECT_NAME}
        PUBLIC
        include
)

target_link_libraries(${PROJECT_NAME}
        command
        ThreadGenerator
        SharedMemory
        Qt5::Core
        Qt5::Network
        Qt5::Widgets
        Eigen3::Eigen
)