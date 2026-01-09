cmake_minimum_required(VERSION 3.21)
project(GUI-SharedMemory)

find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} STATIC
        SharedMemory.cpp
)

add_library(GUI::SharedMemory ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${PROJECT_NAME}
        Eigen3::Eigen
)