cmake_minimum_required(VERSION 3.21)
project(PDTorqueGenerator)

find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} STATIC
        PDTorqueGenerator.cpp
)
add_library(canine_controller::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
        PUBLIC
        .
)
target_link_libraries(${PROJECT_NAME}
        Eigen3::Eigen
        SharedMemory
)