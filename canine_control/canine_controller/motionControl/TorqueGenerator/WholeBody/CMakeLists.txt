cmake_minimum_required(VERSION 3.21)
project(WBController)
find_package(RBDL REQUIRED)
find_package(RBDL COMPONENTS URDFREADER REQUIRED)

add_library(${PROJECT_NAME} STATIC
        src/WBTorqueGenerator.cpp
)
add_library(canine_controller::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
        PUBLIC
        include
)
target_link_libraries(${PROJECT_NAME}
        qpOASES
        canine_controller::ControlUtils
        ${RBDL_LIBRARY}
        ${RBDL_URDFREADER_LIBRARY}
        SharedMemory
)