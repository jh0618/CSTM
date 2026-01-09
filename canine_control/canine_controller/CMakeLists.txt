cmake_minimum_required(VERSION 3.12)
project(canine_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_subdirectory(motionControl)
add_subdirectory(utils)
add_subdirectory(command)
add_subdirectory(communication)

add_library(${PROJECT_NAME} STATIC ControlMain.cpp)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        .
)

target_link_libraries(${PROJECT_NAME}
        PRIVATE
        ControlMain
        StateEstimator
        command
        DataAnalysis
        communication
        PUBLIC
        Robot::Utility
        ThreadGenerator
        SharedMemory
)
if(${SIMULATION})
    message(STATUS "SIMULATION is ON in ControlMain")
else()
    message(STATUS "HARDWARE is ON in ControlMain")
    ADD_COMPILE_DEFINITIONS(EXTERNAL)
endif ()

set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/install")
install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(FILES ControlMain.hpp DESTINATION include)