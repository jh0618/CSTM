cmake_minimum_required(VERSION 3.21)
project(canine_control)

find_package(Eigen3 REQUIRED)

add_subdirectory(canine_controller)

if (${SIMULATION})
    add_subdirectory(canine_simulation)
    message(STATUS "SIMULATION MODE")

else ()
    add_subdirectory(canine_hardware)
    message(STATUS "HARDWARE MODE")
endif ()