cmake_minimum_required(VERSION 3.21)
project(canine)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_BUILD_TYPE Release)

##
set(SIMULATION OFF CACHE INTERNAL "Simulation mode")
set(CONSOLE ON)
##
add_subdirectory(canine_control)
if(CONSOLE)
    add_subdirectory(canine_console)
endif()
