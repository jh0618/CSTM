cmake_minimum_required(VERSION 3.21)
project(command)

add_library(${PROJECT_NAME} STATIC
        src/Command.cpp
        src/CommandMapping.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        include
)

target_link_libraries(${PROJECT_NAME}
        Robot::Utility
        ControlUtils
)

if(${SIMULATION})
    message(STATUS "SIMULATION is ON in Command")
else()
    message(STATUS "HARDWARE is ON in Command")
    ADD_COMPILE_DEFINITIONS(EXTERNAL)
endif()