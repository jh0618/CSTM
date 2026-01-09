cmake_minimum_required(VERSION 3.21)
project(ThreadGenerator)


add_library(${PROJECT_NAME} STATIC
        ThreadGenerator.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        .
)
target_link_libraries(${PROJECT_NAME}

        pthread
)
get_property(result GLOBAL PROPERTY SIMULATION)

if(${SIMULATION})
    message(STATUS "SIMULATION is ON in ThreadGenerator")
else()
    message(STATUS "HARDWARE is ON in ThreadGenerator")
    ADD_COMPILE_DEFINITIONS(EXTERNAL)
endif()