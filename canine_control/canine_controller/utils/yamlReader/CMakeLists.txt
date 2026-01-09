cmake_minimum_required(VERSION 3.21)
project(yamlReader)

find_package(yaml-cpp REQUIRED)
add_compile_definitions(YAML_RSC_DIR="${CMAKE_SOURCE_DIR}/src/robotInit/")

add_library(${PROJECT_NAME} STATIC
        YamlReader.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        .
)
target_link_libraries(${PROJECT_NAME}
        yaml-cpp
)