cmake_minimum_required(VERSION 3.21)
project(GUI-threadGenerator)


add_library(${PROJECT_NAME} STATIC
        threadGenerator.cpp
)

add_library(GUI::threadGenerator ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
        PUBLIC
        .
)
target_link_libraries(${PROJECT_NAME}
        pthread
)