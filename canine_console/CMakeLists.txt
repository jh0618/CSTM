cmake_minimum_required(VERSION 3.21)
project(canine_console)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_BUILD_TYPE Release)

find_package(Qt5 CONFIG REQUIRED COMPONENTS Widgets)
find_package(Qt5 COMPONENTS Widgets Network REQUIRED)
find_package(Eigen3 REQUIRED)

add_compile_definitions(GUI_RSC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/rsc/")

add_subdirectory(lib)

add_executable(${PROJECT_NAME}
        src/main.cpp
        src/JoystickCommand.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        include
)

target_link_libraries(${PROJECT_NAME}
        GUI
        GUI::threadGenerator
        GUI::SharedMemory
        GUI::descriptions
        GUI::Communication
        Gamepad
)

add_library(descriptions INTERFACE)

add_library(GUI::descriptions ALIAS descriptions)

target_include_directories(descriptions
        INTERFACE
        include
)

target_link_libraries(descriptions
        INTERFACE
        Eigen3::Eigen
)