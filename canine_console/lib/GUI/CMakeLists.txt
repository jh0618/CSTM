cmake_minimum_required(VERSION 3.21)

project(GUI VERSION 0.1)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)


add_library(${PROJECT_NAME} STATIC
        mainwindow.cpp
        qcustomplot.cpp
        mainwindow.ui
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(GUI
        Qt${QT_VERSION_MAJOR}::Widgets
        GUI::SharedMemory
)

if (QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(NewGUI)
endif ()
