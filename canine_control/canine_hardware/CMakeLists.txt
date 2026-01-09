cmake_minimum_required(VERSION 3.21)
project(canine_hardware)
find_package(Qt5 COMPONENTS Core REQUIRED)
add_executable(${PROJECT_NAME} src/canine_hardware_main.cpp
        src/canine_imu.cpp
        src/CanMotorHL.cpp
        src/CanMotorFL.cpp
        src/CanMotorHR.cpp
        src/CanMotorFR.cpp
        src/canine_motor.cpp
        include/canine_hardware/canine_hardware.hpp
)

add_compile_definitions(URDF_RSC_DIR="${CMAKE_SOURCE_DIR}/canine_urdf/canineV5/urdf/")

target_link_libraries(${PROJECT_NAME}
        canine_controller
        Qt5::Core
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        include
)

target_compile_definitions(${PROJECT_NAME}
        PRIVATE
        URDF_RSC_DIR
)

set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/install" CACHE PATH "Default install path" FORCE )
install(TARGETS ${PROJECT_NAME} DESTINATION lib)