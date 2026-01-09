cmake_minimum_required(VERSION 3.21)
project(canine_simulation)

add_compile_definitions(URDF_RSC_DIR="${CMAKE_SOURCE_DIR}/canine_urdf/canineV5/urdf/")

add_subdirectory(lib/SimulVisualizer)

add_executable(canine_simulation src/simulMain.cpp)

target_include_directories(canine_simulation
        PUBLIC
        include
)
target_link_libraries(canine_simulation
        SimulVisualizer
        canine_controller
)
target_compile_definitions(canine_simulation
        PRIVATE
        URDF_RSC_DIR
)