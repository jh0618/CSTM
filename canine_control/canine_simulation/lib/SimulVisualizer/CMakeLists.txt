cmake_minimum_required(VERSION 3.21)
project(SimulVisualizer)
set(RAISIM_PATH "$ENV{HOME}/Library/raisimLib")
set(RAISIM_LIB_PATH "${RAISIM_PATH}/raisim/linux")

if(${APPLE})
    message("MAC OS X")
    set(RAISIM_PATH "$ENV{HOME}/Documents/git/raisimLib/raisim/m1")
endif()
ADD_COMPILE_DEFINITIONS (RAISIM_RSC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/rsc/")
ADD_COMPILE_DEFINITIONS (RAISIM_SIM_DIR="${RAISIM_PATH}/raisimUnityOpengl/linux/")

find_package(Eigen3 REQUIRED)
add_library(${PROJECT_NAME} SHARED src/SimulVisualizer.cpp)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        ${RAISIM_LIB_PATH}/include
        include
)

target_link_directories(${PROJECT_NAME}
        PUBLIC
        ${RAISIM_LIB_PATH}/lib
)


target_link_libraries(${PROJECT_NAME}
        canine_controller
        raisim

)
