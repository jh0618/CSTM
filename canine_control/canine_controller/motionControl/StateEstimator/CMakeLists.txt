cmake_minimum_required(VERSION 3.21)
project(StateEstimator)

find_package(RBDL REQUIRED)
find_package(RBDL COMPONENTS URDFREADER REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME} STATIC
        src/ContactStateEstimator.cpp
        src/StateEstimator.cpp
        src/DivergenceEstimator.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        include
)
target_link_libraries(${PROJECT_NAME}
        Robot::Utility
        Eigen3::Eigen
        SharedMemory
        ${RBDL_LIBRARY}
        ${RBDL_URDFREADER_LIBRARY}
)
set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/install")
install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include)