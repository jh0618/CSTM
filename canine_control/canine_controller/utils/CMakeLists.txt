cmake_minimum_required(VERSION 3.21)
project(utils)

find_package(Eigen3 REQUIRED)

add_subdirectory(Utility)
add_subdirectory(SharedMemory)
add_subdirectory(ThreadGenerator)
add_subdirectory(DataAnalysis)
add_subdirectory(TrajectoryGenerator)

add_subdirectory(qpOASES)

#add_subdirectory(yamlReader)