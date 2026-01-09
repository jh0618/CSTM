PROJECT(qpOASES CXX)
SET(PACKAGE_NAME "qpOASES")
SET(PACKAGE_VERSION "3.2.2")
SET(PACKAGE_SO_VERSION "3.2")
SET(PACKAGE_DESCRIPTION "An implementation of the online active set strategy")
SET(PACKAGE_AUTHOR "Hans Joachim Ferreau, Andreas Potschka, Christian Kirches et al.")
SET(PACKAGE_MAINTAINER "Hans Joachim Ferreau, Andreas Potschka, Christian Kirches et al.")
SET(PACKAGE_URL "https://projects.coin-or.org/qpOASES")

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/libs)

option(BUILD_SHARED_LIBS "If ON, build shared library instead of static" ON)
option(QPOASES_AVOID_LA_NAMING_CONFLICTS "If ON, avoid to re-defined symbols that conflict with Blas/Lapack provided functions." OFF)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)



FILE(GLOB SRC src/*.cpp)
add_library(qpOASES STATIC ${SRC})
target_include_directories(qpOASES
        PUBLIC
        ${PROJECT_SOURCE_DIR}/include
)