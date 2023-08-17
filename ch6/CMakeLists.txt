cmake_minimum_required(VERSION 3.10)
project(ch6)

# set(CMAKE_BUILD_TYPE Release)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_DEBUG  "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE  "-O3")

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# OpenCV
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(G2O REQUIRED)

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    # "/usr/include/eigen3"
)

add_library(MatJoint SHARED
    ./src/MatJoint.cc
)

add_executable(gaussNewton gaussNewton.cpp)
target_link_libraries(gaussNewton 
    ${OpenCV_LIBS}
)

add_executable(gaussNewtonMat gaussNewtonMatrix.cxx)
target_link_libraries(gaussNewtonMat
    ${OpenCV_LIBS}
    MatJoint
)

add_executable(ceresCurveFitting ceresCurveFitting.cpp)
target_link_libraries(ceresCurveFitting ${OpenCV_LIBS} ${CERES_LIBRARIES})

add_executable(g2oCurveFitting g2oCurveFitting.cpp)
target_link_libraries(g2oCurveFitting ${OpenCV_LIBS} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})
