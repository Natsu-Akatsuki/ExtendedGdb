cmake_minimum_required(VERSION 3.10)
project(test)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_BUILD_TYPE DEBUG)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wdeprecated-declarations -Wall -Wwrite-strings")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL QUIET)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

add_executable(test test.cpp)
target_link_libraries(test ${OpenCV_LIBS})
target_link_libraries(test ${PCL_LIBRARIES})