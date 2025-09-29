cmake_minimum_required(VERSION 3.0.2)
project(my_fast_calib)
set(CMAKE_CXX_FLAGS "-g3 -O -Wall")
set(CMAKE_CXX_STANDARD 20)

find_package(gflags REQUIRED)
find_package(glog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)

set(CMAKE_BUILD_TYPE Release)

if (CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
endif()

add_executable(my_fast_calib src/main.cpp src/print_utils.cpp src/io_utils.cpp)
target_link_libraries(my_fast_calib gflags yaml-cpp glog ${PCL_LIBRARIES} ${OpenCV_LIBS})