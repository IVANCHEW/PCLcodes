cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(Tutorials)
find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcd_write_test test.cpp)
target_link_libraries (pcd_write_test ${PCL_LIBRARIES})
