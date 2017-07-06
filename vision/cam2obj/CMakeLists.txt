cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(cam2obj)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (cam2obj cam2obj.cpp )
target_link_libraries (cam2obj ${PCL_LIBRARIES})
