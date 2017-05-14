# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(arc_ui)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs genmsg)

## Declare ROS messages and services
add_service_files(FILES UI_Server.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

## Declare a catkin package
catkin_package()
