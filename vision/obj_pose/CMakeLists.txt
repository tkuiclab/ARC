cmake_minimum_required(VERSION 2.8.3)
project(obj_pose)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  actionlib
  message_generation
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)
find_package(PCL 1.7 REQUIRED REQUIRED COMPONENTS 
	io
	registration
	segmentation
	visualization
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES ObjectPose.action
)
generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  LIBRARIES actionlib
  CATKIN_DEPENDS actionlib_msgs message_runtime roscpp std_msgs geometry_msgs
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
)


## Specify libraries to link a library or executable target against
add_executable (object_pose_estimator src/object_pose_estimator.cpp)
target_link_libraries (object_pose_estimator ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(object_pose_estimator obj_pose_generate_messages_cpp)
add_dependencies(object_pose_estimator darkflow_detect_generate_messages_cpp)

# add_executable (center_object src/obj_center.cpp)
# target_link_libraries (center_object ${PCL_LIBRARIES} ${catkin_LIBRARIES})
# add_dependencies(center_object obj_pose_generate_messages_cpp)
# add_dependencies(center_object darkflow_detect_generate_messages_cpp)
