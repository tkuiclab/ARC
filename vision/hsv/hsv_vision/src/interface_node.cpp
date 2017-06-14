#include <ros/ros.h>
#include "interface.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "interface_node");
  InterfaceProc ip;
  ros::spin();
  return 0;
}
