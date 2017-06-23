#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Empty.h>
#include <ros/package.h>
//pcl::toROSMsg
#include <pcl/io/pcd_io.h>

//typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointXYZRGBA PT;           //Point Type
typedef pcl::PointCloud<PT> PCT;

PCT cloud2;

PCT LoadPcd(std::string filePath)
{ 
  PCT modelPCD;
  ROS_INFO("Loading PCD....");
  if (pcl::io::loadPCDFile (filePath, modelPCD) < 0) { 
    ROS_ERROR("Error loading Model cloud");
    assert(false);
  }else {
    return modelPCD;
  }
}

int main (int argc, char** argv)
{
  std::string fileName;
  if (argc != 2) {
    ROS_ERROR("rosrun pcl_saver pcd_reader <path/to/fileName>");
    return 0;
  }
  fileName = argv[1];
  
  ros::init(argc, argv, "pcl_reader");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<sensor_msgs::PointCloud2>("/pcd_reader/point_cloud", 1);
  ros::Rate loop_rate(10);
  sensor_msgs::PointCloud2 pc2;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  std::stringstream ss;
  ss << ros::package::getPath("pcl_saver") << "/pattern/" << "apple_00001" << ".pcd";
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (fileName, *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read  pcd file \n");
    return (-1);
  }
  // std::cout << "Loaded "
  //           << cloud->width * cloud->height
  //           << " data points from test_pcd.pcd with the following fields: "
  //           << std::endl;
  // for (size_t i = 0; i < cloud->points.size (); ++i)
  //   std::cout << "    " << cloud->points[i].x
  //             << " "    << cloud->points[i].y
  //             << " "    << cloud->points[i].z << std::endl;

  ROS_INFO("Publish PCD to Topic : /pcd_reader/point_cloud");
  while (ros::ok())
  {
    pcl::toROSMsg (*cloud, pc2);
    pc2.header.frame_id = "camera_rgb_optical_frame";
    chatter_pub.publish(pc2);
    ros::spinOnce();
  }

  return (0);
}
