#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointXYZRGB PT;           //Point Type
typedef pcl::PointCloud<PT> PCT;

PCT::Ptr cloud;

void callback(const PointCloud::ConstPtr& msg)
{
  printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
    printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcd_saver");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1, callback);
  ros::spin();
}
