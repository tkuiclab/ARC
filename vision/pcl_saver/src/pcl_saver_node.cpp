#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/Image.h>
#include <std_msgs/Empty.h>
//pcl::toROSMsg
#include <pcl/io/pcd_io.h>

//typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointXYZRGB PT;           //Point Type
typedef pcl::PointCloud<PT> PCT;

class PointCloud
{
public:

  PointCloud()
  {
    ROS_INFO("initialize...");
    sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &PointCloud::callback, this);
    image_pub_ = nh_.advertise<sensor_msgs::Image> ("/pcl_saver/viewer", 30);
    savePcd_ = nh_.subscribe("/pcl_saver/save", 1, &PointCloud::SavePcd, this);
  }

  //void callback(const PointCloud::ConstPtr& msg)
  void callback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
  //  printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  //  BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
  //    printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
    if ((msg->width * msg->height) == 0)
      return; //return if the cloud is not dense!
    try
    {
      pcl::toROSMsg (*msg, image_); //convert the cloud
    }
    catch (std::runtime_error e)
    {
      ROS_ERROR_STREAM("Error in converting cloud to image message: "
                        << e.what());
    }
    image_pub_.publish(image_);
    pcl::fromROSMsg(*msg, cloud);
  }

  void SavePcd(const std_msgs::Empty msg)
  {
    ROS_INFO("Save Pcd");
    writer.write<PT> ("/home/neet/Documents/test.pcd", cloud, false);
  }

private:
  ros::NodeHandle nh_;
  sensor_msgs::Image image_;
  ros::Subscriber sub_;
  ros::Publisher image_pub_;
  ros::Subscriber savePcd_;

  PCT cloud;
  pcl::PCDWriter writer;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_saver");
  PointCloud pc;
//  ros::NodeHandle nh;
//  ros::Subscriber sub = nh.subscribe("/camera/depth_registered/points", 1, callback);
//  ros::Publisher image_pub = nh.advertise<sensor_msgs::Image> ("/pcl_saver/viewer", 30);;
  ros::spin();
  return 0;
}
