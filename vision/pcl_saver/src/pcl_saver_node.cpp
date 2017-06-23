#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/Image.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <iostream>
#include <iomanip>
#include <ros/package.h>
//pcl::toROSMsg
#include <pcl/io/pcd_io.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointXYZRGB PT;           //Point Type
typedef pcl::PointCloud<PT> PCT;

std::stringstream ssImage;
std::stringstream ssPcd;

class PointCloud
{
public:

  PointCloud()
  {
    ROS_INFO("initialize...");

    cloud = PCT::Ptr (new PCT);

    point_sub_ = nh_.subscribe("/camera/depth_registered/points", 1, &PointCloud::PointCallback, this);
    image_sub_ = nh_.subscribe("/camera/rgb/image_color", 1, &PointCloud::ImageCallback, this);
    // image_pub_ = nh_.advertise<sensor_msgs::Image> ("/pcl_saver/viewer", 30);
    // point_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("/pcl_saver/points_viewer", 30);
    savePcd_ = nh_.subscribe("/pcl_saver/save", 1, &PointCloud::SavePcd, this);

    tmpString_ = "";
    tmpCount_ = 1;

    // cv::namedWindow("view");
  }

  ~PointCloud()
  {
    // cv::destroyWindow("view");
  }

  void ImageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      // cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
      // cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    image = cv_ptr->image;
  }

  void PointCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
  {
    if ((msg->width * msg->height) == 0)
      return; //return if the cloud is not dense!
    try
    {
      // pcl::toROSMsg (*msg, image_); //convert the cloud
      // image_pub_.publish(image_);
      pcl::fromROSMsg(*msg, *cloud);
      //pcl::toROSMsg(cloud, pc2);
      //pc2.header.frame_id = "camera_rgb_optical_frame";
      //point_pub_.publish(pc2);
    }
    catch (std::runtime_error e)
    {
      ROS_ERROR_STREAM("Error in converting cloud to image message: "
                        << e.what());
    }
  }

  void SavePcd(const std_msgs::String msg)
  {
    if (tmpString_ != msg.data) {
      tmpString_ = msg.data;
      tmpCount_ = 1;
    }
    ROS_INFO("Save Pcd and Image : %s_%05d", msg.data.c_str(), tmpCount_);
    // ssPcd << ros::package::getPath("pcl_saver") << "/pattern/pcd_" << std::setfill('0') << std::setw(5) << count << ".pcd";
    // ssImage << ros::package::getPath("pcl_saver") << "/pattern/img_" << std::setfill('0') << std::setw(5) << count << ".jpg";
    std::stringstream s1;
    s1 << ros::package::getPath("pcl_saver") << "/pattern/"
       << tmpString_ << "_" << std::setfill('0') << std::setw(5) << tmpCount_ << ".pcd";
    writer.write<PT> (s1.str(), *cloud, true);

    s1.str("");
    s1.clear();

    s1 << ros::package::getPath("pcl_saver") << "/pattern/"
       << tmpString_ << "_" << std::setfill('0') << std::setw(5) << tmpCount_ << ".jpg";
    cv::imwrite(s1.str(), image);

    s1.str("");
    s1.clear();

    tmpCount_++;
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber point_sub_;
  ros::Subscriber savePcd_;
  ros::Subscriber image_sub_;
  ros::Publisher image_pub_;
  ros::Publisher point_pub_;

  sensor_msgs::Image image_;
  sensor_msgs::PointCloud2 pc2;

  // PCT cloud;
  PCT::Ptr cloud;
  pcl::PCDWriter writer;
  cv::Mat image;

  std::string tmpString_;
  int tmpCount_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_saver");

  PointCloud pc;

  ros::spin();
  return 0;
}
