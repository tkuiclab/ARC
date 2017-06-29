#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Empty.h>
#include <ros/package.h>
#include <iostream>
// pcl::toROSMsg
#include <pcl/io/pcd_io.h>
// ros image
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

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
    ROS_ERROR("e.g. rosrun pcl_saver pcd_reader /home/user/PcdAndImg_fileName");
    return 0;
  }
  if (std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help") {
    std::cout<<"Publish pcd and jpg file to topic"<<std::endl<<std::endl;
    std::cout<<"rosrun pcl_saver pcd_reader <path/to/fileName>"<<std::endl;
    std::cout<<"e.g. rosrun pcl_saver pcd_reader /home/user/PcdAndImg_fileName"<<std::endl;
    return 0;
  }
  fileName = argv[1];
  
  ros::init(argc, argv, "pcl_reader");
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);
  image_transport::Publisher image_pub = it.advertise("/camera/rgb/image_color", 1);
  ros::Publisher chatter_pub = n.advertise<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1);
  ros::Rate loop_rate(10);
  sensor_msgs::PointCloud2 pc2;
  sensor_msgs::ImagePtr imgMsg;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  // std::stringstream ss;
  // ss << ros::package::getPath("pcl_saver") << "/pattern/" << "apple_00001" << ".pcd";
  std::stringstream ssPcd, ssImg;
  ssPcd << fileName << ".pcd";
  ssImg << fileName << ".jpg";

  cv::Mat image;
  image = cv::imread(ssImg.str(), CV_LOAD_IMAGE_COLOR);   // Read the file
  if (! image.data ) {
    std::cout <<  "Could not open or find the image" << std::endl ;
    return (-1);
  }

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (ssPcd.str(), *cloud) == -1) //* load the file
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

  ROS_INFO("Publish PCD to Topic : /camera/rgb/image_color");
  ROS_INFO("Publish Img to Topic : /camera/depth_registered/points");
  while (ros::ok())
  {
    imgMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    image_pub.publish(imgMsg);

    pcl::toROSMsg (*cloud, pc2);
    pc2.header.frame_id = "camera_rgb_optical_frame";
    chatter_pub.publish(pc2);
    ros::spinOnce();
  }
  ssPcd.str(""); ssPcd.clear();
  ssImg.str(""); ssImg.clear();

  return (0);
}
