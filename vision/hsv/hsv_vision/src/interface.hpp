#include <cstdio>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core/core.hpp"
#include <vector>
#include <deque>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <ros/package.h>
#include "std_msgs/UInt32MultiArray.h"

using namespace cv;
using namespace std;
typedef unsigned char BYTE;
namespace enc = sensor_msgs::image_encodings;

class InterfaceProc
{
private:
  ros::NodeHandle nh;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_threshold_;

  ros::Subscriber s2;

  cv::Mat *frame;
  cv::Mat *centermats;

  cv::Mat *ColorModels;
  cv::Mat *outputframe;

  int hmax,hmin,smax,smin,vmax,vmin;

public:
  InterfaceProc();
  ~InterfaceProc();
  int HSVBoxMsg[6];

  void imageCb(const sensor_msgs::ImageConstPtr&);
  void colorcall(const std_msgs::UInt32MultiArray);

  cv::Mat ColorModel(const cv::Mat iframe);
};
