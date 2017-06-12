#define PI 3.14159265
#include "interface.hpp"
#include "math.h"
#include <time.h>
#define FRAME_COLS 659 //width  x695
#define FRAME_ROWS 493 //height y493

static const std::string OPENCV_WINDOW = "Image window";
using namespace std;
namespace enc = sensor_msgs::image_encodings;

void InterfaceProc::colorcall(const std_msgs::UInt32MultiArray msg)
{
  for(int i=0;i<6;i++) HSVBoxMsg[i]=msg.data[i];
}

InterfaceProc::InterfaceProc()
    :it_(nh) 
{
  ros::NodeHandle n("~");
  //image_sub_ = it_.subscribe("/camera/image_raw", 1, &InterfaceProc::imageCb, this);
  //image_sub_ = it_.subscribe("usb_cam/image_raw", 1, &InterfaceProc::imageCb, this);
  image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &InterfaceProc::imageCb, this);
  image_pub_threshold_ = it_.advertise("/camera/image", 1);//http://localhost:8080/stream?topic=/camera/image webfor /camera/image

  s2 = nh.subscribe("interface/color", 1000, &InterfaceProc::colorcall,this);

  cv::namedWindow(OPENCV_WINDOW, CV_WINDOW_AUTOSIZE);

  frame = new cv::Mat(cv::Size(FRAME_COLS, FRAME_ROWS),CV_8UC3 );
  ColorModels = new cv::Mat(cv::Size(FRAME_COLS, FRAME_ROWS), CV_8UC3);;
  outputframe= new cv::Mat(cv::Size(FRAME_COLS, FRAME_ROWS), CV_8UC3);
} 
InterfaceProc::~InterfaceProc()
{
  delete frame;
  delete ColorModels;
  delete outputframe;
  cv::destroyWindow(OPENCV_WINDOW);
}

void InterfaceProc::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
  }catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  *frame = cv_ptr->image;
  *outputframe = *frame;

  *ColorModels = ColorModel(*frame);
  cv::imshow(OPENCV_WINDOW, *ColorModels);
  outputframe = ColorModels;
  
  sensor_msgs::ImagePtr thresholdMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", *outputframe).toImageMsg();
  image_pub_threshold_.publish(thresholdMsg);
  cv::waitKey(3);
}

cv::Mat InterfaceProc::ColorModel(const cv::Mat iframe)
{
  static cv::Mat oframe(cv::Size(iframe.cols,iframe.rows), CV_8UC3);
  for (int i = 0; i < iframe.rows; i++) {
    for (int j = 0; j < iframe.cols; j++) {
      double B = iframe.data[(i*iframe.cols*3)+(j*3)+0];
      double G = iframe.data[(i*iframe.cols*3)+(j*3)+1];
      double R = iframe.data[(i*iframe.cols*3)+(j*3)+2];
      double H,S,V;
      double Max = (max(R,G)>max(G,B))?max(R,G):max(G,B);   //max(R,G,B);
      double Min = (min(R,G)<min(G,B))?min(R,G):min(G,B);   //min(R,G,B);

      if(Max==Min)Max+=1;
      if(R==Max){H=(G-B)*60/(Max-Min);}
      if(G==Max){H=120+(B-R)*60/(Max-Min);}
      if(B==Max){H=240+(R-G)*60/(Max-Min);}
      if(B==G&&B==R) H=0;
      if(H<0){H=H+360;}
      S=(((Max-Min)*100)/Max);
      if(Max==0)S=0;
      V=Max;
      //  usleep(300);
      hmax = HSVBoxMsg[1];
      hmin = HSVBoxMsg[0];
      smax = HSVBoxMsg[3];
      smin = HSVBoxMsg[2];
      vmax = HSVBoxMsg[5];
      vmin = HSVBoxMsg[4];

      vmin=vmin*2.56;
      vmax=vmax*2.56;

      if(hmax>hmin){
        if((H<=hmax)&&(H>=hmin)&&(S<=smax)&&(S>=smin)&&(V<=vmax)&&(V>=vmin) ){
          oframe.data[(i*oframe.cols*3)+(j*3)+0] = 197;
          oframe.data[(i*oframe.cols*3)+(j*3)+1] = 149;
          oframe.data[(i*oframe.cols*3)+(j*3)+2] = 0 ;
        }else{
          oframe.data[(i*iframe.cols*3)+(j*3)+0] = iframe.data[(i*iframe.cols*3)+(j*3)+0];
          oframe.data[(i*iframe.cols*3)+(j*3)+1] = iframe.data[(i*iframe.cols*3)+(j*3)+1];
          oframe.data[(i*iframe.cols*3)+(j*3)+2] = iframe.data[(i*iframe.cols*3)+(j*3)+2];
        } 
      }else{
        if(((H<=hmax)||(H>=hmin))&&(S<=smax)&&(S>=smin)&&(V<=vmax)&&(V>=vmin) ){
          oframe.data[(i*oframe.cols*3)+(j*3)+0] = 197;
          oframe.data[(i*oframe.cols*3)+(j*3)+1] = 149;
          oframe.data[(i*oframe.cols*3)+(j*3)+2] = 0 ;}else{
          oframe.data[(i*iframe.cols*3)+(j*3)+0] = iframe.data[(i*iframe.cols*3)+(j*3)+0];
          oframe.data[(i*iframe.cols*3)+(j*3)+1] = iframe.data[(i*iframe.cols*3)+(j*3)+1];
          oframe.data[(i*iframe.cols*3)+(j*3)+2] = iframe.data[(i*iframe.cols*3)+(j*3)+2];
        } 
      }
    }
  }
  return oframe;
}
