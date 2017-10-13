#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>

//#include <fake_roi/Detect.h>
#include <darkflow_detect/Detected.h>
#include <darkflow_detect/Detect.h>

#include <sift/sift.h>
#include <sensor_msgs/PointCloud2.h>
#include <actionlib/server/simple_action_server.h>
#include <obj_pose/ObjectPoseAction.h>
#include <manipulator_h_base_module_msgs/GetKinematicsPose.h>

#include <boost/thread/thread.hpp>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl_ros/transforms.h>
#include <pcl/console/parse.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
//#include <pcl/surface/impl/mls.hpp>
#include <pcl/surface/mls.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "object_pose_auxiliary.hpp"
#include <tf/transform_broadcaster.h>
#include "tf_conversions/tf_eigen.h"
#include "manipulator_h_base_module_msgs/IK_Cmd.h"



enum ProcessingState{
    NADA,
    FOTO,
    SEGMETATION,
    //CALL_RCNN,
    ALIGMENT,
    POSE_ESTIMATION,
    GET_ONE_ROI,
    GET_CLOSEST,
    GET_CLOSEST_SIFT,
    UNKNOWN_CLOSEST
}state, next_state;

//for respone to error_code

//#define ERR_DETECT_LESS_MAX               1
#define ERR_CANNOT_GET_CLOSEST            2
#define ERR_CANNOT_CALL_DETECT_SERVICE    3
#define ERR_CALL_DETECT_OVER_TIMES        4
//#define ERR_DETECT_RESPONE_FAIL    5
#define ERR_0_CLOUD        6

namespace ObjEstAction_namespace
{

class ObjEstAction
{
public:

  ObjEstAction(int argc, char **argv, std::string name) :
    as_(nh_, name, false),
    action_name_(name),
    scene_cloud(new PCT),
    ROI_cloud(new PCT),
    limit_z_min(0.3)
  {
    g_argc = argc;
    g_argv = argv;

    //scence_seg = pcl::console::find_switch (argc, argv, "-ss");
    scence_seg = true;
    // pcd_folder = "/";
    // path = ros::package::getPath("obj_pose");
    // path.append(pcd_folder);
    //ROS_INFO("Get path=%s",path.c_str());
    as_.registerGoalCallback(boost::bind(&ObjEstAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&ObjEstAction::preemptCB, this));

    segmented_pub_ =nh_.advertise<sensor_msgs::PointCloud2>("segmented_pointcloud", 1);
    align_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("align_pointcloud", 1);
    cloud_sub = nh_.subscribe("/camera/depth_registered/points", 10, &ObjEstAction::cloudCB,this);
    arm_fb_sub = nh_.subscribe("/robotis/fk_fb", 1, &ObjEstAction::arm_fbCB, this);
    as_.start();

    roi_client = nh_.serviceClient<darkflow_detect::Detect>("/detect");
    sift_roi_client = nh_.serviceClient<sift::sift>("/sift_server");

    ROS_INFO("obj_pose READY!");
  }

  void goalCB();
  void preemptCB();
  void cloudCB(const sensor_msgs::PointCloud2ConstPtr& input);

  void arm_fbCB(const manipulator_h_base_module_msgs::IK_Cmd::ConstPtr& eef_pose_msg){
    eef_pose.linear.x = eef_pose_msg->data[0];
    eef_pose.linear.y = eef_pose_msg->data[1];
    eef_pose.linear.z = eef_pose_msg->data[2];
    eef_pose.angular.x = eef_pose_msg->data[5];
    eef_pose.angular.y = eef_pose_msg->data[3];
    eef_pose.angular.z = eef_pose_msg->data[4];

    double tmp_roll= eef_pose.angular.x/3.14*180;
    double tmp_pitch=eef_pose.angular.y/3.14*180;
    double tmp_yaw = eef_pose.angular.z/3.14*180;

    tmp_eef = euler2Quaternion(180,90,0);
    q = tf::Quaternion(tmp_eef.x(),tmp_eef.y(),tmp_eef.z(), tmp_eef.w());
    transform.setOrigin( tf::Vector3(0,0,0));
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "ros_world", "robot_arm_base"));

    tmp_eef = euler2Quaternion(0,0,0);
    q = tf::Quaternion(tmp_eef.x(),tmp_eef.y(),tmp_eef.z(), tmp_eef.w()); 
    transform.setOrigin( tf::Vector3(eef_pose.linear.z, eef_pose.linear.y, eef_pose.linear.x));
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "robot_arm_base", "tcp_tmp_y"));

    tmp_eef = euler2Quaternion(0,tmp_pitch,0);
    q = tf::Quaternion(tmp_eef.x(),tmp_eef.y(),tmp_eef.z(), tmp_eef.w()); 
    transform.setOrigin( tf::Vector3(0,0,0));
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "tcp_tmp_y", "tcp_tmp_z"));


    tmp_eef = euler2Quaternion(0,0,tmp_yaw);
    q = tf::Quaternion(tmp_eef.x(),tmp_eef.y(),tmp_eef.z(), tmp_eef.w()); 
    transform.setOrigin( tf::Vector3(0,0,0));
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "tcp_tmp_z", "tcp_tmp_x"));
    
    tmp_eef = euler2Quaternion(tmp_roll,0,0);
    q = tf::Quaternion(tmp_eef.x(),tmp_eef.y(),tmp_eef.z(), tmp_eef.w()); 
    transform.setOrigin( tf::Vector3(0,0,0));
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "tcp_tmp_x", "tcp"));

    tmp_eef = euler2Quaternion(180,0,0);
    q = tf::Quaternion(tmp_eef.x(),tmp_eef.y(),tmp_eef.z(), tmp_eef.w());
    transform.setOrigin( tf::Vector3(0.07, 0.0, 0.09));
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "tcp", "camera_link"));

    // tmp_eef = euler2Quaternion(-90,90,0);
    // q = tf::Quaternion(tmp_eef.x(),tmp_eef.y(),tmp_eef.z(), tmp_eef.w());
    // transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0));
    // transform.setRotation(q);
    // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_link", "camera_rgb_optical_frame"));
  }
  
  void pub_feedback(std::string msg,int progress);
  void pub_error(const char *err_msg);
  

  //bool get_roi();
  void poseEstimation();
  void aligment();
  void segmentation();
  void cpc_segmentation();
  void do_ICP();
  
  void get_one_roi();   //need obj_name
  void get_closest();   //need obj_list
  void get_closest_SIFT();
  void unknown_closest();

protected:
    

  
  void set_ROI_cloud(int mini_x,int mini_y,int max_x, int max_y);

  bool load_amazon_pcd(std::string pcd_filename);
  bool is_obj_in_obj_list(std::string name);
  void print4x4Matrix (Eigen::Matrix4f & matrix, Eigen::Vector4f centroid);
  // Eigen::Matrix3d euler2Quaternion( double roll, double pitch, double yaw);
  Eigen::Quaterniond euler2Quaternion( const double roll, const double pitch, const double yaw);
  
  void check_0_cloud_error_sub(const char *cloud_name);

  bool check_0_cloud(PCT::Ptr i_cloud, const char *cloud_name);
  //------ROS--------//
  ros::NodeHandle nh_;
  ros::Publisher segmented_pub_;
  ros::Publisher align_pub_;
  ros::Subscriber cloud_sub;
  ros::Subscriber arm_fb_sub;
  ros::ServiceClient roi_client;
  ros::ServiceClient sift_roi_client;

  actionlib::SimpleActionServer<obj_pose::ObjectPoseAction> as_;
  std::string action_name_;

  obj_pose::ObjectPoseFeedback feedback_;
  obj_pose::ObjectPoseResult result_;
  geometry_msgs::Twist obj_pose;
  geometry_msgs::Twist eef_pose;
  //--------Class Usage------//
  int mini_x;
  int mini_y;
  int max_x;
  int max_y;

  int call_detect_times ;

  bool scence_seg;
  
  darkflow_detect::Detect roi_srv;
  manipulator_h_base_module_msgs::GetKinematicsPose arm_fb_srv;
  sift::sift sift_roi_srv;

  // std::string tmp_path;
  // std::string tmp_path2;
  std::string obj_name;
  std::vector<std::string> obj_list;
  //std::string path;
  //std::string pcd_folder;
  
  sensor_msgs::PointCloud2 seg_msg;

  //--------- Pub TF ----------//
  tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  Eigen::Quaterniond tmp_eef; 

private:
  PT min_p, max_p;
  PCT::Ptr cloud;
  //PCT::Ptr my_ROICloud;
  PCT::Ptr scene_cloud ;
  PCT::Ptr ROI_cloud;

  //limit for passthrough
  float limit_x_min, limit_x_max, 
        limit_y_min, limit_y_max, 
        limit_z_min, limit_z_max;


  int g_argc;
  char** g_argv;

  pcl::PointCloud<pcl::PointXYZ>::Ptr model_PCD;
  PCT::CloudVectorType clusters;
  pcl::PointCloud<pcl::PointXYZ>::Ptr Max_cluster;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster;
  Eigen::Matrix4f transformation_matrix;

  int error_code;


  int Unknown_Closest_num; // 0 -> most closest , 1 -> second closest
};
}