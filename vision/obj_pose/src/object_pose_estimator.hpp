#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>

//#include <fake_roi/Detect.h>
#include <darkflow_detect/Detected.h>
#include <darkflow_detect/Detect.h>
#include <sensor_msgs/PointCloud2.h>
#include <actionlib/server/simple_action_server.h>
#include <obj_pose/ObjectPoseAction.h>


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



enum ProcessingState{
    NADA,
    FOTO,
    SEGMETATION,
    //CALL_RCNN,
    ALIGMENT,
    POSE_ESTIMATION,
    GET_ONE_ROI,
    GET_CLOSEST,
    UNKNOWN_CLOSEST
}state, next_state;

//for respone to error_code

//#define ERR_DETECT_LESS_MAX               1
#define ERR_CANNOT_GET_CLOSEST            2
#define ERR_CANNOT_CALL_DETECT_SERVICE    3
#define ERR_CALL_DETECT_OVER_TIMES        4
//#define ERR_DETECT_RESPONE_FAIL    5


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

    scence_seg = pcl::console::find_switch (argc, argv, "-ss");
    // pcd_folder = "/";
    // path = ros::package::getPath("obj_pose");
    // path.append(pcd_folder);
    //ROS_INFO("Get path=%s",path.c_str());
    as_.registerGoalCallback(boost::bind(&ObjEstAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&ObjEstAction::preemptCB, this));

    segmented_pub_ =nh_.advertise<sensor_msgs::PointCloud2>("segmented_pointcloud", 1);
    align_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("align_pointcloud", 1);
    cloud_sub = nh_.subscribe("/camera/depth_registered/points", 10, &ObjEstAction::cloudCB,this);
    
    as_.start();

    roi_client = nh_.serviceClient<darkflow_detect::Detect>("/detect");
  
    ROS_INFO("obj_pose READY!");
  }

  void goalCB();
  void preemptCB();
  void cloudCB(const sensor_msgs::PointCloud2ConstPtr& input);
  
  void pub_feedback(std::string msg,int progress);
  void pub_error();
  

  //bool get_roi();
  void poseEstimation();
  void aligment();
  void segmentation();
  void cpc_segmentation();
  void do_ICP();
  
  void get_one_roi();   //need obj_name
  void get_closest();   //need obj_list
  void unknown_closest();

protected:
    

  
  void set_ROI_colud(int mini_x,int mini_y,int max_x, int max_y);

  bool load_amazon_pcd(std::string pcd_filename);
  bool is_obj_in_obj_list(std::string name);
  void print4x4Matrix (const Eigen::Matrix4f & matrix);


  //------ROS--------//
  ros::NodeHandle nh_;
  ros::Publisher segmented_pub_;
  ros::Publisher align_pub_;
  ros::Subscriber cloud_sub;
  ros::ServiceClient roi_client;

  actionlib::SimpleActionServer<obj_pose::ObjectPoseAction> as_;
  std::string action_name_;

  obj_pose::ObjectPoseFeedback feedback_;
  obj_pose::ObjectPoseResult result_;
  geometry_msgs::Twist obj_pose;

  //--------Class Usage------//
  int mini_x;
  int mini_y;
  int max_x;
  int max_y;

  int call_detect_times ;

  bool scence_seg;
  
  darkflow_detect::Detect roi_srv;
  // std::string tmp_path;
  // std::string tmp_path2;
  std::string obj_name;
  std::vector<std::string> obj_list;
  //std::string path;
  //std::string pcd_folder;
  
  sensor_msgs::PointCloud2 seg_msg;



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