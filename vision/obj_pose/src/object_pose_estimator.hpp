#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>

#include <fake_roi/Detect.h>
#include <boost/thread/thread.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <actionlib/server/simple_action_server.h>

#include <obj_pose/ObjectPoseAction.h>

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
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

// Types
typedef pcl::PointXYZ PT;           //Point Type
typedef pcl::PointCloud<PT> PCT;

typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

//#define ShowCloud
#define SaveCloud

enum ProcessingState{
    NADA,
    FOTO,
    CALL_RCNN,
    ALIGMENT,
    POSE_ESTIMATION
}state;

namespace ObjEstAction_namespace
{

class ObjEstAction
{
public:

  ObjEstAction(std::string name) :
    as_(nh_, name, false),
    action_name_(name),
    cloud (new PCT)
    //cloud(new pcl::PCLPointCloud2)
  {
    //foto = false;
    //ya_foto = false;
    pcd_folder = "/pcd_file/";
    path = ros::package::getPath("obj_pose");
    path.append(pcd_folder);
    //ROS_INFO("Get path=%s",path.c_str());
    //SetInputDirectory("/home/iclab-ming/ARC_ws/src/obj_pose/pcd_file/");
    as_.registerGoalCallback(boost::bind(&ObjEstAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&ObjEstAction::preemptCB, this));

    cloud_sub = nh_.subscribe("/camera/depth/points", 10, &ObjEstAction::cloudCB,this);
    
    as_.start();

    roi_client = nh_.serviceClient<fake_roi::Detect>("/detect");
  
    ROS_INFO("obj_pose READY!");
  }

  void goalCB();
  void preemptCB();
  void cloudCB(const sensor_msgs::PointCloud2ConstPtr& input);
  void poseEstimation();
  void aligment();
  void get_roi();

protected:

  //------ROS--------//
  ros::NodeHandle nh_;
  ros::Subscriber cloud_sub;
  ros::ServiceClient roi_client;

  actionlib::SimpleActionServer<obj_pose::ObjectPoseAction> as_;
  std::string action_name_;

  obj_pose::ObjectPoseFeedback feedback_;
  obj_pose::ObjectPoseResult result_;

  //--------Class Usage------//
  int mini_x;
  int mini_y;
  int max_x;
  int max_y;
  
  fake_roi::Detect roi_srv;
  std::string tmp_path;
  std::string tmp_path2;
  std::string obj_name;
  std::string path;
  std::string pcd_folder;

  // pcl::PCLPointCloud2 pcl_pc2;
  // pcl_conversions::toPCL(*input,pcl_pc2);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::fromPCLPointCloud2(pcl_pc2,*cloud);

private:
  PCT::Ptr cloud;
   //pcl::PCLPointCloud2* cloud ;
};
}
