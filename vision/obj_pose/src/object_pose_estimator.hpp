#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>

//#include <fake_roi/Detect.h>
#include <darkflow_detect/Detect.h>
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
#include <pcl/common/common.h>
#include <pcl_ros/transforms.h>
#include <pcl/console/parse.h>
#include <pcl_conversions/pcl_conversions.h>

// Types
typedef pcl::PointXYZRGBA PT;           //Point Type
typedef pcl::PointCloud<PT> PCT;

typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

//#define ShowCloud
#define SaveCloud
std::string AmazonModelList[40] =
{
  "Avery_Binder",
  "Ballons",
  "Band_Aid_Tape",
  "Bath_Sponge",
  "Black_Fashion_Gloves",
  "Burls_Bees_Baby_Wipes",
  "Colgate_Toothbrush_4PK",
  "Composition_Book",
  "Crayons",
  "Duct_Tape",
  "Epsom_Salts",
  "Expo_Eraser",
  "Fiskars_Scissors",
  "Flashlight",
  "Glue_Sticks",
  "Hand_Weight",
  "Hanes_Socks",
  "Hinged_Ruled_Index_Cards",
  "Ics_Cube_Tray",
  "Irish_Spring_Soap",
  "Laugh_Out_Loud_Jokes",
  "Marbles",
  "Measuring_Spoons",
  "Mesh_Cup",
  "Mouse_Traps",
  "Pie_Plates",
  "Plastic_Wine_Glass",
  "Poland_Spring_Water",
  "Reynolds_Wrap",
  "Robots_DVD",
  "Robots_Everywhere",
  "Scotch_Sponges",
  "Speed_Stick",
  "Table_Cloth",
  "Tennis_Ball_Container",
  "Ticonderoga_Pencils",
  "Tissue_Box",
  "Toilet_Brush",
  "White_Facecloth",
  "Windex"
};

std::string LabelList[40] =
{
  "avery1BinderWhite",
  "bagOfBalloons",
  "johnsonjohnsonPaperTape",
  "theBatheryDelicateBathSponge",
  "knitGlovesBlack",
  "burtsBeesBabyWipes",
  "colgateToothbrushs",
  "greenCompositionBook",
  "crayolaCrayons24",
  "scotchClothDuctTape",
  "drtealsEpsomSalts",  
  "expoEraser",  
  "fiskarScissors",  
  "arFlashlihgts",
  "elmersGlueSticks6Ct",  
  "neopreneWeightPink",
  "hanesWhitteSocks",  
  "spiralIndexCards",
  "steriliteIceCubeTray",
  "irishSpring",
  "laughOutLoundJokesForKids",
  "miniMarblesClearLustre",
  "targetBrandMeasuringSpoons",
  "meshPencilCup",
  "tomcatMousetraps",
  "reynoldsPiePans2ct",
  "plasticWineGlasses",
  "polandSpringsWaterBottle",
  "reynoldsWrap85Sqft",
  "dvdRobots",
  "robotsEverywhere",
  "scotchSponges",
  "speedStick2Pack",
  "tableCover",
  "wilsonTennisBalls",
  "ticonderogaPencils",
  "kleenexCoolTouchTissues",
  "cloroxToiletBrush",
  "whiteFaceCloth",
  "windexSprayBottle23oz"
};

enum ProcessingState{
    NADA,
    FOTO,
    SEGMETATION,
    CALL_RCNN,
    ALIGMENT,
    POSE_ESTIMATION
}state;

namespace ObjEstAction_namespace
{

class ObjEstAction
{
public:

  ObjEstAction(int argc, char **argv, std::string name) :
    as_(nh_, name, false),
    action_name_(name),
    cloud (new PCT)
  {
    scence_seg = pcl::console::find_switch (argc, argv, "-ss");
    pcd_folder = "/";
    path = ros::package::getPath("obj_pose");
    path.append(pcd_folder);
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
  void poseEstimation();
  void aligment();
  void get_roi();
  void segmentation();
  void cpc_segmentation();
  void do_ICP();
  bool load_pcd(std::string pcd_filename);
  void set_feedback(std::string msg,int progress);
  void print4x4Matrix (const Eigen::Matrix4f & matrix);

protected:

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

  bool scence_seg;
  
  darkflow_detect::Detect roi_srv;
  std::string tmp_path;
  std::string tmp_path2;
  std::string obj_name;
  std::string path;
  std::string pcd_folder;
  
  sensor_msgs::PointCloud2 seg_msg;
private:
  PT min_p, max_p;
  PCT::Ptr cloud;
  PCT::Ptr my_ROICloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr model_PCD;
  PCT::CloudVectorType clusters;
  pcl::PointCloud<pcl::PointXYZ>::Ptr Max_cluster;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster;
  Eigen::Matrix4f transformation_matrix;
};
}