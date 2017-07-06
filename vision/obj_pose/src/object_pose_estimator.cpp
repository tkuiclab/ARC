
#include "object_pose_estimator.hpp"
//#include "seg_plane_cam2obj.hpp"
//#include "cpc_segmentation.hpp"
#include "cam2obj_ros.hpp"
#include "ICP_alignment.hpp"


using namespace ObjEstAction_namespace;

void ObjEstAction::goalCB()
{
  state = FOTO;
  obj_name = as_.acceptNewGoal()->object_name;
  ROS_INFO("Action calling! Goal=%s",obj_name.c_str());    
  std::cout << "-------------" << obj_name << " ----------" << std::endl;  
  
}

void ObjEstAction::preemptCB()
{
  ROS_INFO("%s: Preempted", action_name_.c_str());
  as_.setPreempted();
}

void ObjEstAction::cloudCB(const sensor_msgs::PointCloud2ConstPtr& input)
{
  if(state==FOTO){
      pcl::fromROSMsg(*input,*scene_cloud);

#ifdef SaveCloud
      //Remove All PCD File in [package]/pcd_file/*.pcd      
      std::string sys_str;
      sys_str = "rm  " +  path + "*.pcd";
      std::cout << "[CMD] -> " << sys_str << std::endl;  
      system(sys_str.c_str());

      //write pcd
      write_pcd_2_rospack(scene_cloud,"scene_cloud.pcd");
#endif

      // feedback_.msg = "Raw Point Could Read Done (From Camera)";
      // feedback_.progress = 30;
      // as_.publishFeedback(feedback_);
      //set_feedback("Grabbing point cloud...",20);
      
      state = CALL_RCNN;
      //state = POSE_ESTIMATION;
      call_rcnn_times = 0;
  }
}


void ObjEstAction::poseEstimation(){
  ROS_INFO("In poseEstimation()");
  
  geometry_msgs::Twist pose;
  geometry_msgs::Vector3 normal;

  PCT::Ptr cloud(new PCT);
  PCT::Ptr cloud_seg (new PCT);
  PCT::Ptr cloud_seg_largest (new PCT);

  *cloud = *ROI_cloud;

  std::vector<int> index;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, index);
  
#ifdef SaveCloud
  write_pcd_2_rospack(cloud,"_rm_NaN.pcd");
#endif

  pass_through_from_arg(cloud, g_argc, g_argv, cloud);

#ifdef SaveCloud
  write_pcd_2_rospack(cloud,"_PassThrough.pcd");
#endif
  // only_obj_center(cloud, 
  //   pose.linear.x, pose.linear.y, pose.linear.z);

  float near_points_percent = 0.1;
  if (pcl::console::find_switch (g_argc, g_argv, "-near")){
    pcl::console::parse (g_argc, g_argv, "-near", near_points_percent);
  }
  cam_2_obj_center(cloud, 
      pose.linear.x, pose.linear.y, pose.linear.z, 
      pose.angular.x, pose.angular.y, pose.angular.z,
      normal.x, normal.y, normal.z,
      near_points_percent);
  
  result_.object_pose = pose;
  result_.norm = normal;

  as_.setSucceeded(result_);

  state = NADA;

}


void ObjEstAction::get_roi(){

  roi_srv.request.object_name = obj_name;
  if(roi_client.call(roi_srv))
  {
    ROS_INFO("Get ROI from Service (/detect) ");

  
    if(!roi_srv.response.result){
      
      if(call_rcnn_times < 20){
        call_rcnn_times++;
        
        return;
      }

      ROS_WARN("CANNOT Call Service (/detect)");
      preemptCB();

      ROS_WARN("/detect ROI result = False");
      geometry_msgs::Twist pose;
      pose.linear.z = -1; //ROI Fail
      result_.object_pose = pose;
      // as_.setSucceeded(result_);

      state = NADA;
         
      return;  
    }
    //darkflow_detect::Detected::ConstPtr detected_msg;
    //const darkflow_detect::Detected all_detect = roi_srv.response.detected;

    for(int i =0;i < roi_srv.response.detected.size();i++){
      darkflow_detect::Detected detected = roi_srv.response.detected[i];

      if(detected.object_name.compare(obj_name)==0){
        mini_x = detected.bound_box[0];
        mini_y = detected.bound_box[1];
        max_x =  detected.bound_box[2];
        max_y =  detected.bound_box[3];

        break;

      }

    }

    
  }else{
    ROS_WARN("CANNOT Call Service (/detect)");
    geometry_msgs::Twist pose;
    result_.object_pose = pose;
    as_.setSucceeded(result_);
    return;
  }
  ROS_INFO("[mini_x: %d, mini_y: %d], [max_x: %d, max_y: %d]",mini_x,mini_y,max_x,max_y);
  
  ROI_cloud->width = max_x-mini_x;
  ROI_cloud->height = max_y-mini_y;
  ROI_cloud->is_dense = false;
  ROI_cloud->points.resize (ROI_cloud->width * ROI_cloud->height);

  int index;
  int index_tmp=0;

  for(int j=mini_y;j<max_y;j++)
  {
    for(int i=mini_x;i<max_x;i++)
    {
      index = j*scene_cloud->width+i;
      ROI_cloud->points[index_tmp].x = scene_cloud->points[index].x;
      ROI_cloud->points[index_tmp].y = scene_cloud->points[index].y;
      ROI_cloud->points[index_tmp].z = scene_cloud->points[index].z;
      ROI_cloud->points[index_tmp].rgb = scene_cloud->points[index].rgb;
      
      index_tmp++;
    }
  }

#ifdef SaveCloud
  write_pcd_2_rospack(ROI_cloud,"_ROI.pcd");
#endif 


  set_feedback("ROI Done",60);
  
  //state = SEGMETATION;
  state = POSE_ESTIMATION;
}

void ObjEstAction::segmentation()
{
  ROS_INFO("Doing 3D Segmentation....");
  std::cout << "scene_seg = " << scence_seg << std::endl;

  /*
  CPCSegmentation cpc_seg;
  if(scence_seg)
  {
    cpc_seg.setPointCloud(scene_cloud);
    cpc_seg.set_3D_ROI(min_p, max_p);
    cpc_seg.do_segmentation();
    cloud_cluster = cpc_seg.get_cloud_cluster();
    state = ALIGMENT;
  }else{
    cpc_seg.setPointCloud(ROI_cloud);
    cpc_seg.do_segmentation();
    Max_cluster = cpc_seg.get_BiggestCluster();
    state = ALIGMENT;
  }

  //----------------- Pub Segmentation Cloud to topic -----------------//
  pcl::PointCloud<pcl::PointXYZL>::Ptr segmented_pc_ptr = cpc_seg.getSegmentedPointCloud();
  pcl::PointCloud<pcl::PointXYZL> cloud2_;

  pcl::copyPointCloud(*segmented_pc_ptr, cloud2_);
  cloud2_.clear();
  BOOST_FOREACH (pcl::PointXYZL point, *segmented_pc_ptr) 
  {
      if (point.label == 0) continue;
      cloud2_.push_back(point);
  }
  pcl::toROSMsg(cloud2_, seg_msg);
  seg_msg.header.frame_id = "camera_rgb_optical_frame";
  segmented_pub_.publish(seg_msg);
  */
}

void ObjEstAction::do_ICP()
{
  ROS_INFO("Aligning....");
  ICP_alignment my_icp;
  pcl::PointCloud<pcl::PointXYZ> temp2;
  transformation_matrix = Eigen::Matrix4f::Identity ();


  if(load_amazon_pcd_model(obj_name, model_PCD))
  {
    ROS_INFO("Load Amazon Model success!");
    if(scence_seg)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
      copyPointCloud(*cloud_cluster, *cloud_xyz);
      my_icp.setSourceCloud(model_PCD);
      my_icp.setTargetCloud(cloud_xyz);
    }else{
      my_icp.setSourceCloud(model_PCD);
      my_icp.setTargetCloud(Max_cluster);
    }
    my_icp.align(temp2);
    transformation_matrix = my_icp.getMatrix ();
    print4x4Matrix (transformation_matrix);
    //pcl::io::savePCDFile ("BIG_SEG.pcd", temp2, false);
    state = NADA;
    //----------------- Pub Segmentation Cloud to topic -----------------//
    pcl::toROSMsg(temp2, seg_msg);
    seg_msg.header.frame_id = "camera_rgb_optical_frame";
    //align_pub_.publish(seg_msg);
  }else{
    state = NADA;
  }
}

void ObjEstAction::set_feedback(std::string msg,int progress)
{
  feedback_.msg = msg;
  feedback_.progress = progress;
  as_.publishFeedback(feedback_);
}

void ObjEstAction::print4x4Matrix (const Eigen::Matrix4f & matrix)
{
  // printf ("Rotation matrix :\n");
  // printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  // printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  // printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  // printf ("Translation vector :\n");
  // printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
  float roll, pitch, yaw;
  Eigen::Affine3f transformatoin;
  transformatoin.matrix() = matrix;
  pcl::getEulerAngles(transformatoin,roll,pitch,yaw);
  Eigen::Affine3f tf_neg = Eigen::Affine3f::Identity();
  tf_neg.rotate (Eigen::AngleAxisf ( -roll,  Eigen::Vector3f::UnitX()));
  tf_neg.rotate (Eigen::AngleAxisf ( -pitch, Eigen::Vector3f::UnitY()));
  tf_neg.rotate (Eigen::AngleAxisf ( -yaw,   Eigen::Vector3f::UnitZ()));
  Eigen::Vector3f center_vec(matrix (0, 3), matrix (1, 3), matrix (2, 3));
  Eigen::Vector3f after_rotate_center_with_neg;

  after_rotate_center_with_neg = tf_neg * center_vec;

  roll = roll/3.14159*180;
  pitch = pitch/3.14159*180;
  yaw = yaw/3.14159*180;
  std::cout << "roll = " << roll << "\t pitch = " << pitch << "\t yaw = " << yaw << std::endl;
  geometry_msgs::Twist pose;
  pose.linear.x = after_rotate_center_with_neg[0];
  pose.linear.y = after_rotate_center_with_neg[1];
  pose.linear.z = after_rotate_center_with_neg[2];
  pose.angular.x = roll;
  pose.angular.y = pitch;
  pose.angular.z = yaw;
  result_.object_pose = pose;
  std::cout << "X = " << pose.linear.x << "\t Y = " << pose.linear.y << "\t Z = " << pose.linear.z << std::endl;
  //as_.setSucceeded(result_);
}



int main (int argc, char **argv)
{
  ros::init(argc, argv, "obj_pose");
  ObjEstAction ObjEst(argc, argv,"obj_pose");
  ros::Rate loop_rate(10);
  while(ros::ok())
  {
    switch(state)
    {
      case NADA:
        break;
        
      case FOTO:
        break;

      case CALL_RCNN:
        ObjEst.set_feedback("Getting ROI....",40);
        ObjEst.get_roi();
        break;

      case SEGMETATION:
        ObjEst.set_feedback("Doing segmentation....",60);
        ObjEst.segmentation();
        break;

      case ALIGMENT:
        ObjEst.set_feedback("Alignment....",80);
        ObjEst.do_ICP();
        break;
      case POSE_ESTIMATION:
        ObjEst.set_feedback("POSE_ESTIMATION....",60);
        ObjEst.poseEstimation();
        
        break;
      default:
        ROS_INFO("Que!?");
        state = NADA;
      
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return (0);
}
