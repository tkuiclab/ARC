#include "object_pose_estimator.hpp"
//#include "cpc_segmentation.hpp"
#include "cam2obj_ros.hpp"
#include "ICP_alignment.hpp"

using namespace ObjEstAction_namespace;

void ObjEstAction::goalCB()
{
  obj_list.clear();
  call_rcnn_times = 0;


  const obj_pose::ObjectPoseGoalConstPtr goal = as_.acceptNewGoal();
  obj_name = goal->object_name;
  ROS_INFO("Action calling! Goal=%s",obj_name.c_str());    
  std::cout << "-------------" << obj_name << " ----------" << std::endl;  
  
  if(goal->object_list.size() > 0){
    std::cout << "Get Object List: [" ;
    for(int i =0;i < goal->object_list.size();i++){
      std::string item_name = goal->object_list[i];
      std::cout << item_name << ",";
      obj_list.push_back(item_name);
    }

    std::cout << "]" << std::endl ;
  }

  state = FOTO;

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

      path = ros::package::getPath("obj_pose");
      path.append("/pcd_file/");
      sys_str = "rm  " +  path + "*.pcd";
      std::cout << "[CMD] -> " << sys_str << std::endl;  
      system(sys_str.c_str());

      //write pcd
      write_pcd_2_rospack(scene_cloud,"scene_cloud.pcd");
#endif
      pub_feedback("Grabbing point cloud...",20);
      
      state = CALL_RCNN;
      
  }
}

void ObjEstAction::pub_feedback(std::string msg,int progress)
{
  feedback_.msg = msg;
  feedback_.progress = progress;
  as_.publishFeedback(feedback_);
}

void ObjEstAction::pub_error(){
   geometry_msgs::Twist pose;
   pose.linear.z = -1; //ROI Fail
   result_.object_pose = pose;

   as_.setSucceeded(result_);
   //preemptCB();

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
  
  result_.object_name = obj_name;
  result_.object_pose = pose;
  result_.norm = normal;

  as_.setSucceeded(result_);

}


//Need Class Var: obj_name, call_rcnn_times
//Output Class Var: int mini_x, int mini_y,  int max_x,  int max_y;
// void ObjEstAction::detect_get_one(){
  
// }

void ObjEstAction::set_ROI_colud(
    int mini_x,int mini_y,
    int max_x, int max_y){
  
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
}

//Need Class Var: obj_list, call_rcnn_times
//Output Class Var: int mini_x, int mini_y,  int max_x,  int max_y;
bool ObjEstAction::get_highest(){
  roi_srv.request.object_name = "all";
  if(roi_client.call(roi_srv))
  {
    ROS_INFO("Get ROI from Service (/detect) ");

    if(!roi_srv.response.result || 
      roi_srv.response.detected.size() == 0){
      if(call_rcnn_times < 20){
        call_rcnn_times++;
        return false;
      }

      ROS_WARN("Call 20 times /detect ROI FAIL");         
      return false;  
    }
    
    //PCT::Ptr pt_cloud;

    //pass_through_from_arg(scene_cloud, g_argc, g_argv, scene_cloud);

#ifdef SaveCloud
  write_pcd_2_rospack(scene_cloud,"_highest_PassThrough.pcd");
#endif


    float pass_z_min = 0.3f;
    float pass_z_max = 0.6f;
    float pass_y_min = 0.0f;
    if (pcl::console::find_switch (g_argc, g_argv, "-pass_z_min")){
      pcl::console::parse (g_argc, g_argv, "-pass_z_min", pass_z_min);
    }

    if (pcl::console::find_switch (g_argc, g_argv, "-pass_z_max")){
      pcl::console::parse (g_argc, g_argv, "-pass_z_max", pass_z_max);
    }

    if (pcl::console::find_switch (g_argc, g_argv, "-pass_y_min")){
      pcl::console::parse (g_argc, g_argv, "-pass_y_min", pass_y_min);
    }

    float near_from_cam = 999.0;
    for(int i =0;i < roi_srv.response.detected.size();i++){
      darkflow_detect::Detected detected = roi_srv.response.detected[i];

      if(!is_obj_in_obj_list(detected.object_name)){
        continue;
      }

      float center_y, center_z;
      get_center_from_2dbox(
            scene_cloud,
            detected.bound_box[0], detected.bound_box[1],
            detected.bound_box[2], detected.bound_box[3],
            pass_z_min, pass_z_max,
            center_y, center_z);
      
      if(center_y > pass_y_min && center_z != -1){
        if(center_z < near_from_cam){
          obj_name = detected.object_name;
          mini_x = detected.bound_box[0];
          mini_y = detected.bound_box[1];
          max_x =  detected.bound_box[2];
          max_y =  detected.bound_box[3];
          
          near_from_cam = center_z;
        }
      }

      std::cout << detected.object_name << 
        " -> (y, z) = ("<< center_y << "," << center_z << ")" 
        << std::endl; 

    }

    
  }else{
    ROS_WARN("CANNOT Call Service (/detect)");
    return false;
  }
  ROS_INFO("The highest is %s -> [mini_x: %d, mini_y: %d], [max_x: %d, max_y: %d]",
      obj_name.c_str(),
      mini_x,mini_y,max_x,max_y);
  
  set_ROI_colud(mini_x,mini_y,max_x,max_y);
  return true;
}

bool ObjEstAction::is_obj_in_obj_list(std::string name){
  //check in request obj_list
  for(int i =0;i < obj_list.size();i++){
      if(obj_list[i].compare(name) == 0 ){
        return true;
      }
  }
  return false;
}

bool ObjEstAction::get_one_roi(){
  roi_srv.request.object_name = obj_name;
  if(roi_client.call(roi_srv))
  {
    ROS_INFO("Get ROI from Service (/detect) ");
    if(!roi_srv.response.result || 
    roi_srv.response.detected.size() == 0){
      if(call_rcnn_times < 20){
        call_rcnn_times++;
        return false;
      }

      ROS_WARN("Call 20 times /detect FAIL");
      return false;  
    }

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
    // pub_error();
    // state = NADA;

    return false;
  }
  ROS_INFO("[mini_x: %d, mini_y: %d], [max_x: %d, max_y: %d]",mini_x,mini_y,max_x,max_y);
  
  set_ROI_colud(mini_x,mini_y,max_x,max_y);

  return true;
}

bool ObjEstAction::get_roi(){
  bool success = false;
  if(obj_list.size() > 0){
    pub_feedback("Getting ROI of Highest....",40);
    success = get_highest();
  }else{
    pub_feedback("Getting ROI....",40);
    success = get_one_roi();
  }

  if(success){
#ifdef SaveCloud
    write_pcd_2_rospack(ROI_cloud,"_ROI.pcd");
#endif 
    pcl::getMinMax3D(*ROI_cloud, min_p, max_p);
    pub_feedback("ROI Done",60);
  }else{
    pub_error();
  }
  return success;
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
    //state = ALIGMENT;
  }else{
    cpc_seg.setPointCloud(ROI_cloud);
    cpc_seg.do_segmentation();
    Max_cluster = cpc_seg.get_BiggestCluster();
    //state = ALIGMENT;
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
  /*
  if(load_amazon_pcd(obj_name))
  {
    ROS_INFO("Load Amazon Model success!");
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    Eigen::Vector4f centroid;
    if(scence_seg)
    {
      // Downsample
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
      copyPointCloud(*cloud_cluster, *cloud_xyz);
      pcl::VoxelGrid<pcl::PointXYZ> sor;
      sor.setInputCloud (cloud_xyz);
      sor.setLeafSize (0.005f, 0.005f, 0.005f);
      sor.filter (*cloud_xyz);
      // Transfer model_cloud to seg_cloud
      pcl::compute3DCentroid (*cloud_xyz, centroid);
      transform_2.translation() << centroid(0), centroid(1), centroid(2);
      pcl::transformPointCloud (*model_PCD, *model_PCD, transform_2);
      // Setup input cloud for ICP
      my_icp.setSourceCloud(model_PCD);
      my_icp.setTargetCloud(cloud_xyz);
    }else{
      // Downsample
      pcl::VoxelGrid<pcl::PointXYZ> sor;
      sor.setInputCloud (Max_cluster);
      sor.setLeafSize (0.005f, 0.005f, 0.005f);
      sor.filter (*Max_cluster);
      // Transfer model_cloud to seg_cloud      
      pcl::compute3DCentroid (*Max_cluster, centroid);
      transform_2.translation() << centroid(0), centroid(1), centroid(2);
      pcl::transprint4x4MatrixformPointCloud (*model_PCD, *model_PCD, transform_2);
      // Setup input cloud for ICP
      my_icp.setSourceCloud(model_PCD);
      my_icp.setTargetCloud(Max_cluster);
    }
    my_icp.align(temp2);
    printf("Align Score = %f\n",my_icp.getScore());
    transformation_matrix = my_icp.getMatrix ();
    print4x4Matrix (transformation_matrix);
    //pcl::io::savePCDFile ("BIG_SEG.pcd", temp2, false);
    //state = NADA;
    //----------------- Pub Segmentation Cloud to topic -----------------//
    pcl::toROSMsg(temp2, seg_msg);
    seg_msg.header.frame_id = "camera_rgb_optical_frame";
    align_pub_.publish(seg_msg);
  }else{
    //state = NADA;
  }
  */
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

bool ObjEstAction::load_amazon_pcd(std::string pcd_filename)
{
  int index=0;
  std::stringstream ss1;
  for(int i =0;i<40;i++)
  {
    if(LabelList[i]==pcd_filename)
    {
      index = i;
    }
  }
  ss1 << "/home/iclab/arc_ws/src/ARC/vision/obj_pose/items/" << AmazonModelList[index] << "/" << AmazonModelList[index] << "1.pcd";
  model_PCD = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ> ());
  ROS_INFO("Loading PCD....");
  ROS_INFO("PCD at %s",ss1.str().c_str());
  if(pcl::io::loadPCDFile (ss1.str(), *model_PCD) < 0)
  {
    ROS_ERROR("Error loading Amazon Model cloud");
    return false;
  }else{
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (model_PCD);
    sor.setLeafSize (0.005f, 0.005f, 0.005f);
    sor.filter (*model_PCD);
    return true;
  }
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
        state = (ObjEst.get_roi())? POSE_ESTIMATION : NADA;
        break;

      case SEGMETATION:
        ObjEst.pub_feedback("Doing segmentation....",60);
        ObjEst.segmentation();
        state = ALIGMENT;
        break;

      case ALIGMENT:
        ObjEst.pub_feedback("Alignment....",80);
        ObjEst.do_ICP();
        state = NADA;
        break;
      case POSE_ESTIMATION:
        ObjEst.pub_feedback("POSE_ESTIMATION....",60);
        ObjEst.poseEstimation();
        state = NADA;
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
