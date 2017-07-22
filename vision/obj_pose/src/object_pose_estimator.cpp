#include "object_pose_estimator.hpp"
#include "cpc_segmentation.hpp"
#include "cam2obj_ros.hpp"
#include "ICP_alignment.hpp"

using namespace ObjEstAction_namespace;

void ObjEstAction::goalCB()
{
  std::cout << "EEF = " << eef_pose.angular.y << std::endl;

  obj_list.clear();
  call_detect_times = 0;


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


  error_code = 0;
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
      
      //state = CALL_RCNN;
      if(obj_list.size() > 0){
        pub_feedback("Getting ROI of Highest....",40);
        state = GET_HIGHEST;
      }else{
        pub_feedback("Getting ROI....",40);
        state = GET_ONE_ROI;
      }
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
   result_.success = false;
   result_.error_code = error_code;

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
  result_.success = true;

  as_.setSucceeded(result_);

}


//Need Class Var: obj_name, call_detect_times
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
  pcl::getMinMax3D(*ROI_cloud, min_p, max_p);
}

//Need Class Var: obj_list, call_detect_times
//Output Class Var: int mini_x, int mini_y,  int max_x,  int max_y;
void ObjEstAction::get_highest(){
  roi_srv.request.object_name = "all";
  if(roi_client.call(roi_srv))
  {
    ROS_INFO("Get ROI from Service (/detect) ");

    if(!roi_srv.response.result || 
      roi_srv.response.detected.size() == 0){
      if(call_detect_times < 20){
        call_detect_times++;
        return ;    // for try next time
      }

      ROS_WARN("Call 20 times /detect FAIL");
      error_code = ERR_CALL_DETECT_OVER_TIMES;     
      pub_error();
      state = NADA;
      return ;
    }
    
    //PCT::Ptr pt_cloud;

    //pass_through_from_arg(scene_cloud, g_argc, g_argv, scene_cloud);

#ifdef SaveCloud
  write_pcd_2_rospack(scene_cloud,"_highest_PassThrough.pcd");
#endif
    /*
    if(roi_srv.respone.result == false){
      ROS_WARN("roi_srv.respone.result == false"); 
      error_code = ERR_DETECT_RESPONE_FAIL;      
      pub_error();
      return ;
    }
    if(roi_srv.response.detected.size() == 0 ){
      ROS_WARN("roi_srv.response.detected.size() == 0"); 
      error_code = ERR_DETECT_RESPONE_FAIL;
      pub_error();
      return ;     
    }
    */


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
      
      
      if(near_from_cam == 999.0){
        ROS_WARN("CANNOT Get Highest near_from_cam == 999.0");
        error_code = ERR_CANNOT_GET_HIGHEST;
        pub_error();
        return ;
      }

    }
    
  }else{
    ROS_WARN("CANNOT Call Service (/detect)");
    error_code = ERR_CANNOT_CALL_DETECT_SERVICE;
    pub_error();
    return ;
  }
  ROS_INFO("The highest is %s -> [mini_x: %d, mini_y: %d], [max_x: %d, max_y: %d]",
      obj_name.c_str(),
      mini_x,mini_y,max_x,max_y);
  
  set_ROI_colud(mini_x,mini_y,max_x,max_y);

  //state = POSE_ESTIMATION;
  state = SEGMETATION;
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

void ObjEstAction::get_one_roi(){
  roi_srv.request.object_name = obj_name;
  if(roi_client.call(roi_srv))
  {
    ROS_INFO("Get ROI from Service (/detect) ");
    if(!roi_srv.response.result || 
    roi_srv.response.detected.size() == 0){
      if(call_detect_times < 20){
        call_detect_times++;
        return ;    // for try next time
      }
      ROS_WARN("Call 20 times /detect FAIL");
      error_code = ERR_CALL_DETECT_OVER_TIMES;
      pub_error();
      state = NADA;  
      return ;  
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
    error_code = ERR_CANNOT_CALL_DETECT_SERVICE;
    pub_error();
    state = NADA;
    return ;
  }
  ROS_INFO("[mini_x: %d, mini_y: %d], [max_x: %d, max_y: %d]",mini_x,mini_y,max_x,max_y);
  
  set_ROI_colud(mini_x,mini_y,max_x,max_y);

  //state = POSE_ESTIMATION;
  state = SEGMETATION;

  return;
}
/*
bool ObjEstAction::get_roi(){
  bool success = false;
  if(obj_list.size() > 0){
    pub_feedback("Getting ROI of Highest....",40);
    success = get_highest();
  }else{
    pub_feedback("Getting ROI....",40);
    success = get_one_roi();
  }

  if(error_code == ERR_DETECT_LESS_MAX){

      return true;    // let to call detect
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
*/
void ObjEstAction::segmentation()
{
  ROS_INFO("Doing 3D Segmentation....");
  std::cout << "scene_seg = " << scence_seg << std::endl;
  
  CPCSegmentation cpc_seg;
  if(scence_seg)
  {
    cpc_seg.setPointCloud(scene_cloud);
    cpc_seg.set_3D_ROI(min_p, max_p);
    cpc_seg.do_segmentation();
    cloud_cluster = cpc_seg.get_cloud_cluster();
  }else{
    cpc_seg.setPointCloud(ROI_cloud);
    cpc_seg.do_segmentation();
    Max_cluster = cpc_seg.get_BiggestCluster();
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
  
  
}

void ObjEstAction::do_ICP()
{
  ROS_INFO("Aligning....");
  ICP_alignment my_icp;
  pcl::PointCloud<pcl::PointXYZ> temp2;
  transformation_matrix = Eigen::Matrix4f::Identity ();
  
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
      pcl::transformPointCloud (*model_PCD, *model_PCD, transform_2);
      // Setup input cloud for ICP
      my_icp.setSourceCloud(model_PCD);
      my_icp.setTargetCloud(Max_cluster);
    }
    my_icp.align(temp2);
    pcl::compute3DCentroid (temp2, centroid);
    printf("Align Score = %f\n",my_icp.getScore());
    transformation_matrix = my_icp.getMatrix ();
    print4x4Matrix (transformation_matrix,centroid);
    //pcl::io::savePCDFile ("BIG_SEG.pcd", temp2, false);
    state = NADA;
    //----------------- Pub Segmentation Cloud to topic -----------------//
    pcl::toROSMsg(temp2, seg_msg);
    seg_msg.header.frame_id = "camera_rgb_optical_frame";
    align_pub_.publish(seg_msg);
  }else{
    state = NADA;
  }
}



void ObjEstAction::print4x4Matrix (Eigen::Matrix4f & matrix, Eigen::Vector4f centroid)
{
  // printf ("Rotation matrix :\n");
  // printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  // printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  // printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  // printf ("Translation vector :\n");
  // printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
  float roll, pitch, yaw;
  Eigen::Affine3f transformatoin;

  tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  Eigen::Quaterniond tmp_eef;  
  //--------------------------------             THIS IS FOR TEST!!!              --------------------------------
  //-------------------------------- Broadcast TF from TCP to camera_link to item --------------------------------
  //  while(true)
  //  {
  //     tmp_eef = euler2Quaternion(0,90,0);

  //     q = tf::Quaternion(tmp_eef.x(),tmp_eef.y(),tmp_eef.z(), tmp_eef.w());
  //     transform.setOrigin( tf::Vector3(0.2, 0, 0.3));
  //     transform.setRotation(q);
  //     br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "robot_arm_base", "tcp"));

  //     tmp_eef = euler2Quaternion(0,0,0);
  //     q = tf::Quaternion(tmp_eef.x(),tmp_eef.y(),tmp_eef.z(), tmp_eef.w());
  //     transform.setOrigin( tf::Vector3(0.0, 0.0, -0.05));
  //     transform.setRotation(q);
  //     br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "tcp", "camera_link"));

  //     tmp_eef = euler2Quaternion(-90,90,0);
  //     q = tf::Quaternion(tmp_eef.x(),tmp_eef.y(),tmp_eef.z(), tmp_eef.w());
  //     transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0));
  //     transform.setRotation(q);
  //     br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_link", "camera_rgb_optical_frame"));

  //     q.setRPY(roll, pitch, yaw);
  //     transform.setOrigin( tf::Vector3(centroid[0], centroid[1], centroid[2]) );
  //     transform.setRotation(q);
  //     br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_rgb_optical_frame", "item"));
  //  }
  //-------------------------------- Broadcast TF from TCP to camera_link to item --------------------------------

  Eigen::Quaterniond tmp_q;
  //-------------------------------- ROS_world to robot_arm_base --------------------------------
  tmp_q = euler2Quaternion(180, 90, 0);
  Eigen::Matrix3d tmp = tmp_q.matrix();
  Eigen::Matrix4f tmp_mat;
  Eigen::Matrix4f tmp_rot_mat;
  Eigen::Matrix4f tmp_eef_mat;

  double tmp_roll=(eef_pose.angular.x/3.14*180);
  double tmp_pitch=(eef_pose.angular.y/3.14*180);
  double tmp_yaw = eef_pose.angular.z/3.14*180;

  // Define robot_arm_base
  tmp_mat << tmp(0,0), tmp(0,1), tmp(0,2), 0,
             tmp(1,0), tmp(1,1), tmp(1,2), 0,
             tmp(2,0), tmp(2,1), tmp(2,2), 0,
                    0,        0,        0, 1;
  //-------------------------------- ROS_world to robot_arm_base --------------------------------
  //-------------------------------- robot_arm_base to TCP --------------------------------
  // Translate to EEF X,Y,Z
  tmp_rot_mat << 1, 0, 0, eef_pose.linear.z,
                 0, 1, 0, eef_pose.linear.y,
                 0, 0, 1, eef_pose.linear.x,
                 0, 1, 0,                 1;
  tmp_eef_mat = tmp_mat*tmp_rot_mat;

  // Rotate Pitch
  tmp_q = euler2Quaternion(0, tmp_pitch, 0);
  tmp = tmp_q.matrix();
  tmp_rot_mat << tmp(0,0), tmp(0,1), tmp(0,2), 0,
                 tmp(1,0), tmp(1,1), tmp(1,2), 0,
                 tmp(2,0), tmp(2,1), tmp(2,2), 0,
                        0,        0,        0, 1;
  tmp_eef_mat = tmp_eef_mat*tmp_rot_mat;

  // Rotate Yaw
  tmp_q = euler2Quaternion(0, 0, tmp_yaw);
  tmp = tmp_q.matrix();
  tmp_rot_mat << tmp(0,0), tmp(0,1), tmp(0,2), 0,
                 tmp(1,0), tmp(1,1), tmp(1,2), 0,
                 tmp(2,0), tmp(2,1), tmp(2,2), 0,
                        0,        0,        0, 1;
  tmp_eef_mat = tmp_eef_mat*tmp_rot_mat;

  // Rotate Roll
  tmp_q = euler2Quaternion(tmp_roll, 0, 0);
  tmp = tmp_q.matrix();
  tmp_rot_mat << tmp(0,0), tmp(0,1), tmp(0,2), 0,
                 tmp(1,0), tmp(1,1), tmp(1,2), 0,
                 tmp(2,0), tmp(2,1), tmp(2,2), 0,
                        0,        0,        0, 1;
  tmp_eef_mat = tmp_eef_mat*tmp_rot_mat;

  std::cout << "=================== robot_arm_base to TCP ===================" << std::endl;                        
  // std::cout << tmp_eef_mat << std::endl;
  transformatoin.matrix() = tmp_eef_mat;
  pcl::getEulerAngles(transformatoin,roll,pitch,yaw);
  std::cout << "rosrun tf static_transform_publisher " << tmp_eef_mat(0,3) << " " << tmp_eef_mat(1,3) << " " << tmp_eef_mat(2,3) << " " << yaw << " " << pitch << " " << roll << " ros_world my_tmp 0.1" << std::endl;
  //-------------------------------- robot_arm_base to TCP --------------------------------
  //-------------------------------- TCP to camera_link --------------------------------
  tmp_q = euler2Quaternion(180,0,0);
  tmp = tmp_q.matrix();
  Eigen::Matrix4f test_rot_mat;
  test_rot_mat << tmp(0,0), tmp(0,1), tmp(0,2), 0.0655,
                  tmp(1,0), tmp(1,1), tmp(1,2), 0.0,
                  tmp(2,0), tmp(2,1), tmp(2,2), 0.091,
                        0,        0,        0,    1;

  Eigen::Matrix4f camera_link_mat;
  camera_link_mat = tmp_eef_mat*test_rot_mat;
  std::cout << "=================== TCP to camera_link ===================" << std::endl;                        
  transformatoin.matrix() = camera_link_mat;
  pcl::getEulerAngles(transformatoin,roll,pitch,yaw);
  std::cout << "rosrun tf static_transform_publisher " << camera_link_mat(0,3) << " " << camera_link_mat(1,3) << " " << camera_link_mat(2,3) << " " << yaw << " " << pitch << " " << roll << " ros_world my_tmp 0.1" << std::endl;
  //-------------------------------- TCP to camera_link --------------------------------
  //-------------------------------- camera_link to camera_rgb_optical --------------------------------
  tmp_q = euler2Quaternion(-90,90,0);
  tmp = tmp_q.matrix();
  test_rot_mat << tmp(0,0), tmp(0,1), tmp(0,2), 0.0,
                  tmp(1,0), tmp(1,1), tmp(1,2), 0.0,
                  tmp(2,0), tmp(2,1), tmp(2,2), 0.0,
                        0,        0,        0,    1;

  Eigen::Matrix4f camera_rgb_optical_frame_mat;
  camera_rgb_optical_frame_mat = camera_link_mat*test_rot_mat;
  std::cout << "=================== camera_link to camera_rgb_optical ===================" << std::endl;
  // Using Quaternion to show TF             
  // Matrix3f mat;
  // mat << camera_rgb_optical_frame_mat(0,0), camera_rgb_optical_frame_mat(0,1), camera_rgb_optical_frame_mat(0,2),
  //        camera_rgb_optical_frame_mat(1,0), camera_rgb_optical_frame_mat(1,1), camera_rgb_optical_frame_mat(1,2),
  //        camera_rgb_optical_frame_mat(2,0), camera_rgb_optical_frame_mat(2,1), camera_rgb_optical_frame_mat(2,2);
  // Eigen::Quaternionf tmp_QQ(mat);
  // std::cout << "tmp_QQ: q.x = " << tmp_QQ.x() << "\tq.y = " << tmp_QQ.y() << "\tq.z = "  << tmp_QQ.z() << "\tq.w = "  << tmp_QQ.w() << std::endl;
  transformatoin.matrix() = camera_rgb_optical_frame_mat;
  pcl::getEulerAngles(transformatoin,roll,pitch,yaw);
  std::cout << "rosrun tf static_transform_publisher " << camera_rgb_optical_frame_mat(0,3) << " " << camera_rgb_optical_frame_mat(1,3) << " " << camera_rgb_optical_frame_mat(2,3) << " " << yaw << " " << pitch << " " << roll << " ros_world my_tmp 0.1" << std::endl;
  //-------------------------------- camera_link to camera_rgb_optical --------------------------------
  //-------------------------------- camera_rgb_optical to item --------------------------------
  // For extent tool
  // std::cout << "=================== camera_rgb_optical to item_rot_mat ===================" << std::endl;
  // std::cout << matrix << std::endl;
  // matrix(2,3) = matrix(2,3)+0.1;
  // std::cout << matrix << std::endl;
  transformatoin.matrix() = matrix;
  pcl::getEulerAngles(transformatoin,roll,pitch,yaw);
  q.setRPY(roll, pitch, yaw);
  tmp_q = Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z());
  tmp = tmp_q.matrix();
  test_rot_mat << tmp(0,0), tmp(0,1), tmp(0,2), centroid[0],
                  tmp(1,0), tmp(1,1), tmp(1,2), centroid[1],
                  tmp(2,0), tmp(2,1), tmp(2,2), centroid[2],
                        0,        0,        0,            1;

  // std::cout << test_rot_mat << std::endl;
  // std::cout <<  centroid[0] <<  centroid[1] <<  centroid[2] << std::endl;

  Eigen::Matrix4f item_mat;
  item_mat = camera_rgb_optical_frame_mat*test_rot_mat;
  std::cout << "=================== camera_rgb_optical to item ===================" << std::endl;
  transformatoin.matrix() = item_mat;
  pcl::getEulerAngles(transformatoin,roll,pitch,yaw);
  std::cout << "rosrun tf static_transform_publisher " << item_mat(0,3) << " " << item_mat(1,3) << " " << item_mat(2,3) << " " << yaw << " " << pitch << " " << roll << " ros_world item 0.1" << std::endl;
  //-------------------------------- camera_rgb_optical to item --------------------------------  
  //-------------------------------- item to grab_item --------------------------------  
  tmp_q = euler2Quaternion(-90,0,-90);

  tmp = tmp_q.matrix();
  test_rot_mat << tmp(0,0), tmp(0,1), tmp(0,2), 0.0,
                  tmp(1,0), tmp(1,1), tmp(1,2), 0.0,
                  tmp(2,0), tmp(2,1), tmp(2,2), 0.0,
                        0,        0,        0,    1;
  Eigen::Matrix4f item_grab_mat;
  item_grab_mat = item_mat*test_rot_mat;

  transformatoin.matrix() = item_grab_mat;
  pcl::getEulerAngles(transformatoin,roll,pitch,yaw);

  std::cout << "=================== item to grab_item ===================" << std::endl;                    
  std::cout << "rosrun tf static_transform_publisher " << item_mat(0,3) << " " << item_mat(1,3) << " " << item_mat(2,3) << " " << yaw << " " << pitch << " " << roll << " ros_world item 0.1" << std::endl;
  roll = roll/3.14159*180;
  pitch = pitch/3.14159*180;
  yaw = yaw/3.14159*180;
  // roll = yaw/3.14159*180*-1;
  // pitch = pitch/3.14159*180-180;
  // yaw = roll/3.14159*-180-180;
  // std::cout << "roll = " << roll << "\t pitch = " << pitch << "\t yaw = " << yaw << std::endl;    
  //-------------------------------- item to grab_item --------------------------------  
  //-------------------------------- FOR TOOL --------------------------------  
  tmp_q = euler2Quaternion(0,0,0);
  tmp = tmp_q.matrix();
  // test_rot_mat << tmp(0,0), tmp(0,1), tmp(0,2), -0.126,
  test_rot_mat << tmp(0,0), tmp(0,1), tmp(0,2), -0.235,
                  tmp(1,0), tmp(1,1), tmp(1,2), 0.0,
                  tmp(2,0), tmp(2,1), tmp(2,2), 0.0,
                        0,        0,        0,    1;

  Eigen::Matrix4f tool_eef;
  tool_eef = item_grab_mat*test_rot_mat;
  transformatoin.matrix() = tool_eef;
  pcl::getEulerAngles(transformatoin,roll,pitch,yaw);
  std::cout << "=================== FOR TOOL ===================" << std::endl;                    
  std::cout << "rosrun tf static_transform_publisher " << tool_eef(0,3) << " " << tool_eef(1,3) << " " << tool_eef(2,3) << " " << yaw << " " << pitch << " " << roll << " ros_world item 0.1" << std::endl;
  //-------------------------------- FOR TOOL Length --------------------------------  
  //-------------------------------- Relative camera to item --------------------------------  
  transformatoin.matrix() = matrix;
  pcl::getEulerAngles(transformatoin,roll,pitch,yaw);
  std::cout << "=================== Relative camera to item ===================" << std::endl;
  roll = roll/3.14159*180;
  pitch = pitch/3.14159*180;
  yaw = yaw/3.14159*180;       
  std::cout << "roll = " << roll << "\t pitch = " << pitch << "\t yaw = " << yaw << std::endl;    
  //-------------------------------- Relative camera to item --------------------------------  

  geometry_msgs::Twist pose;
  pose.linear.x = tool_eef(0,3);
  pose.linear.y = tool_eef(1,3);
  pose.linear.z = tool_eef(2,3);
  pose.angular.x = roll;
  pose.angular.y = pitch;
  pose.angular.z = yaw;
  result_.object_pose = pose;
  //std::cout << "X = " << pose.linear.x << "\t Y = " << pose.linear.y << "\t Z = " << pose.linear.z << std::endl;
  // std::cout << "roll = " << roll << "\t pitch = " << pitch << "\t yaw = " << yaw << std::endl;
  //std::cout << roll << " " << pitch << " " << yaw << std::endl;
  as_.setSucceeded(result_);
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
  std::string tmp_path;
  tmp_path = ros::package::getPath("obj_pose");
  tmp_path.append("/items/");
  ss1 << tmp_path << AmazonModelList[index] << "/" << AmazonModelList[index] << "1.pcd";
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

Eigen::Quaterniond ObjEstAction::euler2Quaternion( double roll,
                                     double pitch,
                                     double yaw )
{
    roll = roll/180*3.14159;
    pitch = pitch/180*3.14159;
    yaw = yaw/180*3.14159;
    
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = rollAngle * pitchAngle * yawAngle;
    Eigen::Matrix3d matrix = q.matrix();
    // printf ("Rotation matrix :\n");
    // printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
    // printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
    // printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
    // std::cout << "Eigen = " << q.x() << std::endl << q.y() << std::endl << q.z() << std::endl << q.w() << std::endl;
    return q;
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "obj_pose");
  ObjEstAction ObjEst(argc, argv,"obj_pose");
  ros::Rate loop_rate(100);
  while(ros::ok())
  {
    switch(state)
    {
      case NADA:
        break;
        
      case FOTO:
        break;

      case GET_ONE_ROI:
        ObjEst.get_one_roi();
        break;

      case GET_HIGHEST:
        ObjEst.get_highest();
        break;
      // case CALL_RCNN:
      //   state = (ObjEst.get_roi())? POSE_ESTIMATION : NADA;
      //   break;

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
