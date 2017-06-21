
#include "object_pose_estimator.hpp"
//#include "seg_plane_cam2obj.hpp"
#include "OrganizedSegmentation.h"
#include "cpc_segmentation.hpp"
#include "ICP_alignment.hpp"


using namespace ObjEstAction_namespace;

void ObjEstAction::goalCB()
{
  state = FOTO;
  obj_name = as_.acceptNewGoal()->object_name;
  ROS_INFO("Action calling! Goal=%s",obj_name.c_str());    
}

void ObjEstAction::preemptCB()
{
  ROS_INFO("%s: Preempted", action_name_.c_str());
  as_.setPreempted();
}

void ObjEstAction::cloudCB(const sensor_msgs::PointCloud2ConstPtr& input)
{
  if(state==FOTO)
  {
      pcl::fromROSMsg(*input,*cloud);
#ifdef SaveCloud
      pcl::PCDWriter writer1;
      std::stringstream ss1;
      // std::string sys_str;
      // sys_str = "rm  " +  path + "*.pcd";
      //std::cout << "[CMD] -> " << sys_str << std::endl;
      //system(sys_str.c_str());
      ss1 << path << "pcd_file/scene_cloud" << ".pcd";
      writer1.write<PT> (ss1.str (), *cloud, false);
      ROS_INFO("Save PCD to %s",ss1.str().c_str());
#endif
      state = CALL_RCNN;
  }
}


void ObjEstAction::poseEstimation(){
  ROS_INFO("In poseEstimation()");
  
  geometry_msgs::Twist pose;

  PCT::Ptr cloud(new PCT);
  PCT::Ptr cloud_hsv (new PCT);
  PCT::Ptr cloud_seg (new PCT);
  PCT::Ptr cloud_seg_largest (new PCT);

  *cloud = *ROI_cloud;

  // if(obj_name.compare("seg_0") == 0){
  //   get_seg_plane(cloud, 0, cloud_seg);
  // }else if(obj_name.compare("seg_1") == 0){
  //   get_seg_plane(cloud, 1, cloud_seg);
  // }else if(obj_name.compare("seg_2") == 0){
  //   get_seg_plane(cloud, 2, cloud_seg);
  // }
  std::vector<int> index;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, index);
  
#ifdef SaveCloud
  write_pcd_2_rospack(cloud,"_rm_NaN.pcd");
#endif

  float tool_z = 0.25; 
  float pass_z_max = 0.60;

  float pass_x_min, pass_x_max, pass_y_min, pass_y_max;
  pass_x_min = pass_x_max = pass_y_min = pass_y_max = 0;



  if (pcl::console::find_switch (g_argc, g_argv, "-pass_x_min")){
    pcl::console::parse (g_argc, g_argv, "-pass_x_min", pass_x_min);
    ROS_INFO("Use pass_x_min = %lf",pass_x_min);
  }

  if (pcl::console::find_switch (g_argc, g_argv, "-pass_x_max")){
    pcl::console::parse (g_argc, g_argv, "-pass_x_max", pass_x_max);
    ROS_INFO("Use pass_x_max = %lf",pass_x_max);
  }

  if (pcl::console::find_switch (g_argc, g_argv, "-pass_y_min")){
    pcl::console::parse (g_argc, g_argv, "-pass_y_min", pass_y_min);
    ROS_INFO("Use pass_y_min = %lf",pass_y_min);
  }
  
  if (pcl::console::find_switch (g_argc, g_argv, "-pass_y_max")){
    pcl::console::parse (g_argc, g_argv, "-pass_y_max", pass_y_max);
    ROS_INFO("Use pass_y_max = %lf",pass_y_max);
  }

  if (pcl::console::find_switch (g_argc, g_argv, "-pass_z_max")){
    pcl::console::parse (g_argc, g_argv, "-pass_z_max", pass_z_max);
    
  }
  ROS_INFO("Use pass_z_max = %lf",pass_z_max);


  get_pass_through_points(cloud,  cloud,
                        pass_x_min, pass_x_max,
                        pass_y_min, pass_y_max,
                        tool_z, pass_z_max
                        
                        );

#ifdef SaveCloud
  write_pcd_2_rospack(cloud,"_PassThrough.pcd");
#endif

  // get_hsv_points(cloud, cloud_hsv,
  //       0.0, 38.0, 
  //       0.03, 1.0, 
  //       0.29, 1.0);

// get_hsv_points(cloud, cloud_hsv,
//         332.0, 36.0, 
//         0.0, 1.0, 
//         0.0, 1.0,
//         true);

  // get_hsv_points(cloud, cloud_hsv,
  //       200.0, 45.0, 
  //       0.0, 1.0, 
  //       0.0, 1.0,
  //       true);
    
// #ifdef SaveCloud
//   write_pcd_2_rospack(cloud_hsv,"_hsv.pcd");

// #endif
  // if(obj_name.compare("seg_0") == 0){
  //   region_growing(cloud, 0, cloud_seg);
  // }else if(obj_name.compare("seg_1") == 0){
  //   region_growing(cloud, 1, cloud_seg);
  // }else if(obj_name.compare("seg_2") == 0){
  //   region_growing(cloud, 2, cloud_seg);
  // }
   
   
  //get_seg_plane(cloud,  cloud_seg);
  //get_largest_cluster(cloud_seg, cloud_seg_largest);

//   region_growing(cloud, 0, cloud_seg);

// #ifdef SaveCloud
//   write_pcd_2_rospack(cloud_seg,"_region_growing.pcd");

// #endif

  //KNote: lots of time, have problem in this function , 
  //get_seg_plane_near(cloud, cloud_seg);
  //*cloud_seg_largest = *cloud_seg;

  cam_2_obj_center(cloud, 
      pose.linear.x, pose.linear.y, pose.linear.z, 
      pose.angular.x, pose.angular.y, pose.angular.z);
  
  result_.object_pose = pose;

  as_.setSucceeded(result_);

  state = NADA;

#ifdef ShowCloud
  //vis_simple(viewer,cloud);
  //viewer->addPointCloud<PT> (cloud);
  //viewer->addPointCloud<PT> (cloud_seg);
 
  get_largest_cluster(cloud_hsv, cloud_hsv);
  viewer->addPointCloud<PT> (cloud_hsv);

  PT min_p, max_p;
  pcl::getMinMax3D(*cloud_hsv,min_p, max_p);

  std::cout << "min_p = " << min_p << std::endl;
  std::cout << "max_p = " << max_p << std::endl;
  

  // vis_one_point(viewer, min_p, "min_p");
  // vis_one_point(viewer, max_p, "max_p");
  
  viewer->addCube(min_p.x, max_p.x,
                  min_p.y, max_p.y,  
                  min_p.z, max_p.z);

  while (!viewer->wasStopped () && state == NADA && ros::ok())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
#endif 

}


void ObjEstAction::get_roi(){
  // // Point clouds
  PCT::Ptr ROI_cloud (new PCT);
  my_ROICloud = PCT::Ptr (new PCT);
  roi_srv.request.object_name = obj_name;
  roi_client.call(roi_srv);
  if(roi_srv.response.result)
  {
    mini_x = roi_srv.response.bound_box[0];
    mini_y = roi_srv.response.bound_box[1];
    max_x = roi_srv.response.bound_box[2];
    max_y = roi_srv.response.bound_box[3];
  }else{
    ROS_INFO("Fail Detect!!!");
    preemptCB();
    state = NADA;
    return;
  }
  ROS_INFO("Get ROIl!");
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
      index = j*cloud->width+i;
      ROI_cloud->points[index_tmp].x = cloud->points[index].x;
      ROI_cloud->points[index_tmp].y = cloud->points[index].y;
      ROI_cloud->points[index_tmp].z = cloud->points[index].z;
      ROI_cloud->points[index_tmp].rgba = cloud->points[index].rgba;
      index_tmp++;
    }
  }
  ROS_INFO("Save point cloud in ROI!\n");
  pcl::PCDWriter writer;
  tmp_path = path;
  tmp_path.append("pcd_file/test_pcd.pcd");
  writer.write<PT> (tmp_path, *ROI_cloud, false);
  std::cerr << "Saved " << ROI_cloud->points.size () << " data points to test_pcd.pcd." << std::endl;
  pcl::getMinMax3D(*ROI_cloud, min_p, max_p);
  //ROS_INFO("ROI max_x = %f, max_y = %f, max_z = %f",max_p.x, max_p.y, max_p.z);
  //ROS_INFO("ROI min_x = %f, min_y = %f, min_z = %f",min_p.x, min_p.y, min_p.z);
  pcl::copyPointCloud(*ROI_cloud, *my_ROICloud);
  state = SEGMETATION;
}

void ObjEstAction::segmentation()
{
  ROS_INFO("Doing 3D Segmentation....");
  std::cout << "scene_seg = " << scence_seg << std::endl;

  CPCSegmentation cpc_seg;
  if(scence_seg)
  {
    cpc_seg.setPointCloud(cloud);
    cpc_seg.set_3D_ROI(min_p, max_p);
    cpc_seg.do_segmentation();
    cloud_cluster = cpc_seg.get_cloud_cluster();
    state = ALIGMENT;
  }else{
    cpc_seg.setPointCloud(my_ROICloud);
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
  seg_msg.header.frame_id = "camera_link";
  segmented_pub_.publish(seg_msg);
}

void ObjEstAction::do_ICP()
{
  ROS_INFO("Aligning....");
  ICP_alignment my_icp;
  pcl::PointCloud<pcl::PointXYZ> temp2;
  transformation_matrix = Eigen::Matrix4f::Identity ();

  tmp_path = path;
  tmp_path.append("items/Hand_Weight/trans_out_trans_out_cut_Hand_Weight1.pcd");
  if(load_pcd(tmp_path))
  {
    ROS_INFO("Load Amazon Model success!");
    if(scence_seg)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
      std::cout << "hahahhahahahahahahahahaha!!!" << std::endl;
      copyPointCloud(*cloud_cluster, *cloud_xyz);
      my_icp.setSourceCloud(cloud_xyz);
    }else{
      my_icp.setSourceCloud(Max_cluster);
    }
    my_icp.setTargetCloud(model_PCD);
    my_icp.align(temp2);
    transformation_matrix = my_icp.getMatrix ();
    print4x4Matrix (transformation_matrix);
    //pcl::io::savePCDFile ("BIG_SEG.pcd", temp2, false);
    state = NADA;
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
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
  float roll, pitch, yaw;
  Eigen::Affine3f transformatoin;
  transformatoin.matrix() = matrix;
  pcl::getEulerAngles(transformatoin,roll,pitch,yaw);
  std::cout << "roll = " << roll << "\t pitch = " << pitch << "\t yaw = " << yaw << std::endl;

}

bool ObjEstAction::load_pcd(std::string pcd_filename)
{
  model_PCD = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ> ());
  ROS_INFO("Loading PCD....");
  if(pcl::io::loadPCDFile (pcd_filename, *model_PCD) < 0)
  {
    ROS_INFO("Error loading Amazon Model cloud");
    return false;
  }else{
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
        ObjEst.set_feedback("Grabbing point cloud...",20);
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
      default:
        ROS_INFO("Que!?");
        state = NADA;
      
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return (0);
}
