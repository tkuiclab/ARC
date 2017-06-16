
#include "object_pose_estimator.hpp"
#include "seg_plane_cam2obj.hpp"


using namespace ObjEstAction_namespace;


int g_argc;
char **g_argv;

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

      feedback_.msg = "Raw Point Could Done (From Camera)";
      feedback_.progress = 30;
      as_.publishFeedback(feedback_);

      //state = POSE_ESTIMATION;
      state = CALL_RCNN;
      call_rcnn_times = 0;
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
  
  //PCT::Ptr ROI_cloud (new PCT);
        
  //std::cerr << "Scene width: " << scene->width << std::endl;
  //std::cerr << "Scene height: " << scene->height << std::endl;

  roi_srv.request.object_name = obj_name;
  if(roi_client.call(roi_srv))
  {
    ROS_INFO("Get ROI from Service (/detect) ");

  
    if(!roi_srv.response.result){
      
      if(call_rcnn_times < 20){
        call_rcnn_times++;
        
        return;
      }

      ROS_WARN("/detect ROI result = False");
      geometry_msgs::Twist pose;
      pose.linear.z = -1; //ROI Fail
      result_.object_pose = pose;
      as_.setSucceeded(result_);

      state = NADA;
         

      return;  
    }
    mini_x = roi_srv.response.bound_box[0];
    mini_y = roi_srv.response.bound_box[1];
    max_x = roi_srv.response.bound_box[2];
    max_y = roi_srv.response.bound_box[3];
  }else{
    ROS_WARN("CANNOT Call Service (/detect), Use Default");
    geometry_msgs::Twist pose;
    result_.object_pose = pose;
    as_.setSucceeded(result_);
    return;
    // mini_x = 170;
    // mini_y = 251;
    // max_x = 337;
    // max_y = 325;
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

  feedback_.msg = "ROI Done";
  feedback_.progress = 60;
  as_.publishFeedback(feedback_);
  
  state = POSE_ESTIMATION;
}

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

int main (int argc, char **argv)
{
  g_argc = argc;
  g_argv = argv;
  
  ros::init(argc, argv, "obj_pose");
  ObjEstAction ObjEst("obj_pose");
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
        ROS_INFO("In case CALL_RCNN");
        ObjEst.get_roi();
        break;

      // case ALIGMENT:
      //   ROS_INFO("Estimate the pose!");
      //   ObjEst.aligment();
      //   break;
      case POSE_ESTIMATION:
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

