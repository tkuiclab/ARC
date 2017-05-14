#include "../include/object_pose_estimator.hpp"

using namespace ObjEstAction_namespace;

void ObjEstAction::cloudCB(const sensor_msgs::PointCloud2ConstPtr& input)
{
  if(state==FOTO)
  {
      sensor_msgs::PointCloud2 output;
      
      output = *input;
      pcl::fromROSMsg(output,*cloud);
      
#ifdef ShowCloud
      viewer.showCloud(cloud);
#endif

#ifdef SaveCloud
    
      pcl::PCDWriter writer1;
      std::stringstream ss1;

      
      ss1 << path << "scene_cloud" << ".pcd";
      writer1.write<pcl::PointXYZRGBA> (ss1.str (), *cloud, false);
      ss1.str("");
      ss1.clear();

      ROS_INFO("Save PCD to %s",ss1.str().c_str());
#endif
      //viewer.close(); 
      //state = CALL_RCNN;
      state = POSE_ESTIMATION;
  }
}

void ObjEstAction::poseEstimation(){
  geometry_msgs::Twist pose;
  pose.linear.x = 1;
  pose.linear.y = 2;
  pose.linear.z = 3;
  
  pose.angular.x = 1.57;
  pose.angular.y = 2.57;
  pose.angular.z = 3.57;
  
  
  result_.object_pose = pose;

  as_.setSucceeded(result_);

}

void ObjEstAction::aligment(){
    // Point clouds
  PointCloudT::Ptr object (new PointCloudT);
  PointCloudT::Ptr object_aligned (new PointCloudT);
  PointCloudT::Ptr scene (new PointCloudT);
  FeatureCloudT::Ptr object_features (new FeatureCloudT);
  FeatureCloudT::Ptr scene_features (new FeatureCloudT);
  
  // Load object and scene
  tmp_path = path;
  tmp_path.append("Hand_Weight1.pcd");
  tmp_path2 = path;
  tmp_path2.append("test_pcd.pcd");
  pcl::console::print_highlight ("Loading point clouds...\n");
  if (pcl::io::loadPCDFile<PointNT> (tmp_path, *object) < 0 ||
      pcl::io::loadPCDFile<PointNT> (tmp_path2, *scene) < 0)
  {
    pcl::console::print_error ("Error loading object/scene file!\n");
  }
  
  //pcl::io::savePCDFileASCII<PointNT>("new_chef.pcd",object);
  // pcl::PCDWriter writer;
  // writer.write<PointNT> ("new_chef.pcd", *object, false);

  // writer.write<PointNT> ("new_rs1.pcd", *scene, false);

  std::cerr << "Scene before filtering: " << scene->width * scene->height << std::endl;

  // Downsample
  pcl::console::print_highlight ("Downsampling...\n");
  pcl::VoxelGrid<PointNT> grid;
  const float leaf = 0.005f;
  grid.setLeafSize (leaf, leaf, leaf);
  grid.setInputCloud (object);
  grid.filter (*object);
  grid.setInputCloud (scene);
  grid.filter (*scene);
  
  std::cerr << "Scene after filtering: " << scene->width * scene->height << std::endl;


  // Estimate normals for object
  pcl::console::print_highlight ("Estimating object normals...\n");
  pcl::NormalEstimationOMP<PointNT,PointNT> nest_obj;
  nest_obj.setRadiusSearch (0.01);
  nest_obj.setInputCloud (object);
  nest_obj.compute (*object);

  // Estimate normals for scene
  pcl::console::print_highlight ("Estimating scene normals...\n");
  pcl::NormalEstimationOMP<PointNT,PointNT> nest;
  nest.setRadiusSearch (0.01);
  nest.setInputCloud (scene);
  nest.compute (*scene);
  
  // Estimate features
  pcl::console::print_highlight ("Estimating features...\n");
  FeatureEstimationT fest;
  fest.setRadiusSearch (0.025);
  fest.setInputCloud (object);
  fest.setInputNormals (object);
  fest.compute (*object_features);
  fest.setInputCloud (scene);
  fest.setInputNormals (scene);
  fest.compute (*scene_features);
  
  // Perform alignment
  pcl::console::print_highlight ("Starting alignment...\n");
  pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
  align.setInputSource (object);
  align.setSourceFeatures (object_features);
  align.setInputTarget (scene);
  align.setTargetFeatures (scene_features);
  align.setMaximumIterations (50000*3); // Number of RANSAC iterations
  align.setNumberOfSamples (4); // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness (5); // Number of nearest features to use
  align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
  align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis
  {
    pcl::ScopeTime t("Alignment");
    align.align (*object_aligned);
  }
  
  if (align.hasConverged ())
  {
    // Print results
    printf ("\n");
    float roll,pitch,yaw;
    Eigen::Matrix4f transformation = align.getFinalTransformation ();
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
    pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
    pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
    pcl::console::print_info ("\n");
    pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid (*scene, centroid);
    printf("center point = < %6.3f, %6.3f, %6.3f >\n", centroid(0)*100, centroid(1)*100, centroid(2)*100);

    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    transform_2 = transformation;
    pcl::getEulerAngles(transform_2,roll,pitch,yaw);
    std::cout << "Roll=" << roll << std::endl;
    std::cout << "Pitch=" << pitch << std::endl;
    std::cout << "Yaw=" << yaw << std::endl;
    pcl::console::print_info ("\n");
    pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());
    
    // Show alignment
    pcl::visualization::PCLVisualizer visu("Alignment");
    visu.addCoordinateSystem (0.1, 0);
    visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
    visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 255.0, 0.0, 0.0), "object_aligned");
    visu.spin ();
  }
  else
  {
    pcl::console::print_error ("Alignment failed!\n");
  }
  state = NADA;
}

void ObjEstAction::get_roi(){
  // // Point clouds
  PointCloudT::Ptr scene (new PointCloudT);
  PointCloudT::Ptr ROI_cloud (new PointCloudT);
        
  // Load object and scene
  tmp_path = path;
  tmp_path.append("Scene_with_handweight.pcd");
  pcl::console::print_highlight ("Loading point clouds...\n");
  if (pcl::io::loadPCDFile<PointNT> ("/home/iclab-ming/ARC_ws/src/object_pose_estimator/pcd_file/Scene_with_handweight.pcd", *scene) < 0)
  {
    pcl::console::print_error ("Error loading object/scene file!\n");
  }
  
  //std::cerr << "Scene width: " << scene->width << std::endl;
  //std::cerr << "Scene height: " << scene->height << std::endl;

  roi_srv.request.object_name = obj_name;
  if(roi_client.call(roi_srv))
  {
    mini_x = roi_srv.response.bound_box[0];
    mini_y = roi_srv.response.bound_box[1];
    max_x = roi_srv.response.bound_box[2];
    max_y = roi_srv.response.bound_box[3];
  }else{
    // for test: min:  [263, 294]  max:  [400, 378]
    mini_x=263;
    mini_y=294;
    max_x=400;
    max_y=378;
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
      index = j*scene->width+i;
      ROI_cloud->points[index_tmp].x = scene->points[index].x;
      ROI_cloud->points[index_tmp].y = scene->points[index].y;
      ROI_cloud->points[index_tmp].z = scene->points[index].z;
      index_tmp++;
    }
  }
  pcl::PCDWriter writer;
  tmp_path = path;
  tmp_path.append("test_pcd.pcd");
  // ROS_INFO("Get path=%s",tmp_path.c_str());
  writer.write<PointNT> (tmp_path, *ROI_cloud, false);
  std::cerr << "Saved " << ROI_cloud->points.size () << " data points to test_pcd.pcd." << std::endl;
  state = ALIGMENT;
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
  ros::init(argc, argv, "object_pose_estimator");
  ObjEstAction ObjEst("ObjEst");
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
        ROS_INFO("Action call Goal!");
        ObjEst.get_roi();
        break;

      case ALIGMENT:
        ROS_INFO("Estimate the pose!");
        ObjEst.aligment();
        break;
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

