#include "pcl_utility.hpp"

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/common/distances.h>

#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/common/common.h>
#include <pcl/surface/mls.h>

void write_pcd_2_rospack(PCT::Ptr cloud, std::string f_name){
    std::string path = ros::package::getPath("obj_pose");
    path.append("/pcd_file/");
    path.append(f_name);

    pcl::PCDWriter writer;
    writer.write<PT> (path, *cloud, false);

    std::cout << "Save PCD -> " << path << std::endl;
}


void write_pcd_2_rospack_normals(PC_Normal::Ptr cloud, std::string f_name){
    std::string path = ros::package::getPath("obj_pose");
    path.append("/pcd_file/");
    path.append(f_name);

    pcl::PCDWriter writer;
    writer.write<PNormal> (path, *cloud, false);

    std::cout << "Save PCD -> " << path << std::endl;
}

void get_largest_cluster( PCT::Ptr i_cloud ,PCT::Ptr o_cloud ){
  pcl::search::KdTree<PT>::Ptr tree (new pcl::search::KdTree<PT>);
  tree->setInputCloud (i_cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PT> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (i_cloud);
  ec.extract (cluster_indices);

  //int j = 0;
  // for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  // {
    //get only first cluster
    std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ();
    PCT::Ptr cloud_cluster (new PCT);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (i_cloud->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    //std::stringstream ss;
    //ss << "cloud_cluster_" << j << ".pcd";
    //writer.write<PT> (ss.str (), *cloud_cluster, false); //*
#ifdef SaveCloud    
  write_pcd_2_rospack(cloud_cluster,"_largest_cluster.pcd");

#endif
    //j++;
  //}
  *o_cloud = *cloud_cluster;
}

void get_seg_plane( PCT::Ptr i_cloud, int want_seg_ind ,PCT::Ptr o_cloud ){
//void get_seg_plane( PCT::Ptr i_cloud, PCT::Ptr o_cloud ){
  
  //PCT::Ptr cloud_blob (new PCT);
  PCT::Ptr cloud_filtered (new PCT), cloud_p (new PCT), cloud_f (new PCT);

  std::cerr << "PointCloud before filtering: " << i_cloud->width * i_cloud->height << " data points." << std::endl;


  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  //pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  pcl::VoxelGrid<PT> sor;
  sor.setInputCloud (i_cloud);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

  // Write the downsampled version to disk
  // pcl::PCDWriter writer;
  // writer.write<PT> (f_name + "_downsample.pcd", *cloud_filtered, false);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<PT> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (5000);
  //seg.setDistanceThreshold (0.01);
  seg.setDistanceThreshold (0.01);

    // Create the filtering object
  pcl::ExtractIndices<PT> extract;


  double min_z = 9999.0;
  int i = 0, nr_points = (int) cloud_filtered->points.size ();
  // While 30% of the original cloud is still there
  //while (cloud_filtered->points.size () > 0.3 * nr_points)
  while (cloud_filtered->points.size () > 0.1 * nr_points){
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    PT center = getCenter(cloud_p);


    if(want_seg_ind == i){
        *o_cloud = *cloud_p;
        ROS_INFO("Use _seg_%d_plane",i);
        //return;
    }
      // if(center.z < min_z){
      //   *o_cloud = *cloud_p;
      //   ROS_INFO("Use _seg_%d_plane z=%lf",i,center.z);
      //   min_z = center.z;
      // }
      //return ;
    

#ifdef SaveCloud
  
    std::string path = ros::package::getPath("obj_pose");
    path.append("/pcd_file/");

    pcl::PCDWriter writer;
    std::stringstream ss;
    ss  << path << "_seg_" << i << ".pcd";
    writer.write<PT> (ss.str (), *cloud_p, false);

    std::cout << "Save PCD -> " << ss.str() << std::endl;
     //ROS_INFO("Save PCD -> %s ", ss.str().c_str());
#endif
    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);

    cloud_filtered.swap (cloud_f);

    i++;
  }


  if(want_seg_ind > i){
      std::cerr << "ERROR!! want_seg_ind > " << i  << std::endl;
  }


}


void get_seg_plane_near( PCT::Ptr i_cloud ,PCT::Ptr o_cloud ){
//void get_seg_plane( PCT::Ptr i_cloud, PCT::Ptr o_cloud ){
  
  //PCT::Ptr cloud_blob (new PCT);
  PCT::Ptr cloud_filtered (new PCT), cloud_p (new PCT), cloud_f (new PCT);
  PCT::Ptr cloud_seg_largest (new PCT);
  std::cerr << "PointCloud before filtering: " << i_cloud->width * i_cloud->height << " data points." << std::endl;


  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  //pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  pcl::VoxelGrid<PT> sor;
  sor.setInputCloud (i_cloud);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

  // Write the downsampled version to disk
  // pcl::PCDWriter writer;
  // writer.write<PT> (f_name + "_downsample.pcd", *cloud_filtered, false);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<PT> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (5000);
  //seg.setDistanceThreshold (0.01);
  seg.setDistanceThreshold (0.01);

    // Create the filtering object
  pcl::ExtractIndices<PT> extract;


  float min_dis = 9999.0;
  int i = 0, nr_points = (int) cloud_filtered->points.size ();
  // While 30% of the original cloud is still there
  //while (cloud_filtered->points.size () > 0.3 * nr_points)

  std::cout << "cloud_filtered->points.size () = "  << cloud_filtered->points.size () << std::endl;
 
  while (cloud_filtered->points.size () > 0.1 * nr_points){
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    
    get_largest_cluster(cloud_p, cloud_seg_largest);
    PT center = getCenter(cloud_seg_largest);
    float dis = pcl::euclideanDistance(PT(0,0,0), center);
    
    if(dis < min_dis){
      *o_cloud = *cloud_seg_largest;
      ROS_INFO("Use _seg_%d_plane dis=%lf",i, dis);
      min_dis = dis;
    }
  
#ifdef SaveCloud
  
    std::string path = ros::package::getPath("obj_pose");
    path.append("/pcd_file/");

    pcl::PCDWriter writer;
    std::stringstream ss;
    ss  << path << "_seg_" << i << ".pcd";
    //writer.write<PT> (ss.str (), *cloud_p, false);
    writer.write<PT> (ss.str (), *cloud_seg_largest, false);

    std::cout << "Save PCD -> " << ss.str() << std::endl;
     //ROS_INFO("Save PCD -> %s ", ss.str().c_str());
#endif
    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);

    cloud_filtered.swap (cloud_f);

    std::cout << "cloud_filtered->points.size () = "  << cloud_filtered->points.size () << std::endl;
 
    i++;
  }


}

//get camera center to object center transform
//assume camera
void cam_2_obj_center(PCT::Ptr i_cloud,
          double &x, double &y, double &z,
          double &roll, double &pitch, double &yaw){
  PCT::Ptr cloud_near_center (new PCT);

  //get center of colud
  PT center =  getCenter(i_cloud);
  std::cout << "Center Point = " << center << std::endl;

  // x = center.x;
  // y = center.y;
  // z = center.z;

  // return;

  //get near center points
  //ouput to cloud_near_center
  //get_near_points(i_cloud, center, 30, cloud_near_center);
  //get_near_points(i_cloud, center, i_cloud->size() * 0.5f, cloud_near_center);

  //get bounding box
  // PT min_p, max_p;
  // pcl::getMinMax3D(*i_cloud,min_p, max_p);
  
  std::cout << "cam_2_obj_center() say Points = " << i_cloud->size() << std::endl;

   pcl::PointCloud<PNormal>::Ptr cloud_normal (new pcl::PointCloud<PNormal>);
  
  

  // ----------------------------------------------------------------
  // -----Calculate surface normals with a search radius of 0.05-----
  // ----------------------------------------------------------------
  // pcl::NormalEstimation<PT, PNormal> ne;
  // //ne.setInputCloud (cloud_near_center);
  // ne.setInputCloud (i_cloud);
  // //option
  // //pcl::search::KdTree<PT>::Ptr tree (new pcl::search::KdTree<PT> ());
  // //ne.setSearchMethod (tree);
  
  // //pcl::PointCloud<pcl::Normal>::Ptr cloud_normal (new pcl::PointCloud<pcl::Normal>);
  // //pcl::PointCloud<PNormal>::Ptr cloud_normal (new pcl::PointCloud<PNormal>);
  
  // ne.setRadiusSearch (0.3);
  // ne.compute (*cloud_normal);

// #ifdef SaveCloud    
//   write_pcd_2_rospack_normals(cloud_normal,"_NormalEstimation.pcd");

// #endif

  //------------------MovingLeastSquares----------------//
   // Create a KD-Tree
  pcl::search::KdTree<PT>::Ptr tree (new pcl::search::KdTree<PT>);
  pcl::PointCloud<pcl::PointNormal> mls_points;
  pcl::MovingLeastSquares<PT, pcl::PointNormal> mls;
  mls.setComputeNormals (true);
  mls.setInputCloud (i_cloud);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.03);
  mls.process (*cloud_normal);
  

#ifdef SaveCloud    
  write_pcd_2_rospack_normals(cloud_normal,"_mls.pcd");
#endif

  if(i_cloud->size() > 1000){
    float leaf = 0.01;
    // if (pcl::console::find_switch (argc, argv, "-leaf")){
    //   pcl::console::parse (argc, argv, "-leaf", leaf);
    // }

    pcl::VoxelGrid<PT> vg;
    vg.setInputCloud (i_cloud);
    vg.setLeafSize (leaf, leaf, leaf);
    vg.filter (*i_cloud);
    std::cout << "cam_2_obj_center() say After VoxelGrid Points = " << i_cloud->size() << std::endl;
  }

   mls.setInputCloud (i_cloud);
   mls.process (*cloud_normal);


#ifdef SaveCloud    
  write_pcd_2_rospack_normals(cloud_normal,"_mls_voxelgrid.pcd");
#endif

  //getNormal_Near_Point(cloud_normal, center);

  std::cout << "cloud_normal width*height = " << cloud_normal->width * cloud_normal->height << std::endl;
  std::cout << "cloud_normal size = " << cloud_normal->size() << std::endl;
  

  Vector3f  obj_normal = get_normal_mean(cloud_normal);
  
  //----------------------------------------//
  // Get Angle of Cam Normal to Object Normal //
  //----------------------------------------//
  Vector3f  cam_normal(0.0,0.0,1.0);
  Matrix3f  R;


  std::cout << " obj_normal -> " << obj_normal << std::endl;  
  if(obj_normal [2] < 0){
    obj_normal = obj_normal * (-1);

    std::cout << " Update  obj_normal -> " << obj_normal << std::endl;  
  }

  
  // ----Calculate Rotate from  cam_normal to obj_normal-------//
  R = Quaternionf().setFromTwoVectors(cam_normal,obj_normal);
 
  
  //Vector3f euler = R.eulerAngles(2, 1, 0);
  //yaw = euler[0]; pitch = euler[1]; roll = euler[2]; 
  Vector3f euler = R.eulerAngles(0, 1, 2);
  yaw = euler[2]; pitch = euler[1]; roll = euler[0]; 
  
  Eigen::Affine3f tf_neg = Eigen::Affine3f::Identity();
  tf_neg.rotate (Eigen::AngleAxisf ( -yaw,   Eigen::Vector3f::UnitZ()));
  tf_neg.rotate (Eigen::AngleAxisf ( -pitch, Eigen::Vector3f::UnitY()));
  tf_neg.rotate (Eigen::AngleAxisf ( -roll,  Eigen::Vector3f::UnitX()));

  
  Vector3f center_vec(center.x, center.y, center.z);
  Vector3f after_rotate_center_with_neg;

  after_rotate_center_with_neg = tf_neg * center_vec;
  x = after_rotate_center_with_neg[0];
  y = after_rotate_center_with_neg[1];
  z = after_rotate_center_with_neg[2];

 

  std::cout << " (roll, pitch, yaw) = "  
    <<  "("  << roll << "," << pitch << "," << yaw << ") = "  
     <<  "(" <<  pcl::rad2deg(roll)  << "," 
     <<  pcl::rad2deg(pitch)  << "," 
     <<  pcl::rad2deg(yaw) << ")" << std::endl;
    
  // no use transform
  // x = center.x;
  // y = center.y;
  // z = center.z;

  // std::cout << "NO USE ROTATE  (roll, pitch, yaw) = "  
  //   <<  "("  << roll << "," << pitch << "," << yaw << ") = "  
  //    <<  "(" <<  pcl::rad2deg(roll)  << "," 
  //    <<  pcl::rad2deg(pitch)  << "," 
  //    <<  pcl::rad2deg(yaw) << ")" << std::endl;
    

//--test negtive the obj_normal-------------------------------------------------//
  /*
  Matrix3f  R2;
  obj_normal = obj_normal * (-1);
  //KNote: a rotation between the two "arbitrary" vectors
  // I think only 0 ~ 180
  R2 = Quaternionf().setFromTwoVectors(cam_normal,obj_normal);
 
  euler = R2.eulerAngles(0, 1, 2);
  yaw = euler[2]; pitch = euler[1]; roll = euler[0]; 
  std::cout << " test again (roll, pitch, yaw) = "  
     <<  "("  << roll <<  "," << pitch << "," << yaw << ") = "  
     <<  "(" <<  pcl::rad2deg(roll)  << "," 
     <<  pcl::rad2deg(pitch)  << "," 
     <<  pcl::rad2deg(yaw) << ")" << std::endl;
     */
//end  ---test negtive the obj_normal------------------------------------//


  std::cout << " center (x, y, z) = "  
  <<  "(" <<  center.x  << ","  <<  center.y  << "," <<  center.z << ")" << std::endl;

  std::cout << " (x, y, z) = "  
  <<  "(" <<  x  << ","  <<  y  << "," <<  z << ")" << std::endl;

  
#ifdef ShowCloud
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  return ;
  Vector3f  new_vec;
  new_vec = R * cam_normal;

  vis_normal(viewer, cloud_near_center, cloud_normal);
  vis_one_point(viewer, center);
  //vis_cloud(viewer, cloud_near_center);
  
  // viewer->addCube(min_p.x, min_p.y, min_p.z, 
  //                 max_p.x, max_p.y, max_p.z);
  //viewer->addLine (PT(0, 0, 0),
  //PT(new_vec[0],new_vec[1],new_vec[2])          );
  PT p_new;
  p_new.x = center.x + new_vec[0];
  p_new.y = center.y + new_vec[1];
  p_new.z = center.z + new_vec[2];

  PT obj_new;
  obj_new.x = center.x + obj_normal[0];
  obj_new.y = center.y + obj_normal[1];
  obj_new.z = center.z + obj_normal[2];


//-----------------TEST-------------------//
  std::cout << " ==============Test=============="  << std::endl;

  Eigen::Affine3f tf = Eigen::Affine3f::Identity();
  tf.rotate (Eigen::AngleAxisf ( yaw,   Eigen::Vector3f::UnitZ()));
  tf.rotate (Eigen::AngleAxisf ( pitch, Eigen::Vector3f::UnitY()));
  tf.rotate (Eigen::AngleAxisf ( roll,  Eigen::Vector3f::UnitX()));

  std::cout << " obj_normal = " <<  obj_normal << std::endl;
  std::cout << " new_vec = " <<  new_vec << std::endl;

  Vector3f  back_vec;
  back_vec = tf * new_vec;
  std::cout << "back_vec = " << std::endl << back_vec << std::endl;
  back_vec = tf_neg * new_vec;
  std::cout << "back_vec_with_negtf = " << std::endl << back_vec << std::endl;
  
  std::cout << " R = " << std::endl <<  tf.matrix()  << std::endl;
  //R same as tf
  std::cout << " tf = " << std::endl <<  tf.matrix()  << std::endl;
  std::cout << " tf_neg = " << std::endl <<  tf_neg.matrix()  << std::endl;


  //!!!Sumary =  Rotate * ori_point + center;
  viewer->addLine (center,   p_new ,255.0, 0, 0);
  viewer->addLine (center,   obj_new , 0, 255.0, 0);

  std::cout << " center = " <<  center_vec << std::endl;
  std::cout << " after_rotate_center_with_neg= " <<  after_rotate_center_with_neg << std::endl;

#endif 
}



void region_growing(PCT::Ptr i_cloud, int want_seg_ind ,PCT::Ptr o_cloud){
  PCT::Ptr cloud (new PCT);
  PCT::Ptr cloud_vg (new PCT);

  std::cout << "i_cloud  size = " << i_cloud->size() << std::endl;
  

  pcl::VoxelGrid<PT> vg;
  vg.setInputCloud (i_cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_vg);

  std::cout << "after vg size = " << cloud_vg->size() << std::endl;

#ifdef SaveCloud    
  write_pcd_2_rospack(cloud_vg,"_vg.pcd");

#endif

  *cloud = *cloud_vg;
  //*cloud = *i_cloud;

  pcl::search::Search<PT>::Ptr tree = boost::shared_ptr<pcl::search::Search<PT> > (new pcl::search::KdTree<PT>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<PT, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<PT> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  pass.filter (*indices);

  pcl::RegionGrowing<PT, pcl::Normal> reg;
//   reg.setMinClusterSize (50);
//   reg.setMaxClusterSize (1000000);
  reg.setMinClusterSize (cloud->points.size () * 0.1);
  reg.setMaxClusterSize (cloud->points.size ());
 
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (30);
  reg.setInputCloud (cloud);
  //reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (1.0);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  std::cout << "Number of clusters is equal to " << clusters.size () << std::endl;
  std::cout << "First cluster has " << clusters[0].indices.size () << " points." << endl;
  std::cout << "These are the indices of the points of the initial" <<
    std::endl << "cloud that belong to the first cluster:" << std::endl;
  
  //PCT::Ptr cloud_extract(new PCT);
  //PCT::Ptr cloud_f(new PCT);
  
  int i = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin (); it != clusters.end (); ++it)
  {
    
   PCT::Ptr cloud_cluster (new PCT);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    
    
#ifdef SaveCloud    
  std::stringstream ss;
  ss <<  "_" << i << "_rg_cluster.pcd";
    
  write_pcd_2_rospack(cloud_cluster,ss.str());

#endif
    if(i == want_seg_ind){
      *o_cloud = *cloud_cluster;
      std::cout << "Use _cluster_" << i << "_plane";
    }
    
    i++;
    

    // PT p = getCenter(cloud_cluster);
    // float dis = pcl::euclideanDistance(PT(0,0,0), p);
    //std::cout << p << ", distance = " << dis << std::endl;
    

  }


  std::cout << "region_growing FINISH" << std::endl;
    

}