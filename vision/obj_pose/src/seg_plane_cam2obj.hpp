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


void write_pcd_2_rospack(PCT::Ptr cloud, std::string f_name){
    std::string path = ros::package::getPath("obj_pose");
    path.append("/pcd_file/");
    path.append(f_name);

    pcl::PCDWriter writer;
    writer.write<PT> (path, *cloud, false);


    std::cout << "Save PCD -> " << path << std::endl;
}


void get_largest_cluster( PCT::Ptr i_cloud ,PCT::Ptr o_cloud ){
  std::cout << " "
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
    //writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
#ifdef SaveCloud    
  write_pcd_2_rospack(cloud_cluster,"_cluster.pcd");

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

    pcl::PointXYZ center = getCenter(cloud_p);


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
    pcl::PointXYZ center = getCenter(cloud_seg_largest);
    float dis = pcl::euclideanDistance(pcl::PointXYZ(0,0,0), center);
    
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
  pcl::PointXYZ center =  getCenter(i_cloud);
  std::cout << "Center Point = " << center << std::endl;

  //get near center points
  //ouput to cloud_near_center
  get_near_points(i_cloud, center, 10, cloud_near_center);

 
  // ----------------------------------------------------------------
  // -----Calculate surface normals with a search radius of 0.05-----
  // ----------------------------------------------------------------
  pcl::NormalEstimation<PT, pcl::Normal> ne;
  ne.setInputCloud (cloud_near_center);
  //option
  //pcl::search::KdTree<PT>::Ptr tree (new pcl::search::KdTree<PT> ());
  //ne.setSearchMethod (tree);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normal (new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch (0.3);
  ne.compute (*cloud_normal);

  
  //getNormal_Near_Point(cloud_normal, center);

  std::cout << "cloud_normal width*height = " << cloud_normal->width * cloud_normal->height << std::endl;
  std::cout << "cloud_normal size = " << cloud_normal->size() << std::endl;
  

  Vector3f  obj_normal = get_normal_mean(cloud_normal);
  
  //----------------------------------------//
  // Get Angle of Cam Normal to Object Normal //
  //----------------------------------------//
  Vector3f  cam_normal(0.0,0.0,1.0);
  Matrix3f  R;

  if(obj_normal [2] < 0){
    obj_normal = obj_normal * (-1);
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
     <<  "(" <<  pcl::rad2deg(roll)  << "," 
     <<  pcl::rad2deg(pitch)  << "," 
     <<  pcl::rad2deg(yaw) << ")" << std::endl;

  std::cout << " center (x, y, z) = "  
  <<  "(" <<  center.x  << ","  <<  center.y  << "," <<  center.z << ")" << std::endl;

  std::cout << " (x, y, z) = "  
  <<  "(" <<  x  << ","  <<  y  << "," <<  z << ")" << std::endl;

  
#ifdef ShowCloud
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  
  Vector3f  new_vec;
  new_vec = R * cam_normal;

  vis_normal(viewer, cloud_near_center, cloud_normal);
  vis_one_point(viewer, center);
  vis_cloud(viewer, cloud_near_center);

  //viewer->addLine (pcl::PointXYZ(0, 0, 0),
  //pcl::PointXYZ(new_vec[0],new_vec[1],new_vec[2])          );
  pcl::PointXYZ p_new;
  p_new.x = center.x + new_vec[0];
  p_new.y = center.y + new_vec[1];
  p_new.z = center.z + new_vec[2];


//-----------------TEST-------------------//
  std::cout << " ==============Test=============="  << std::endl;

  Eigen::Affine3f tf = Eigen::Affine3f::Identity();
  tf.rotate (Eigen::AngleAxisf ( yaw,   Eigen::Vector3f::UnitZ()));
  tf.rotate (Eigen::AngleAxisf ( pitch, Eigen::Vector3f::UnitY()));
  tf.rotate (Eigen::AngleAxisf ( roll,  Eigen::Vector3f::UnitX()));

  std::cout << " obj_normal = " <<  obj_normal << std::endl;

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

  std::cout << " center = " <<  center_vec << std::endl;
  std::cout << " after_rotate_center_with_neg= " <<  after_rotate_center_with_neg << std::endl;

#endif 
}
