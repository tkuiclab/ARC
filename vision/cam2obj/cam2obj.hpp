#include "pcl_utility.hpp"

#include <stdio.h>
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
#include <pcl/filters/extract_indices.h>



int g_argc;
char** g_argv;

void pass_through_from_arg(PCT::Ptr cloud_in, 
            int argc,  char **argv,
            PCT::Ptr cloud_out){

  float tool_z = 0.26; 
  float pass_z_max = 0.60;

  float pass_z_min = tool_z;
  float pass_x_min, pass_x_max, pass_y_min, pass_y_max;
  pass_x_min = pass_x_max = pass_y_min = pass_y_max = 0;


  if (pcl::console::find_switch (argc, argv, "-pass_x_min")){
    pcl::console::parse (argc, argv, "-pass_x_min", pass_x_min);
    std::cout << "Use pass_x_min =" << pass_x_min << std::endl;
  }

  if (pcl::console::find_switch (argc, argv, "-pass_x_max")){
    pcl::console::parse (argc, argv, "-pass_x_max", pass_x_max);
   
    std::cout << "Use pass_x_max =" << pass_x_max << std::endl;
  }

  if (pcl::console::find_switch (argc, argv, "-pass_y_min")){
    pcl::console::parse (argc, argv, "-pass_y_min", pass_y_min);
    // ROS_INFO("Use pass_y_min = %lf",pass_y_min);
    std::cout << "Use pass_y_min =" << pass_y_min << std::endl;
  }
  
  if (pcl::console::find_switch (argc, argv, "-pass_y_max")){
    pcl::console::parse (argc, argv, "-pass_y_max", pass_y_max);
    //ROS_INFO("Use pass_y_max = %lf",pass_y_max);

    std::cout << "Use pass_y_max =" << pass_y_max << std::endl;
  }

  if (pcl::console::find_switch (argc, argv, "-pass_z_min")){
    pcl::console::parse (argc, argv, "-pass_z_min", pass_z_min);

  }

  if (pcl::console::find_switch (argc, argv, "-pass_z_max")){
    pcl::console::parse (argc, argv, "-pass_z_max", pass_z_max);
    
  }
  //ROS_INFO("Use pass_z_max = %lf",pass_z_max);
  std::cout << "Use pass_z_min =" << pass_z_min << std::endl;
  std::cout << "Use pass_z_max =" << pass_z_max << std::endl;

  get_pass_through_points(cloud_in,  cloud_out,
                        pass_x_min, pass_x_max,
                        pass_y_min, pass_y_max,
                        tool_z, pass_z_max
                     
                        );


}
Vector3f del_out_mean_normal(PC_Normal::Ptr i_cloud, PC_Normal::Ptr o_cloud){
  Vector3f  obj_normal = get_normal_mean(i_cloud);
  
   printf("obj_normal=(%lf,%lf,%lf)\n", 
      obj_normal[0],
      obj_normal[1],
      obj_normal[2]);

  float diff = 0;
  float sum_diff = 0;
  for(int i =0;i < i_cloud->size();i++){
    diff = 0;
    diff += fabs(obj_normal[0] - i_cloud->points[i].normal_x );
    diff += fabs(obj_normal[1] - i_cloud->points[i].normal_y );
    diff += fabs(obj_normal[2] - i_cloud->points[i].normal_z );
  
    printf("[%4d] -> (%lf,%lf,%lf) -> %lf\n",
      i, 
      i_cloud->points[i].normal_x,
      i_cloud->points[i].normal_y,
      i_cloud->points[i].normal_z,
      diff);
    sum_diff+=diff;
  }
  float mean_diff = sum_diff / i_cloud->size();
  float want_below_diff = mean_diff * 0.9;

  printf("mean_diff=%lf\n",mean_diff);
  printf("want_below_diff=%lf\n",want_below_diff);

  std::vector<int> want_indices;
  
  for(int i =0;i < i_cloud->size();i++){
      diff = 0;
      diff += fabs(obj_normal[0] - i_cloud->points[i].normal_x );
      diff += fabs(obj_normal[1] - i_cloud->points[i].normal_y );
      diff += fabs(obj_normal[2] - i_cloud->points[i].normal_z );
      
      if(diff < want_below_diff){
         printf("[%4d] -> (%lf,%lf,%lf) -> %lf\n",
      i, 
      i_cloud->points[i].normal_x,
      i_cloud->points[i].normal_y,
      i_cloud->points[i].normal_z,
      diff);
        want_indices.push_back(i);
      }
  }
  
  pcl::PointIndices::Ptr pointIndices(new pcl::PointIndices);
  pointIndices->indices=want_indices;


  pcl::ExtractIndices<PNormal> extract;
  extract.setInputCloud (i_cloud);
  extract.setIndices (pointIndices);
  extract.setNegative (false);
  extract.filter (*o_cloud);


  std::cout << "want_below_diff size = " << o_cloud->size() << std::endl;
  

  obj_normal = get_normal_mean(o_cloud);
   printf("NEW obj_normal=(%lf,%lf,%lf)\n", 
      obj_normal[0],
      obj_normal[1],
      obj_normal[2]);
  // float diff = 0;
  // for(int i =0;i < cloud_normal->size();i++){
  //   diff = 0;
  //   diff += fabs(obj_normal[0] - cloud_normal->points[i].normal_x );
  //   diff += fabs(obj_normal[1] - cloud_normal->points[i].normal_y );
  //   diff += fabs(obj_normal[2] - cloud_normal->points[i].normal_z );
  
  //   printf("[%4d] -> (%lf,%lf,%lf) -> %lf\n",
  //     i, 
  //     cloud_normal->points[i].normal_x,
  //     cloud_normal->points[i].normal_y,
  //     cloud_normal->points[i].normal_z,
  //     diff);
    
  // }
  return obj_normal;
}
//get camera center to object center transform
void cam_2_obj_center(PCT::Ptr i_cloud,
          double &x, double &y, double &z,
          double &roll, double &pitch, double &yaw,
          float near_points_percent = 0.1){
  PCT::Ptr cloud (new PCT);
  PCT::Ptr cloud_near_center (new PCT);
  PC_Normal::Ptr cloud_normal (new PC_Normal);
  PC_Normal::Ptr del_normal(new PC_Normal);

  std::cout << "cam_2_obj_center() say Original Points = " << i_cloud->size() << std::endl;

  //get center of colud
  PT center =  getCenter(i_cloud);
  std::cout << "Center Point = " << center << std::endl;


  //get near center points
  //float near_points_percent = 0.1;
  //------------------get_near_points----------------//
  
  std::cout << "Use Near Points Percent = " << (near_points_percent*100) << "%" << std::endl;
  get_near_points(i_cloud, center, i_cloud->size() * near_points_percent, cloud);
  
  std::cout << "cam_2_obj_center() say After get_near_points() Points = " << cloud->size() << std::endl;
  
#ifdef SaveCloud
  write_pcd(cloud,"_near_points.pcd");
#endif

  //------------------VoxelGrid (>3000)----------------//
  
  if(cloud->size() > 5000){
    float leaf = 0.01;
    // if (pcl::console::find_switch (argc, argv, "-leaf")){
    //   pcl::console::parse (argc, argv, "-leaf", leaf);
    // }

    pcl::VoxelGrid<PT> vg;
    vg.setInputCloud (cloud);
    vg.setLeafSize (leaf, leaf, leaf);
    vg.filter (*cloud);
    std::cout << "cam_2_obj_center() say After VoxelGrid Points = " << cloud->size() << std::endl;
  
#ifdef SaveCloud
  write_pcd(cloud,"_vg.pcd");
#endif
  
  }
  

  //------------------MovingLeastSquares----------------//
   // Create a KD-Tree
  pcl::search::KdTree<PT>::Ptr tree (new pcl::search::KdTree<PT>);
  pcl::PointCloud<PNormal> mls_points;
  pcl::MovingLeastSquares<PT, PNormal> mls;
  mls.setComputeNormals (true);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.03);
  mls.setInputCloud (cloud);
  mls.process (*cloud_normal);

#ifdef SaveCloud    
  write_pcd_normals(cloud_normal,"_mls.pcd");
#endif

  std::cout << "cloud_normal width*height = " << cloud_normal->width * cloud_normal->height << std::endl;
  std::cout << "cloud_normal size = " << cloud_normal->size() << std::endl;
  
  Vector3f  obj_normal = del_out_mean_normal(cloud_normal,del_normal);
  
#ifdef SaveCloud
  write_pcd_normals(del_normal,"_del_out_normal.pcd");
#endif  
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
    

  std::cout << " center (x, y, z) = "  
  <<  "(" <<  center.x  << ","  <<  center.y  << "," <<  center.z << ")" << std::endl;

  std::cout << " (x, y, z) = "  
  <<  "(" <<  x  << ","  <<  y  << "," <<  z << ")" << std::endl;

}