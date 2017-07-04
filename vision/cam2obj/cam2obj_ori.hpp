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
    //std::cout << "Use pass_x_min =" << pass_x_min << std::endl;
  }

  if (pcl::console::find_switch (argc, argv, "-pass_x_max")){
    pcl::console::parse (argc, argv, "-pass_x_max", pass_x_max);
    //std::cout << "Use pass_x_max =" << pass_x_max << std::endl;
  }

  if (pcl::console::find_switch (argc, argv, "-pass_y_min")){
    pcl::console::parse (argc, argv, "-pass_y_min", pass_y_min);
    // ROS_INFO("Use pass_y_min = %lf",pass_y_min);
   // std::cout << "Use pass_y_min =" << pass_y_min << std::endl;
  }
  
  if (pcl::console::find_switch (argc, argv, "-pass_y_max")){
    pcl::console::parse (argc, argv, "-pass_y_max", pass_y_max);
    //ROS_INFO("Use pass_y_max = %lf",pass_y_max);
    //std::cout << "Use pass_y_max =" << pass_y_max << std::endl;
  }

  if (pcl::console::find_switch (argc, argv, "-pass_z_min")){
    pcl::console::parse (argc, argv, "-pass_z_min", pass_z_min);

  }

  if (pcl::console::find_switch (argc, argv, "-pass_z_max")){
    pcl::console::parse (argc, argv, "-pass_z_max", pass_z_max);
    
  }
  //ROS_INFO("Use pass_z_max = %lf",pass_z_max);
  //std::cout << "Use pass_z_min =" << pass_z_min << std::endl;
  //std::cout << "Use pass_z_max =" << pass_z_max << std::endl;

  get_pass_through_points(cloud_in,  cloud_out,
                        pass_x_min, pass_x_max,
                        pass_y_min, pass_y_max,
                        tool_z, pass_z_max
                     
                        );
}


//cam_normal_2_obj_normal 
//input: obj_normal
//output: roll, pitch (degree)
void cam_normal_2_obj_normal(Vector3f obj_normal,float& roll, float& pitch){

  Vector3f cam_normal(0,0,1);
  float cam_abs = 1 ;  // sqrt( dot(cam_normal,cam_normal) )

  //------------Roll--------------//
  Vector3f xp( 0, obj_normal[1], obj_normal[2]);//obj in x plane
  float xp_abs = sqrt( xp.dot(xp) );
  float cos_th_xp = cam_normal.dot(xp) / xp_abs ; //* cam_abs;
  float r_rad = acos(cos_th_xp);

  //------------Pitch--------------//
  //rotate obj_normal with -r_rad
  Eigen::Affine3f tf = Eigen::Affine3f::Identity();
  tf.rotate (Eigen::AngleAxisf ( -r_rad,   Eigen::Vector3f::UnitX()));
  Vector3f new_obj_normal = tf * obj_normal;
 
  Vector3f yp( new_obj_normal[0], 0 , new_obj_normal[2]);//obj in y plane
  float yp_abs = sqrt( yp.dot(yp) );
  float cos_th_yp = cam_normal.dot(yp) / yp_abs;// * cam_abs;
  float p_rad = acos(cos_th_yp);


  printf("xp=(%lf,%lf,%lf),roll=%lf\n", xp[0], xp[1], xp[2],roll);
  printf("yp=(%lf,%lf,%lf),pitch=%lf\n", yp[0], yp[1], yp[2],pitch);

  printf("ORI (roll,pitch)=(%lf,%lf)\n", pcl::rad2deg( r_rad), pcl::rad2deg( p_rad));

  roll = (xp[1] < 0) ? (-1) * r_rad : r_rad;
  pitch = (yp[0] > 0) ? (-1) * p_rad : p_rad;


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
  
    // printf("[%4d] -> (%lf,%lf,%lf) -> %lf\n",
    //   i, 
    //   i_cloud->points[i].normal_x,
    //   i_cloud->points[i].normal_y,
    //   i_cloud->points[i].normal_z,
    //   diff);
    sum_diff+=diff;
  }
  float mean_diff = sum_diff / i_cloud->size();
  float want_below_diff = mean_diff * 0.9;

  // printf("mean_diff=%lf\n",mean_diff);
  // printf("want_below_diff=%lf\n",want_below_diff);

  std::vector<int> want_indices;
  
  for(int i =0;i < i_cloud->size();i++){
      diff = 0;
      diff += fabs(obj_normal[0] - i_cloud->points[i].normal_x );
      diff += fabs(obj_normal[1] - i_cloud->points[i].normal_y );
      diff += fabs(obj_normal[2] - i_cloud->points[i].normal_z );
      
      if(diff < want_below_diff){
      //    printf("[%4d] -> (%lf,%lf,%lf) -> %lf\n",
      // i, 
      // i_cloud->points[i].normal_x,
      // i_cloud->points[i].normal_y,
      // i_cloud->points[i].normal_z,
      // diff);
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


  // std::cout << "want_below_diff size = " << o_cloud->size() << std::endl;
  

  obj_normal = get_normal_mean(o_cloud);
  //  printf("NEW obj_normal=(%lf,%lf,%lf)\n", 
  //     obj_normal[0],
  //     obj_normal[1],
  //     obj_normal[2]);

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

  //get center of colud
  PT center =  getCenter(i_cloud);
  //std::cout << "Center Point = " << center << std::endl;


  //------------------get_near_points----------------//
  get_near_points(i_cloud, center, i_cloud->size() * near_points_percent, cloud);
  
  // std::cout << "Near Points Percent = " << (near_points_percent*100) << "%" 
  //     << ", get_near_points() Points = " << cloud->size()  << std::endl;

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

  //std::cout << "cloud_normal size = " << cloud_normal->size() << std::endl;
  
  Vector3f  obj_normal = del_out_mean_normal(cloud_normal,del_normal);
  
#ifdef SaveCloud
  write_pcd_normals(del_normal,"_del_out_normal.pcd");
#endif  
  //----------------------------------------//
  // Get Angle of Cam Normal to Object Normal //
  //----------------------------------------//
  Vector3f  cam_normal(0.0,0.0,1.0);
  Matrix3f  R;

  //std::cout << " obj_normal -> " << obj_normal << std::endl;  
  if(obj_normal [2] < 0){
    obj_normal = obj_normal * (-1);

    std::cout << " Update  obj_normal -> " << obj_normal << std::endl;  
  }

  // ----Calculate Rotate from  cam_normal to obj_normal-------//
  R = Quaternionf().setFromTwoVectors(cam_normal,obj_normal);
  Vector3f euler = R.eulerAngles(0, 1, 2);
  yaw = euler[2]; pitch = euler[1]; roll = euler[0];
  // std::cout << " (roll, pitch, yaw) = "  
  //    <<  "(" <<  pcl::rad2deg(roll)  << "," 
  //    <<  pcl::rad2deg(pitch)  << "," 
  //    <<  pcl::rad2deg(yaw) << ")" << std::endl; 

  
  float r, p;
  cam_normal_2_obj_normal(obj_normal, r ,p);
  std::cout << " (r, p) = "   <<  "("  << pcl::rad2deg(r) << "," << pcl::rad2deg(p) << ")"  << std::endl;
  
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
    

  // std::cout << " center (x, y, z) = "  
  // <<  "(" <<  center.x  << ","  <<  center.y  << "," <<  center.z << ")" << std::endl;

  // std::cout << " (x, y, z) = "  
  // <<  "(" <<  x  << ","  <<  y  << "," <<  z << ")" << std::endl;

}



void only_obj_center(PCT::Ptr i_cloud,
          double &x, double &y, double &z){
  PT center =  getCenter(i_cloud);
  std::cout << "Center Point = " << center << std::endl;

  x = center.x;
  y = center.y;
  z = center.z;

  return;

}