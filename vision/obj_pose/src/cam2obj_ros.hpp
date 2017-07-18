#include "object_pose_auxiliary.hpp"

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
#include <pcl/common/common_headers.h>
//#include <pcl/surface/impl/mls.hpp>
#include <pcl/surface/mls.h>
#include <pcl/filters/extract_indices.h>


using namespace std;
using namespace Eigen;

int g_argc;
char** g_argv;


PT getCenter( PCT::Ptr cloud){
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid (*cloud, centroid);
  
  PT p;
  p.x = centroid[0];
  p.y = centroid[1];
  p.z = centroid[2];
  
  return p;

}


void get_near_points(PCT::Ptr cloud_in, 
            PT searchPoint,
            int K ,
            PCT::Ptr cloud_out){

  //-----------Get near pointS --------------//
  pcl::KdTreeFLANN<PT> kdtree;
  kdtree.setInputCloud (cloud_in);


  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  // std::cout << "K nearest neighbor search at (" << searchPoint.x 
  //           << " " << searchPoint.y 
  //           << " " << searchPoint.z
  //           << ") with K=" << K << std::endl;

  if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
  {
    // for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
    //   std::cout << "    "  <<   cloud_in->points[ pointIdxNKNSearch[i] ].x 
    //             << " " << cloud_in->points[ pointIdxNKNSearch[i] ].y 
    //             << " " << cloud_in->points[ pointIdxNKNSearch[i] ].z 
    //             << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
    
  }else{
    std::cout << "kdtree.nearestKSearch  ERROR!!!!!!!!!" << std::endl;

  }

  //return pointIdxNKNSearch;

  pcl::PointIndices::Ptr pointIndices(new pcl::PointIndices);

  //here, I need to set the points from pointIdxNkNSearch to the PoinIndices: inliers 
  pointIndices->indices=pointIdxNKNSearch;

  pcl::ExtractIndices<PT> extract;

  // Extract the inliers
  extract.setInputCloud (cloud_in);
  extract.setIndices (pointIndices);
  extract.setNegative (false);
  extract.filter (*cloud_out);
  std::cerr << "PointCloud representing the planar component: " << cloud_out->width * cloud_out->height << " data points." << std::endl;
}


void get_pass_through_points(PCT::Ptr cloud_in,
            PCT::Ptr cloud_out,
            float min_x, float max_x,
            float min_y, float max_y,
            float min_z, float max_z
            ){
  
  PCT::Ptr now_cloud  (new PCT);;
  *now_cloud = *cloud_in;

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<PT> pass;

  if(min_z!=0 || max_z!=0){
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (min_z, max_z);

    //std::cout << "PT->min_z= " << min_z << ", max_z = " << max_z << std::endl;


    pass.setInputCloud (now_cloud);
    pass.filter (*now_cloud);

  }
  if(min_y!=0 || max_y!=0){
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (min_y, max_y);

    //std::cout << "PT->min_y= " << min_y << ", max_y = " << max_y << std::endl;

    pass.setInputCloud (now_cloud);
    pass.filter (*now_cloud);
  }

  if(min_x!=0 || max_x!=0){
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (min_x, max_x);

    //std::cout << "PT->min_x= " << min_x << ", max_x = " << max_x << std::endl;


    pass.setInputCloud (now_cloud);
    pass.filter (*now_cloud);
  }

  
  *cloud_out = *now_cloud;
}

void get_pass_xyz_from_arg( int argc,  char **argv,
            float& pass_x_min, float& pass_x_max, 
            float& pass_y_min, float& pass_y_max,
            float& pass_z_min, float& pass_z_max){

  float tool_z = 0.26;

  pass_z_max = 0.60;
  pass_z_min = tool_z;

  pass_x_min = -0.5;
  pass_x_max =  0.5;
  pass_y_min = -0.5;
  pass_y_max =  0.5;


  if (pcl::console::find_switch (argc, argv, "-pass_x_min")){
    pcl::console::parse (argc, argv, "-pass_x_min", pass_x_min);
  }

  if (pcl::console::find_switch (argc, argv, "-pass_x_max")){
    pcl::console::parse (argc, argv, "-pass_x_max", pass_x_max);
  }

  if (pcl::console::find_switch (argc, argv, "-pass_y_min")){
    pcl::console::parse (argc, argv, "-pass_y_min", pass_y_min);
  }
  
  if (pcl::console::find_switch (argc, argv, "-pass_y_max")){
    pcl::console::parse (argc, argv, "-pass_y_max", pass_y_max);
  }

  if (pcl::console::find_switch (argc, argv, "-pass_z_min")){
    pcl::console::parse (argc, argv, "-pass_z_min", pass_z_min);

  }

  if (pcl::console::find_switch (argc, argv, "-pass_z_max")){
    pcl::console::parse (argc, argv, "-pass_z_max", pass_z_max);
  }

}

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
void cam_normal_2_obj_normal(Vector3f obj_normal,
float& roll, float& pitch){ 
//,float& cam_roll, float &cam_pitch){

  Vector3f cam_normal(0,0,1);
  float cam_abs = 1 ;  // sqrt( dot(cam_normal,cam_normal) )

  //------------Roll--------------//
  Vector3f xp( 0, obj_normal[1], obj_normal[2]);//obj in x plane
  float xp_abs = sqrt( xp.dot(xp) );
  float cos_th_xp = cam_normal.dot(xp) / xp_abs ; //* cam_abs;
  float r_rad = acos(cos_th_xp);

  r_rad = (xp[1] > 0.0) ? (-1.0) * r_rad : r_rad;

  //------------Pitch--------------//
  //rotate obj_normal with -r_rad
  Eigen::Affine3f tf = Eigen::Affine3f::Identity();
  tf.rotate (Eigen::AngleAxisf ( -r_rad,   Eigen::Vector3f::UnitX()));
  Vector3f new_obj_normal = tf * obj_normal;
 
  Vector3f yp( new_obj_normal[0], 0 , new_obj_normal[2]);//obj in y plane
  float yp_abs = sqrt( yp.dot(yp) );
  float cos_th_yp = cam_normal.dot(yp) / yp_abs;// * cam_abs;
  float p_rad = acos(cos_th_yp);
  p_rad = (yp[0] < 0.0) ? (-1.0) * p_rad : p_rad;  
 

  roll = r_rad;
  pitch = p_rad;
  
  printf("(roll,pitch)=(%lf,%lf)\n", pcl::rad2deg( roll), pcl::rad2deg( pitch));
  printf("xp=(%lf,%lf,%lf),roll=%lf\n", xp[0], xp[1], xp[2],pcl::rad2deg( roll));
  printf("yp=(%lf,%lf,%lf),pitch=%lf\n", yp[0], yp[1], yp[2],pcl::rad2deg( pitch));
  
}

//rotation_with_tool 
//input: obj_normal
//output: roll, pitch (degree)
void rotation_with_tool(Vector3f obj_normal,
float& yaw, float& roll){ 
  //yaw (z), roll(x)
  Vector3f cam_normal(0,0,1);
  Vector3f y_normal(0,1,0);
  float y_abs = 1 ;  // sqrt( dot(cam_normal,cam_normal) )

  //------------Tool_Yaw--------------//
  Vector3f xyp(obj_normal[0] , obj_normal[1], 0);//obj in xy plane (z=0)
  float xyp_abs = sqrt( xyp.dot(xyp) );
  float cos_th_xyp = y_normal.dot(xyp) / xyp_abs ; //* cam_abs;
  float yaw_rad = acos(cos_th_xyp);

  yaw_rad = (xyp[0] > 0.0) ? (-1.0) * yaw_rad : yaw_rad;

  
  //------------Tool_Roll--------------//
  //rotate obj_normal with -r_rad
  Eigen::Affine3f tf = Eigen::Affine3f::Identity();
  tf.rotate (Eigen::AngleAxisf ( -yaw_rad,   Eigen::Vector3f::UnitZ()));
  Vector3f new_obj_normal = tf * obj_normal;
 
  Vector3f yzp( 0, new_obj_normal[1], new_obj_normal[2]);//obj in x=0, yz plane
  float yzp_abs = sqrt( yzp.dot(yzp) );
  float cos_th_yzp = cam_normal.dot(yzp) / yzp_abs ; //* cam_abs;
  float roll_rad = acos(cos_th_yzp);

  roll_rad = (yzp[1] > 0.0) ? (-1.0) * roll_rad : roll_rad;
 

  yaw = yaw_rad;
  roll = roll_rad;

  printf("obj_normal=(%lf,%lf,%lf)\n", 
      obj_normal[0],
      obj_normal[1],
      obj_normal[2]);
  
  printf("(yaw,roll)=(%lf,%lf)\n", pcl::rad2deg( yaw), pcl::rad2deg( roll));
  printf("xyp=(%lf,%lf,%lf),yaw=%lf\n", xyp[0], xyp[1], xyp[2],pcl::rad2deg( yaw));
  printf("yzp=(%lf,%lf,%lf),roll=%lf\n", yzp[0], yzp[1], yzp[2],pcl::rad2deg( roll));
  
}

Vector3f get_normal_mean(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normal){
 
  Vector3f obj_normal ;

  obj_normal[0] = obj_normal[1] = obj_normal[2] = 0.0f;
  
  for(int i =0;i < cloud_normal->size();i++){
    obj_normal [0] += cloud_normal->points[i].normal_x;
    obj_normal [1] += cloud_normal->points[i].normal_y;
    obj_normal [2] += cloud_normal->points[i].normal_z;
  }

  obj_normal [0] = obj_normal [0] / cloud_normal->size();
  obj_normal [1] = obj_normal [1] / cloud_normal->size();
  obj_normal [2] = obj_normal [2] / cloud_normal->size();
  
  return obj_normal;
}

Vector3f del_out_mean_normal(PC_NT::Ptr i_cloud, PC_NT::Ptr o_cloud){
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


  pcl::ExtractIndices<PointNT> extract;
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
          double &nx, double &ny, double &nz,
          float near_points_percent = 0.1){
  PCT::Ptr cloud (new PCT);
  PCT::Ptr cloud_near_center (new PCT);
  PC_NT::Ptr cloud_normal (new PC_NT);
  PC_NT::Ptr del_normal(new PC_NT);

  //get center of colud
  PT center =  getCenter(i_cloud);
  //std::cout << "Center Point = " << center << std::endl;


  //------------------get_near_points----------------//
  get_near_points(i_cloud, center, i_cloud->size() * near_points_percent, cloud);
  
  // std::cout << "Near Points Percent = " << (near_points_percent*100) << "%" 
  //     << ", get_near_points() Points = " << cloud->size()  << std::endl;

#ifdef SaveCloud
  write_pcd_2_rospack(cloud,"_near_points.pcd");
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
  write_pcd_2_rospack(cloud,"_vg.pcd");
#endif
  
  }
  

  //------------------MovingLeastSquares----------------//
   // Create a KD-Tree
  pcl::search::KdTree<PT>::Ptr tree (new pcl::search::KdTree<PT>);
  pcl::PointCloud<PointNT> mls_points;
  pcl::MovingLeastSquares<PT, PointNT> mls;
  mls.setComputeNormals (true);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.03);
  mls.setInputCloud (cloud);
  mls.process (*cloud_normal);

#ifdef SaveCloud    
  write_pcd_2_rospack_normals(cloud_normal,"_mls.pcd");
#endif

  //std::cout << "cloud_normal size = " << cloud_normal->size() << std::endl;
  
  Vector3f  obj_normal = del_out_mean_normal(cloud_normal,del_normal);
  
#ifdef SaveCloud
  write_pcd_2_rospack_normals(del_normal,"_del_out_normal.pcd");
#endif  
  //----------------------------------------//
  // Get Angle of Cam Normal to Object Normal //
  //----------------------------------------//
  Vector3f  cam_normal(0.0,0.0,1.0);

  //std::cout << " obj_normal -> " << obj_normal << std::endl;  
  // if(obj_normal [2] < 0){
  //   obj_normal = obj_normal * (-1);

  //   std::cout << " Update  obj_normal -> " << obj_normal << std::endl;  
  // }

  if(obj_normal [2] > 0){
    obj_normal = obj_normal * (-1);

    std::cout << " Update  obj_normal -> " << obj_normal << std::endl;  
  }

  float tool_yaw , tool_roll ;
  rotation_with_tool(obj_normal, tool_yaw, tool_roll );

// std::cout << " (yaw, roll) = "   <<  
//       "("  << (tool_yaw) << "," << 
//       (tool_roll) << ")"  << std::endl;

  std::cout << " (yaw, roll) = "   <<  
      "("  << pcl::rad2deg(tool_yaw) << "," << 
      pcl::rad2deg(tool_roll) << ")"  << std::endl;

  float real_tool_yaw = (tool_yaw > 0) ? (tool_yaw-M_PI) : (tool_yaw+M_PI);
  float real_tool_roll = 90 - (tool_roll + 180);
   std::cout << " (real_tool_yaw, real_tool_roll) = "   <<  
      "("  << pcl::rad2deg(real_tool_yaw) << "," << 
      pcl::rad2deg(real_tool_roll) << ")"  << std::endl;


  yaw = tool_yaw;
  roll = tool_roll;
  pitch = 0;
  
  // Eigen::Affine3f tf_neg = Eigen::Affine3f::Identity();
  // tf_neg.rotate (Eigen::AngleAxisf ( -yaw, Eigen::Vector3f::UnitZ()));
  // tf_neg.rotate (Eigen::AngleAxisf ( -roll,  Eigen::Vector3f::UnitX()));

  
  // Vector3f center_vec(center.x, center.y, center.z);
  // Vector3f after_rotate_center_with_neg;

  // after_rotate_center_with_neg = tf_neg * center_vec;
  // float tool_trans_x = after_rotate_center_with_neg[0];
  // float tool_trans_y = after_rotate_center_with_neg[1];
  // float tool_trans_z = after_rotate_center_with_neg[2];

  // x = tool_trans_x;
  // y = tool_trans_y;
  // z = tool_trans_z;
  x = center.x;
  y = center.y;
  z = center.z;

  nx = obj_normal[0];
  ny = obj_normal[1];
  nz = obj_normal[2];

  return;

  // ----cam_normal_2_obj_normal()-------//
  float r, p ;//, cam_r, cam_p;
  cam_normal_2_obj_normal(obj_normal, r ,p ); //, cam_r, cam_p);
  //std::cout << " (r, p) = "   <<  "("  << pcl::rad2deg(r) << "," << pcl::rad2deg(p) << ")"  << std::endl;
  
  //output rotation
  roll = r;
  pitch = p;
  yaw = 0;

/*
  std::cout << "-------Move------" << std::endl;  

  //------rotate center----------//
  tf_neg = Eigen::Affine3f::Identity();
  //tf_neg.rotate (Eigen::AngleAxisf ( -yaw,   Eigen::Vector3f::UnitZ()));
  tf_neg.rotate (Eigen::AngleAxisf ( -p, Eigen::Vector3f::UnitY()));
  tf_neg.rotate (Eigen::AngleAxisf ( -r,  Eigen::Vector3f::UnitX()));

  
  Vector3f center_vec(center.x, center.y, center.z);
  Vector3f after_rotate_center_with_neg;

  after_rotate_center_with_neg = tf_neg * center_vec;
  float c_trans_x = after_rotate_center_with_neg[0];
  float c_trans_y = after_rotate_center_with_neg[1];
  float c_trans_z = after_rotate_center_with_neg[2];

  // std::cout << " center (x, y, z) = "  
  // <<  "(" <<  center.x  << ","  <<  center.y  << "," <<  center.z << ")" << std::endl;

  std::cout << "c_trans (x, y, z) = "  
  <<  "(" <<  c_trans_x  << ","  <<  c_trans_y  << "," <<  c_trans_z << ")" << std::endl;


  x = c_trans_x;
  y = c_trans_y;
  z = c_trans_z;


  //------Record Move -----//
  float cam2tool_y = -0.11;  //#-0.095  #cam axis
  float cam2tool_z = 0.23;   //# + 0.035
  
  float move_cam_x = c_trans_x;
  float move_cam_y = c_trans_y - cam2tool_y;
  float move_cam_z = c_trans_z - cam2tool_z;


  printf("----------Test move cam----------\n");
  printf("task.Arm.relative_rot_nsa(pitch = %lf, yaw = %lf)\n",
      pcl::rad2deg(roll), pcl::rad2deg(pitch));
  printf("task.Arm.relative_move_nsa(n= %lf, s = %lf, a = %lf -obj_dis)\n",
      move_cam_y, move_cam_x, move_cam_z);
*/
    
}


bool get_center_from_2dbox(
    PCT::Ptr i_cloud,
    int mini_x,int mini_y,
    int max_x, int max_y, 
    //pass_through_z
    float pt_min_z, float pt_max_z,
    float& center_y, float& center_z){


    float sum_z = 0;
    float sum_y = 0;
    unsigned int  cp = 0 ;
    int index;
    for(int j=mini_y;j<max_y;j++){
        for(int i=mini_x;i<max_x;i++)  {

          index = j*i_cloud->width+i;
          if (pcl::isFinite (i_cloud->points[index])) {
            float y = i_cloud->points[index].y;
            float z = i_cloud->points[index].z;
            if(z > pt_min_z &&  z < pt_max_z 
               ){
              sum_z += z;
              sum_y += y;
              ++cp;
            }
          }
        }
    }

  
  if(cp > 0 ){

    center_y = sum_y/(float)cp;
    center_z =  sum_z / (float)cp;
    return true;
  }else{
    center_z = -1;
    return false;
  }
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