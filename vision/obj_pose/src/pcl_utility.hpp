#ifndef _PCL_UTILITY_H_
#define _PCL_UTILITY_H_

#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Eigen/Geometry>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types_conversion.h>
#include <pcl/filters/passthrough.h>

using namespace Eigen;

// Types
typedef pcl::PointXYZRGB PT;       //point type
typedef pcl::PointCloud<PT> PCT;



std::string path;

#ifdef ShowCloud
//  pcl::visualization::CloudViewer viewer("Cloud Viewer");
//#else
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

  //pcl::visualization::PCLVisualizer visu("Alignment");
#endif
//(input)    ./appname xxx.pcd 
//(output)   xxx
std::string get_first_pcd_name(int argc, char** argv){
  std::vector<int> filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");

  if (filenames.size () != 1) {
      std::cout << "No PCD File Input" << std::endl;
      return "";
  }

  char* pch = strtok (argv[filenames[0]],".");
  
  return std::string(pch);
}

void vis_simple( boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
                pcl::PointCloud<PT>::ConstPtr cloud 
){

  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<PT> single_color(cloud, 255, 255, 255);
  viewer->addPointCloud<PT> (cloud, single_color, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();

}


void vis_one_point(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
                PT p , 
                std::string name = "red"){
  pcl::PointCloud<PT>::Ptr cloud (new pcl::PointCloud<PT>);
  
  //cloud_center
  cloud->width  = 1;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);
  
  cloud->points[0].x = p.x;
  cloud->points[0].y = p.y;
  cloud->points[0].z = p.z;
  
  pcl::visualization::PointCloudColorHandlerCustom<PT> single_color(cloud, 255, 0, 0);
  viewer->addPointCloud<PT> (cloud, single_color, name);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, name);
  
} 

void vis_cloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
                pcl::PointCloud<PT>::ConstPtr cloud ){
  pcl::visualization::PointCloudColorHandlerCustom<PT> single_color(cloud, 255, 0, 255);
  viewer->addPointCloud<PT> (cloud, single_color, "purple");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "purple");
  
}



void vis_normal (boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,
                pcl::PointCloud<PT>::ConstPtr cloud, 
                 pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  //viewer->setBackgroundColor (0, 0, 0);

  pcl::visualization::PointCloudColorHandlerCustom<PT> xyz (cloud, 255, 255, 255); // 0
  viewer->addPointCloud<PT> (cloud,xyz, "normal");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "normal");
  
  //pcl::visualization::PointCloudColorHandlerCustom<PT> n_color (normals, 255, 0, 255); // Red

  viewer->addPointCloudNormals<PT, pcl::Normal> (cloud, normals, 10, 1.0, "normals");
  
  
  //viewer->addCoordinateSystem (1.0);
  //viewer->initCameraParameters ();
}

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

  std::cout << "K nearest neighbor search at (" << searchPoint.x 
            << " " << searchPoint.y 
            << " " << searchPoint.z
            << ") with K=" << K << std::endl;

  if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
  {
    for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
      std::cout << "    "  <<   cloud_in->points[ pointIdxNKNSearch[i] ].x 
                << " " << cloud_in->points[ pointIdxNKNSearch[i] ].y 
                << " " << cloud_in->points[ pointIdxNKNSearch[i] ].z 
                << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
    
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


//-----------Get mean normal--------------//
Vector3f get_normal_mean(pcl::PointCloud<pcl::Normal>::Ptr cloud_normal){
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

void get_hsv_points(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, 
             pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out,
            float h_min, float h_max, 
            float s_min, float s_max,
            float v_min, float v_max,
            bool negtive = false){
           

  
  std::vector<int> want_indices;
  
  bool h_type = (h_min > h_max) ? true:false ;


  for(int i = 0 ;i < cloud_in->size();i++){
      pcl::PointXYZHSV p;
      pcl::PointXYZRGBtoXYZHSV(cloud_in->points[i],p);
      if(h_type){
        if(p.h >= h_min || p.h <= h_max && 
            p.s >= s_min && p.s <= s_max && 
            p.v >= v_min && p.v <= v_max  ){

              want_indices.push_back(i);
        }
      }else{
        if(p.h >= h_min && p.h <= h_max && 
          p.s >= s_min && p.s <= s_max && 
          p.v >= v_min && p.v <= v_max  ){

            want_indices.push_back(i);
        }
      }
  }
  
  pcl::PointIndices::Ptr pointIndices(new pcl::PointIndices);
  pointIndices->indices=want_indices;


  //extract
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud (cloud_in);
  extract.setIndices (pointIndices);
  extract.setNegative (negtive);
  extract.filter (*cloud_out);

}
void get_pass_through_points(PCT::Ptr cloud_in, 
            float min_z, float max_z,
            PCT::Ptr cloud_out){
  
  
  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::PassThrough<PT> pass;
  pass.setInputCloud (cloud_in);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (min_z, max_z);
  pass.filter (*cloud_out);

  
  /*
  pass.filter (*indices);
  //PCT::Ptr cloud_cluster (new PCT);
  for (std::vector<int>::const_iterator pit = indices->begin (); pit != indices->end (); ++pit)
    cloud_out->points.push_back (cloud_in->points[*pit]); //*
  cloud_out->width = cloud_out->points.size ();
  cloud_out->height = 1;
  cloud_out->is_dense = true;
  */
}

#endif
