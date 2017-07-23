// Stdlib
#include <stdlib.h>
#include <cmath>
#include <limits.h>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>

// PCL input/output
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

//PCL other
#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>
#include <pcl/segmentation/supervoxel_clustering.h>

// The segmentation class this example is for
#include <pcl/segmentation/cpc_segmentation.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/filters/extract_indices.h>

// #include "cam2obj_ros.hpp"
#include "object_pose_auxiliary.hpp"

// VTK
// #include <vtkImageReader2Factory.h>
// #include <vtkImageReader2.h>
// #include <vtkImageData.h>
// #include <vtkImageFlip.h>
// #include <vtkPolyLine.h>

/// *****  Type Definitions ***** ///

//typedef pcl::PointXYZRGB PT;  // The point type used for input
typedef pcl::LCCPSegmentation<PT>::SupervoxelAdjacencyList SuperVoxelAdjacencyList;
/// Callback and variables

bool show_normals = false, normals_changed = false;
bool show_adjacency = false, line_changed = false;
bool show_supervoxels = false;
bool show_segmentation = true;
bool show_help = true;
bool bg_white = false;
float normals_scale;
float line_width = 2.0f;
float textcolor;

class CPCSegmentation
{
  public:
    CPCSegmentation()
    {
      scene_segmentation=false;
    }
    void setPointCloud(pcl::PointCloud<PT>::Ptr input_cloud_ptr)
    {
      input_cloud_ptr_ = input_cloud_ptr;
    }
    void set_3D_ROI(PT min_p, PT max_p)
    {
      scene_segmentation=true;
      max_points = max_p;
      min_points = min_p;
      //std::cout << "max_x = \t" << max_p.x << "max_y = \t" << max_p.y << "max_z = " << max_p.z << std::endl;
      //std::cout << "min_x = \t" << min_p.x << "min_y = \t" << min_p.y << "min_z = " << min_p.z << std::endl;
    }


    void do_CPC(){
      /// -----------------------------------|  Preparations  |-----------------------------------

      bool sv_output_specified;
      bool show_visualization;
      bool ignore_provided_normals;
      bool add_label_field;
      bool save_binary_pcd = false;
      bool output_specified = true;

      /// Create variables needed for preparations
      pcl::PointCloud<pcl::Normal>::Ptr input_normals_ptr (new pcl::PointCloud<pcl::Normal>);
      bool has_normals = false;

    /// -----------------------------------|  Main Computation  |-----------------------------------

      ///  Default values of parameters before parsing
      // Supervoxel Stuff
      float voxel_resolution = 0.0075f;
      float seed_resolution = 0.03f;
      float color_importance = 0;//1;
      float spatial_importance = 1.0f;
      float normal_importance = 4.0f;
      bool use_single_cam_transform = false;
      bool use_supervoxel_refinement;

      // LCCPSegmentation Stuff
      float concavity_tolerance_threshold = 10;
      float smoothness_threshold = 0.1;
      uint32_t min_segment_size = 0;

      bool use_extended_convexity=true;
      bool use_sanity_criterion=false;

      // CPCSegmentation Stuff
      float min_cut_score = 0.1;
      unsigned int max_cuts = 5;
      unsigned int cutting_min_segments = 200;
      bool use_local_constrain;
      bool use_directed_cutting;
      bool use_clean_cutting;
      unsigned int ransac_iterations = 10000;

      normals_scale = seed_resolution / 2.0;

      // Segmentation Stuff
      unsigned int k_factor = 0;
      if (use_extended_convexity)
        k_factor = 1;
      
      textcolor = bg_white?0:1;

      pcl::SupervoxelClustering<PT> super (voxel_resolution, seed_resolution);
      super.setInputCloud (input_cloud_ptr_);
      if (has_normals)
        super.setNormalCloud (input_normals_ptr);
      super.setColorImportance (color_importance);
      super.setSpatialImportance (spatial_importance);
      super.setNormalImportance (normal_importance);
      std::map<uint32_t, pcl::Supervoxel<PT>::Ptr> supervoxel_clusters;

      // PCL_INFO ("Extracting supervoxels\n");
      super.extract (supervoxel_clusters);

      if (use_supervoxel_refinement)
      {
        // PCL_INFO ("Refining supervoxels\n");
        super.refineSupervoxels (2, supervoxel_clusters);
      }
      std::stringstream temp;
      temp << "  Nr. Supervoxels: " << supervoxel_clusters.size () << "\n";
      // PCL_INFO (temp.str ().c_str ());

      // PCL_INFO ("Getting supervoxel adjacency\n");
      std::multimap<uint32_t, uint32_t>supervoxel_adjacency;
      super.getSupervoxelAdjacency (supervoxel_adjacency);

      /// Get the cloud of supervoxel centroid with normals and the colored cloud with supervoxel coloring (this is used for visulization)
      pcl::PointCloud<pcl::PointNormal>::Ptr sv_centroid_normal_cloud = pcl::SupervoxelClustering<PT>::makeSupervoxelNormalCloud (supervoxel_clusters);

      /// Set paramters for LCCP preprocessing and CPC (CPC inherits from LCCP, thus it includes LCCP's functionality)

      // PCL_INFO ("Starting Segmentation\n");
      pcl::CPCSegmentation<PT> cpc;
      cpc.setConcavityToleranceThreshold (concavity_tolerance_threshold);
      cpc.setSanityCheck (use_sanity_criterion);
      cpc.setCutting (max_cuts, cutting_min_segments, min_cut_score, use_local_constrain, use_directed_cutting, use_clean_cutting);
      cpc.setRANSACIterations (ransac_iterations);
      cpc.setSmoothnessCheck (true, voxel_resolution, seed_resolution, smoothness_threshold);
      cpc.setKFactor (k_factor);
      cpc.setInputSupervoxels (supervoxel_clusters, supervoxel_adjacency);
      cpc.setMinSegmentSize (min_segment_size);
      cpc.segment ();
      
      // PCL_INFO ("Interpolation voxel cloud -> input cloud and relabeling\n");
      pcl::PointCloud<PLabel>::Ptr sv_labeled_cloud = super.getLabeledCloud ();
      //pcl::PointCloud<PLabel>::Ptr cpc_labeled_cloud = sv_labeled_cloud->makeShared ();
      cpc_labeled_cloud = sv_labeled_cloud->makeShared ();
      cpc.relabelCloud (*cpc_labeled_cloud);
      SuperVoxelAdjacencyList sv_adjacency_list;
      cpc.getSVAdjacencyList (sv_adjacency_list);  // Needed for visualization


    }

    void unknown_highest(){
      do_CPC();
      
    }

    //map<int, int> label_map;    // index, label_count      
    void extract_with_min_size( //pcl::PointCloud<PLabel>::Ptr cpc_labeled_cloud, 
                                int min_size,
                                pcl::PointCloud<PLabel>::Ptr exceed_pt,
                                std::map<int, int> &label_map                          
                                ){
      //using namespace std;
      // map<int, int> label_map;  //<label, size)
      // map<int, int>::iterator  iter;
      for(int i=0;i<cpc_labeled_cloud->size();i++)
      {
        int l = cpc_labeled_cloud->points[i].label;
        if ( label_map.find(l) == label_map.end() ) {
            label_map[l] = 1;
          } else {
            label_map[l] = label_map[l] + 1;
          }
        //label_map.insert(pair<int, int>("r000", "student_zero"));

      }

      // int big_min_size_count = 0;
      
      // for(iter = label_map.begin(); iter != label_map.end(); iter++){
        
      //     if(iter->second >= min_size){
      //       cout<<iter->first<<" "<<iter->second<<endl;
      //       big_min_size_count++;
      //     }
      // }
      // cout<< "big_min_size_count = " << big_min_size_count <<endl;

      //get bigger 1000 points
      //pcl::PointCloud<PLabel>::Ptr exceed_pt (new pcl::PointCloud<PLabel>);
      exceed_pt->height= cpc_labeled_cloud->height; 
      exceed_pt->width= cpc_labeled_cloud->width; 
      exceed_pt->is_dense = false;
      exceed_pt->points.resize(cpc_labeled_cloud->size()); 

      for(int i=0;i<cpc_labeled_cloud->size();i++)
      {
        //cout << "cpc_labeled_cloud->points[i].label =" <<cpc_labeled_cloud->points[i].label << endl;
        if(label_map[cpc_labeled_cloud->points[i].label] >= min_size)
        {
          exceed_pt->points[i].x = cpc_labeled_cloud->points[i].x;
          exceed_pt->points[i].y = cpc_labeled_cloud->points[i].y;
          exceed_pt->points[i].z = cpc_labeled_cloud->points[i].z;
          exceed_pt->points[i].label = cpc_labeled_cloud->points[i].label;

        }else{
          exceed_pt->points[i].x = 0;
          exceed_pt->points[i].y = 0;
          exceed_pt->points[i].z = 0;
          exceed_pt->points[i].label = 0;
          
        }
      }
      //cout << "Before write " << endl;
      // pcl::PCDWriter writer;
      // writer.write<PLabel> ("_min_size.pcd", *exceed_pt, false);

      //pcl::io::savePCDFile ( "_min_size.pcd", exceed_pt, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), save_binary_pcd);
    }

    void getCloudWithLabel (pcl::PointCloud<PLabel>::Ptr label_cloud,
                            PCT::Ptr scene_cloud,
                            PCT::Ptr out,
                        int label_num){
      // find the 'label' field index
      std::vector <pcl::PCLPointField> fields;
      int label_idx = -1;
      pcl::PointCloud <PLabel> point;
      label_idx = pcl::getFieldIndex (point, "label", fields);

      if (label_idx != -1)
      {
        for (size_t i = 0; i < label_cloud->points.size (); i++)
        {
          // get the 'label' field                                                                       
          uint32_t label;
          memcpy (&label, reinterpret_cast<char*> (&label_cloud->points[i]) + fields[label_idx].offset, sizeof(uint32_t));

          if (static_cast<int> (label) == label_num)
          {
            // pcl::PointXYZ point;
            // // X Y Z
            // point.x = in->points[i].x;
            // point.y = in->points[i].y;
            // point.z = in->points[i].z;

            PT point;
            point.x = scene_cloud->points[i].x;
            point.y = scene_cloud->points[i].y;
            point.z = scene_cloud->points[i].z;
            point.r = scene_cloud->points[i].r;
            point.g = scene_cloud->points[i].g;
            point.b = scene_cloud->points[i].b;
            point.a = scene_cloud->points[i].a;

            out->points.push_back (point);
          }
        }
        out->width = static_cast<int> (out->points.size ());
        out->height = 1;
        out->is_dense = false;
      }
    }


    
    bool getCentroidWithLabel (pcl::PointCloud<PLabel>::Ptr in,int label_num, pcl::PointXYZ &center){

      float sum_x = 0;
      float sum_z = 0;
      float sum_y = 0;
      unsigned int  cp = 0 ;

      // pcl::PointXYZ center(0.0,0.0,0.0);
      center.x =  center.y =  center.z = 0.0;

      if (label_num > 0){
        for (int i = 0; i < in->points.size (); i++){
          // get the 'label' field                                                                       
          //uint32_t label;
        
          if (in->points[i].label == label_num){

            sum_x += in->points[i].x;
            sum_y += in->points[i].y;
            sum_z += in->points[i].z;
            ++cp;
          }
        }

        if(cp > 0 ){
          center.x = sum_x / (float)cp;
          center.y = sum_y /(float)cp;
          center.z = sum_z / (float)cp;
          return true;
        }else{
          std::cout << "[ERROR] say from getCentroidWithLabel() cp <= 0" << std::endl;  
          return false;
        }

      }else{
        std::cout << "[ERROR] say from getCentroidWithLabel() label <= 0" << std::endl;
        return false;
      }

    }

    void do_segmentation(){
      do_CPC();
      segmented_cloud_ptr_ = cpc_labeled_cloud;

      max_label=0;
      for(int i=0;i<cpc_labeled_cloud->size();i++)
      {
        if(cpc_labeled_cloud->points[i].label > max_label)
          max_label = cpc_labeled_cloud->points[i].label;
      }
      //std::cout << "max label: " << max_label << std::endl;
      if(scene_segmentation)
      {
        /// -----------------------------------|  Scene Segmentation  |-----------------------------------
        std::cout << "Doing Scene Segmentation!!!" << std::endl;
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster1(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>); 
        cloud_cluster = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);

        cluster->height= cpc_labeled_cloud->height; 
        cluster->width= cpc_labeled_cloud->width; 
        cluster->is_dense = false;
        cluster->points.resize(cpc_labeled_cloud->size()); 
        for(int j=1;j<max_label;j++)
        {
          for(int i=0;i<cpc_labeled_cloud->size();i++)
          {
            if(cpc_labeled_cloud->points[i].label == j)
            {
              cluster->points[i].x = cpc_labeled_cloud->points[i].x;
              cluster->points[i].y = cpc_labeled_cloud->points[i].y;
              cluster->points[i].z = cpc_labeled_cloud->points[i].z;
            }else{
              cluster->points[i].x = 0;
              cluster->points[i].y = 0;
              cluster->points[i].z = 0;
            }
          }

          // removing the extra zeroes(0,0,0) points from the point cloud 
          pcl::IndicesPtr indices_ex (new std::vector <int>); 
          pcl::PassThrough<pcl::PointXYZ> pass; 
          pass.setInputCloud (cluster); 
          pass.setFilterFieldName ("z");
          pass.setFilterLimits (0.0,0.0);
          pass.setFilterLimitsNegative (true); 
          pass.filter (*indices_ex); // storing the final result in as indices 

          
          pcl::ExtractIndices<pcl::PointXYZRGBA> extract; 
          extract.setInputCloud (input_cloud_ptr_);  // setting the color PointCloud as the input 
          
          extract.setIndices(indices_ex); // passing the indices that I want to extract from the colored PointCloud 
          extract.setNegative (false); 
          extract.filter (*cloud_cluster1); // extracted the wanted cluster from the color pointcloud 

          PT tmp_max_p, tmp_min_p;
          pcl::getMinMax3D(*cloud_cluster1, tmp_min_p, tmp_max_p);

          Eigen::Vector4f centroid;
          pcl::compute3DCentroid (*cloud_cluster1, centroid);
          
          if(tmp_max_p.x<=(max_points.x+0.02) && tmp_max_p.y<=(max_points.y+0.02) && tmp_max_p.z<=(max_points.z+0.02) &&
                        tmp_min_p.x>=(min_points.x-0.02) && tmp_min_p.y>=(min_points.y-0.02) && tmp_min_p.z>=(min_points.z-0.02))
          //if(centroid(0) <= max_points.x && centroid(1) <= max_points.y && centroid(2) <= max_points.z 
          //        && centroid(0) >= min_points.x && centroid(1) >= min_points.y && centroid(2) >= min_points.z)
          {
            //std::cout << "------------------------------------------------" << std::endl;
            //std::cout << "cluster_" << j << " save!!!" << std::endl;
            //ROS_INFO("Cluster_%d max_x = %f, max_y = %f, max_z = %f", j, tmp_max_p.x, tmp_max_p.y, tmp_max_p.z);
            //ROS_INFO("Cluster_%d min_x = %f, min_y = %f, min_z = %f\n", j, tmp_min_p.x, tmp_min_p.y, tmp_min_p.z);
            // std::cout << "The XYZ coordinates of the centroid are: ("
            //           << centroid[0] << ", "
            //           << centroid[1] << ", "
            //           << centroid[2] << ")." << std::endl;
            clusters.push_back (*cloud_cluster1);
            for(int i=0;i<cloud_cluster1->points.size();i++)
            {
              cloud_cluster->points.push_back (cloud_cluster1->points[i]);
            }
          }
        }
        
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        
        // std::stringstream ss;
        // ss << "Clusters.pcd";
        // pcl::io::savePCDFile (ss.str(), *cloud_cluster, save_binary_pcd);
        
#ifdef SaveCloud
      //write pcd
      write_pcd_2_rospack(cloud_cluster,"_Clusters.pcd");
#endif

      }else{
        /// -----------------------------------|  ROI Segmentation  |-----------------------------------
        std::cout << "Doing ROI Segmentation!!!" << std::endl;
        Max_cluster = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>); 
        Max_cluster->height= cpc_labeled_cloud->height; 
        Max_cluster->width= cpc_labeled_cloud->width; 
        Max_cluster->is_dense = false;
        Max_cluster->points.resize(cpc_labeled_cloud->size()); 
        max_label_index=0;
        int tmp_counter=0;
        for(int j=1;j<max_label;j++)
        {
          counter=0;
          for(int i=0;i<cpc_labeled_cloud->size();i++)
          {
            if(cpc_labeled_cloud->points[i].label == j)
            {
              counter++;
            }
          }
          if(tmp_counter < counter)
          {
            max_label_index = j;
            tmp_counter=counter;
          }
          //std::cout << (j-1) << " : " << counter << std::endl;
        }
        for(int i=0;i<cpc_labeled_cloud->size();i++)
        {
          if(cpc_labeled_cloud->points[i].label == max_label_index)
          {
            Max_cluster->points[i].x = cpc_labeled_cloud->points[i].x;
            Max_cluster->points[i].y = cpc_labeled_cloud->points[i].y;
            Max_cluster->points[i].z = cpc_labeled_cloud->points[i].z;
          }
        }
        // removing the extra zeroes(0,0,0) points from the point cloud 
        pcl::IndicesPtr indices_ex (new std::vector <int>); 
        pcl::PassThrough<pcl::PointXYZ> pass; 
        pass.setInputCloud (Max_cluster); 
        pass.setFilterFieldName ("z");
        pass.setFilterLimits (0.0,0.0);
        pass.setFilterLimitsNegative (true); 
        pass.filter (*indices_ex); // storing the final result in as indices 

        
        pcl::ExtractIndices<pcl::PointXYZ> extract; 
        extract.setInputCloud (Max_cluster);  // setting the color PointCloud as the input 
        
        extract.setIndices(indices_ex); // passing the indices that I want to extract from the colored PointCloud 
        extract.setNegative (false); 
        extract.filter (*Max_cluster); // extracted the wanted cluster from the color pointcloud 
       
        //pcl::io::savePCDFile ("BIG_SEG.pcd", *Max_cluster, false);
        
#ifdef SaveCloud
      write_pcd_2_rospack(cloud_cluster,"_BIG_SEG.pcd");
#endif

      }

    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr get_cloud_cluster(){return cloud_cluster;}
    pcl::PointCloud<pcl::PointXYZ>::Ptr get_BiggestCluster(){return Max_cluster;}
    pcl::PointCloud<PLabel>::Ptr getSegmentedPointCloud(){return segmented_cloud_ptr_;}
private:
   int max_label;
   int counter;
   int max_label_index;
   bool scene_segmentation;
   PT max_points, min_points;
   pcl::PCLPointCloud2 input_pointcloud2;
   pcl::PointCloud<PT>::Ptr input_cloud_ptr_;
   pcl::PointCloud<PLabel>::Ptr cpc_labeled_cloud; // same as segmented_cloud_ptr_
   pcl::PointCloud<PLabel>::Ptr segmented_cloud_ptr_;
   pcl::PointCloud<pcl::PointXYZ>::Ptr Max_cluster;
   pcl::PointCloud<pcl::PointXYZRGBA>::CloudVectorType clusters;
   pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster;
};
