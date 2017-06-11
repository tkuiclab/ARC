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
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
//PCL other
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/supervoxel_clustering.h>

// The segmentation class this example is for
#include <pcl/segmentation/cpc_segmentation.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/filters/extract_indices.h>
// VTK
#include <vtkImageReader2Factory.h>
#include <vtkImageReader2.h>
#include <vtkImageData.h>
#include <vtkImageFlip.h>
#include <vtkPolyLine.h>

/// *****  Type Definitions ***** ///

typedef pcl::PointXYZRGBA PointT;  // The point type used for input
typedef pcl::LCCPSegmentation<PointT>::SupervoxelAdjacencyList SuperVoxelAdjacencyList;
	pcl::PointCloud<PointT>::CloudVectorType clusters;

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
    CPCSegmentation(){};
    void setPointCloud(pcl::PointCloud<PointT>::Ptr input_cloud_ptr)
    {
      input_cloud_ptr_ = input_cloud_ptr;
    }

    void do_segmentation()
    {
      /// -----------------------------------|  Preparations  |-----------------------------------

      bool sv_output_specified;
      bool show_visualization;
      bool ignore_provided_normals;
      bool add_label_field;
      bool save_binary_pcd;
      bool output_specified = true;

      /// Create variables needed for preparations
      std::string outputname ("");
      pcl::PointCloud<pcl::Normal>::Ptr input_normals_ptr (new pcl::PointCloud<pcl::Normal>);
      bool has_normals = false;

    /// -----------------------------------|  Main Computation  |-----------------------------------

      ///  Default values of parameters before parsing
      // Supervoxel Stuff
      float voxel_resolution = 0.0075f;
      float seed_resolution = 0.03f;
      float color_importance = 0.0f;
      float spatial_importance = 1.0f;
      float normal_importance = 4.0f;
      bool use_single_cam_transform = false;
      bool use_supervoxel_refinement;

      // LCCPSegmentation Stuff
      float concavity_tolerance_threshold = 10;
      float smoothness_threshold = 0.1;
      uint32_t min_segment_size = 0;
      bool use_extended_convexity=true;
      bool use_sanity_criterion;

      // CPCSegmentation Stuff
      float min_cut_score = 0.16;
      unsigned int max_cuts = 25;
      unsigned int cutting_min_segments = 400;
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

      pcl::SupervoxelClustering<PointT> super (voxel_resolution, seed_resolution);
      super.setInputCloud (input_cloud_ptr_);
      if (has_normals)
        super.setNormalCloud (input_normals_ptr);
      super.setColorImportance (color_importance);
      super.setSpatialImportance (spatial_importance);
      super.setNormalImportance (normal_importance);
      std::map<uint32_t, pcl::Supervoxel<PointT>::Ptr> supervoxel_clusters;

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
      pcl::PointCloud<pcl::PointNormal>::Ptr sv_centroid_normal_cloud = pcl::SupervoxelClustering<PointT>::makeSupervoxelNormalCloud (supervoxel_clusters);

      /// Set paramters for LCCP preprocessing and CPC (CPC inherits from LCCP, thus it includes LCCP's functionality)

      // PCL_INFO ("Starting Segmentation\n");
      pcl::CPCSegmentation<PointT> cpc;
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
      pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = super.getLabeledCloud ();
      pcl::PointCloud<pcl::PointXYZL>::Ptr cpc_labeled_cloud = sv_labeled_cloud->makeShared ();
      cpc.relabelCloud (*cpc_labeled_cloud);
      SuperVoxelAdjacencyList sv_adjacency_list;
      cpc.getSVAdjacencyList (sv_adjacency_list);  // Needed for visualization
      segmented_cloud_ptr_ = cpc_labeled_cloud;

      Max_cluster = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>); 
      Max_cluster->height= cpc_labeled_cloud->height; 
      Max_cluster->width= cpc_labeled_cloud->width; 
      Max_cluster->is_dense = false;
      Max_cluster->points.resize(cpc_labeled_cloud->size()); 

      max_label=0;
      for(int i=0;i<cpc_labeled_cloud->size();i++)
      {
        if(cpc_labeled_cloud->points[i].label > max_label)
          max_label = cpc_labeled_cloud->points[i].label;
      }
      std::cout << "max label: " << max_label << std::endl;
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
        std::cout << (j-1) << " : " << counter << std::endl;
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
      // std::cout << "Max Label Index＝　" << max_label_index <<  std::endl;
      //pcl::io::savePCDFile ("BIG_SEG.pcd", *Max_cluster, false);
    } 
    pcl::PointCloud<pcl::PointXYZ>::Ptr get_BiggestCluster(){return Max_cluster;}
    pcl::PointCloud<pcl::PointXYZL>::Ptr getSegmentedPointCloud(){return segmented_cloud_ptr_;}
private:
   int max_label;
   int counter;
   int max_label_index;
   pcl::PCLPointCloud2 input_pointcloud2;
   pcl::PointCloud<PointT>::Ptr input_cloud_ptr_;
   pcl::PointCloud<pcl::PointXYZL>::Ptr segmented_cloud_ptr_;
   pcl::PointCloud<pcl::PointXYZ>::Ptr Max_cluster;
   pcl::PointCloud<pcl::PointXYZ>::CloudVectorType clusters;
};
