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
      bool use_extended_convexity;
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

      // pcl::console::print_info ("Maximum cuts: %d\n", max_cuts);
      // pcl::console::print_info ("Minumum segment siz: %d\n", cutting_min_segments);
      // pcl::console::print_info ("Use local constrain: %d\n", use_local_constrain);
      // pcl::console::print_info ("Use directed weights: %d\n", use_directed_cutting);
      // pcl::console::print_info ("Use clean cuts: %d\n", use_clean_cutting);
      // pcl::console::print_info ("RANSAC iterations: %d\n", ransac_iterations);  

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

      // /// Creating Colored Clouds and Output
      // if (cpc_labeled_cloud->size () == input_cloud_ptr_->size ())
      // {
      //   if (output_specified)
      //   {
      //     // if (pcl::getFieldIndex (input_pointcloud2, "label") >= 0)
      //     //   PCL_WARN ("Input cloud already has a label field. It will be overwritten by the cpc segmentation output.\n");
      //     pcl::PCLPointCloud2 output_label_cloud2, output_concat_cloud2;
      //     pcl::toPCLPointCloud2 (*cpc_labeled_cloud, output_label_cloud2);
      //     pcl::concatenateFields (input_pointcloud2, output_label_cloud2, output_concat_cloud2);
      //     //pcl::io::savePCDFile (outputname + "_out.pcd", output_concat_cloud2, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), save_binary_pcd);
      //   }
      //   //segmented_cloud_ptr_ = cloud_cluster1;
      // }
      // else
      // {
      //   PCL_ERROR ("ERROR:: Sizes of input cloud and labeled supervoxel cloud do not match. No output is produced.\n");
      // }
    }  /// END main

    pcl::PointCloud<pcl::PointXYZL>::Ptr getSegmentedPointCloud(){return segmented_cloud_ptr_;}
private:
   pcl::PCLPointCloud2 input_pointcloud2;
   pcl::PointCloud<PointT>::Ptr input_cloud_ptr_;
   pcl::PointCloud<pcl::PointXYZL>::Ptr segmented_cloud_ptr_;
};
