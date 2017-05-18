#include <pcl/common/angles.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

typedef pcl::PointXYZRGBA PointType;
bool use_planar_refinement_ = true;


class OrganizedSegmentation
{
public:
	OrganizedSegmentation()
	{
		labels = pcl::PointCloud<pcl::Label>::Ptr (new pcl::PointCloud<pcl::Label>);
		euclidean_cluster_comparator_ = pcl::EuclideanClusterComparator<PointType, pcl::Normal, pcl::Label>::Ptr 
										(new pcl::EuclideanClusterComparator<PointType, pcl::Normal, pcl::Label>());
		viewer1 = boost::shared_ptr<pcl::visualization::PCLVisualizer> (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	}

	void PlaneSegmentation(pcl::PointCloud<PointType>::Ptr &cloud)
	{
		pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);

		std::vector<pcl::ModelCoefficients> model_coefficients;
		std::vector<pcl::PointIndices> inlier_indices;

		// Estimate Normals
		pcl::IntegralImageNormalEstimation<PointType, pcl::Normal> ne;
		ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
		ne.setMaxDepthChangeFactor (0.02f);
		ne.setNormalSmoothingSize (20.0f);
		ne.setInputCloud (cloud);
		ne.compute (*normal_cloud);

		// Segment planes
		pcl::OrganizedMultiPlaneSegmentation< PointType, pcl::Normal, pcl::Label > mps;
		mps.setMinInliers (10000);
		mps.setAngularThreshold (pcl::deg2rad (2.0)); // 3 degrees
		mps.setDistanceThreshold (0.005); // 1.5 cm

		mps.setInputNormals (normal_cloud);
		mps.setInputCloud (cloud);
		if (use_planar_refinement_)
		{
			mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);
		}
		else
		{
			mps.segment (regions);
		}
	}

	void ObjectSegmentation(pcl::PointCloud<PointType>::Ptr &cloud)
	{
		// Segment Objects
		if (regions.size () > 0)
		{
			std::vector<bool> plane_labels;
			plane_labels.resize (label_indices.size (), false);
			for (size_t i = 0; i < label_indices.size (); i++)
			{
				if (label_indices[i].indices.size () > 10000)
				{
					plane_labels[i] = true;
				}
			} 

			euclidean_cluster_comparator_->setInputCloud (cloud);
			euclidean_cluster_comparator_->setLabels (labels);
			euclidean_cluster_comparator_->setExcludeLabels (plane_labels);
			euclidean_cluster_comparator_->setDistanceThreshold (0.01f, false);

			pcl::PointCloud<pcl::Label> euclidean_labels;
			std::vector<pcl::PointIndices> euclidean_label_indices;
			pcl::OrganizedConnectedComponentSegmentation<PointType,pcl::Label> euclidean_segmentation (euclidean_cluster_comparator_);
			euclidean_segmentation.setInputCloud (cloud);
			euclidean_segmentation.segment (euclidean_labels, euclidean_label_indices);

			for (size_t i = 0; i < euclidean_label_indices.size (); i++)
			{
				if (euclidean_label_indices[i].indices.size () > 1000)
				{
					pcl::PointCloud<PointType> cluster;
					pcl::copyPointCloud (*cloud,euclidean_label_indices[i].indices,cluster);
					clusters.push_back (cluster);
				}    
			}

			PCL_INFO ("Got %d euclidean clusters!\n", clusters.size ());
		}
		else 
		{
			PCL_INFO ("Can't find euclidean clusters!\n");
		}
	}

	void clusterViewer(pcl::PointCloud<PointType>::Ptr &cloud)
	{
		// Display
		pcl::PointCloud<PointType>::Ptr PCD_add (new pcl::PointCloud<PointType>);
		pcl::PointCloud<PointType>::Ptr PCD_src (new pcl::PointCloud<PointType>);
		
		unsigned char red [6] = {255,   0,   0, 255, 255,   0};
		unsigned char grn [6] = {  0, 255,   0, 255,   0, 255};
		unsigned char blu [6] = {  0,   0, 255,   0, 255, 255};

		std::stringstream ss;

		for (size_t i = 0; i < clusters.size (); i++)
		{
			ss << "cloud_" << i;
			Eigen::Vector4f centroid;

			pcl::compute3DCentroid (clusters[i], centroid);

			PointType point;

			point.getArray4fMap() << centroid;
			viewer1->addText3D(ss.str (), point, 0.02, 1.0, 0.0, 0.0, ss.str ());
			ss.str("");
			ss.clear();

			//  Copy clouds to show segment result.
			pcl::copyPointCloud(clusters[i], *PCD_src);
			for (int j = 0; j < PCD_src->width; j++)
			{
				PCD_src->points[j].r = red[i%15];
				PCD_src->points[j].g = grn[i%15];
				PCD_src->points[j].b = blu[i%15];
			}
			*PCD_add = *PCD_add + *PCD_src;
		}

		viewer1->addPointCloud(cloud, "original_cloud");
		viewer1->addPointCloud(PCD_add, "cloud_");
		viewer1->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud_cluster");
		viewer1->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "cloud_cluster");


		while (!viewer1->wasStopped())
		{
		viewer1->spinOnce();
		}
		}

	int FindSegmentPlaneScene(pcl::PointCloud<PointType>::Ptr &cloud)
	{
		PlaneSegmentation(cloud);
		ObjectSegmentation(cloud);	

		return (clusters.size());
	}

	void SaveClusterModel(std::string &OUT_PATH , int frames)
	{
		pcl::PCDWriter writer;
		std::stringstream ss;

		// Have a problem !!!
		Eigen::Vector4f centroid;
		Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

		for (size_t i = 0; i < clusters.size (); i++)
		{
			// Have a problem !!!
			pcl::compute3DCentroid (clusters[i], centroid);

			//transformation(0, 3) = -centroid[0];
			//transformation(1, 3) = -centroid[1];
			//transformation(2, 3) = -centroid[2];

			pcl::transformPointCloud(clusters[i], clusters[i], transformation);

			ss << OUT_PATH.c_str() << "cloud_cluster_" << frames << "_" << i << ".pcd";
			writer.write<PointType> (ss.str (), clusters[i], false);

			ss.str("");
			ss.clear();
		}
	}


private:
	std::vector<pcl::PlanarRegion<PointType>, Eigen::aligned_allocator<pcl::PlanarRegion<PointType> > > regions;

	pcl::PointCloud<pcl::Label>::Ptr labels;
	std::vector<pcl::PointIndices> label_indices;
	std::vector<pcl::PointIndices> boundary_indices;
	pcl::PointCloud<PointType>::CloudVectorType clusters;
	pcl::EuclideanClusterComparator<PointType, pcl::Normal, pcl::Label>::Ptr euclidean_cluster_comparator_;

public:
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1;
};
