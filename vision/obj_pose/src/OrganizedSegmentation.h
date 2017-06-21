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

#include <algorithm>
#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>

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

	boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (
    	pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
	{
		// --------------------------------------------------------
		// -----Open 3D viewer and add point cloud and normals-----
		// --------------------------------------------------------
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
		viewer->setBackgroundColor (0, 0, 0);
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
		viewer->addPointCloud<pcl::PointXYZRGBA> (cloud, rgb, "sample cloud");
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
		viewer->addPointCloudNormals<pcl::PointXYZRGBA, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
		viewer->addCoordinateSystem (1.0);
		viewer->initCameraParameters ();
		return (viewer);
	}


	void PlaneSegmentation(pcl::PointCloud<PointType>::Ptr &cloud)
	{
		pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);

		std::vector<pcl::ModelCoefficients> model_coefficients;
		std::vector<pcl::PointIndices> inlier_indices;

		//Estimate Normals
		pcl::IntegralImageNormalEstimation<PointType, pcl::Normal> ne;
		ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
		ne.setMaxDepthChangeFactor (0.02f);
		ne.setNormalSmoothingSize (5.0f);
		ne.setInputCloud (cloud);
		ne.compute (*normal_cloud);

		// pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne2;
		// ne2.setInputCloud (cloud);
  		// pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());
		// ne2.setSearchMethod (tree);
  		// pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
	  	// ne2.setRadiusSearch (0.1);
  		// ne2.compute (*cloud_normals2);

		viewer = normalsVis(cloud, normal_cloud);
		while (!viewer->wasStopped ())
		{
			viewer->spinOnce (100);
			boost::this_thread::sleep (boost::posix_time::microseconds (100000));
		}
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
			std::vector<int> m_indices;
			std::vector<int> already_process_indices;

			for (size_t i = 0; i < euclidean_label_indices.size (); i++)
			{
				if (euclidean_label_indices[i].indices.size () > 1000)
				{
					// std::vector<int>::iterator result;
					// std::vector<int>::iterator result2;
					// result = std::max_element(euclidean_label_indices[i].indices.begin(), euclidean_label_indices[i].indices.end());
					// result2 = std::min_element(euclidean_label_indices[i].indices.begin(), euclidean_label_indices[i].indices.end());
					// int max = euclidean_label_indices[i].indices[std::distance(euclidean_label_indices[i].indices.begin(), result)];
					// int min = euclidean_label_indices[i].indices[std::distance(euclidean_label_indices[i].indices.begin(), result2)];
					// std::cout << "max number is: " << max << '\n';
					// std::cout << "max number is: " << min << '\n';
					// if(max < ((max_y-1)*cloud->width+max_x-1) && min > ((mini_y-1)*cloud->width+mini_x-1))
					// {
					// 	PCL_INFO ("Cluster %d is in ROI!\n",i);
					// 	pcl::PointCloud<PointType> cluster;
					// 	pcl::copyPointCloud (*cloud,euclidean_label_indices[i].indices,cluster);
					// 	clusters.push_back (cluster);
					// }else{
					// 	PCL_INFO ("Cluster %d not in ROI!\n",i);
					// }
					int tmp=0;
					while(tmp<cloud->size())
					{
						int a = tmp/cloud->width;
						int b = tmp%cloud->width;
						if(a <= max_y && a >= mini_y && b <= max_x && b >= mini_x)
						{
							m_indices.push_back(tmp);
						}else{
							already_process_indices.push_back(tmp);
						}
						tmp++;
					}
					
					
					
					// PCL_INFO ("The size of segmentation is %d!\n", euclidean_label_indices[i].indices.size ());
					// //PCL_INFO ("The width of cloud is %d!\n", cloud->width);
					// int tmp=0;
					// while(tmp < euclidean_label_indices[i].indices.size ())
					// {
					// 	int b = euclidean_label_indices[i].indices[tmp]%cloud->width;
					// 	int a = euclidean_label_indices[i].indices[tmp]/cloud->width;
					// 	if(a <= max_y && a >= mini_y && b <= max_x && b >= mini_x)
					// 	{
					// 		m_indices.push_back(euclidean_label_indices[i].indices[tmp]);
					// 	}else{
					// 		already_process_indices.push_back(euclidean_label_indices[i].indices[tmp]);
					// 	}
					// 	tmp++;
					// }



					
					// std::vector<int>::iterator result;
					// result = std::max_element(euclidean_label_indices[i].indices.begin(), euclidean_label_indices[i].indices.end());
					// int max = euclidean_label_indices[i].indices[std::distance(euclidean_label_indices[i].indices.begin(), result)];
					// std::cout << "max number is: " << max << '\n';
					// int a = max/cloud->width;
					// int b = max%cloud->width;
					// std::cout << "b = " << b << '\n';
					// std::cout << "a = " << a << '\n';
				}
			}
			pcl::PointCloud<PointType> cluster;
			pcl::copyPointCloud (*cloud,m_indices,cluster);
			clusters.push_back (cluster);
			pcl::PointCloud<PointType> already_process_cluster;
			pcl::copyPointCloud (*cloud,already_process_indices,already_process_cluster);
			clusters.push_back (already_process_cluster);
			PCL_INFO ("Got %d euclidean cluster!\n", clusters.size ());
			PCL_INFO ("The size of cluster is %d!\n", cluster.size ());
		}
		else 
		{
			std::vector<int> m_indices;
			std::vector<int> already_process_indices;
			PCL_INFO ("Can't find euclidean clusters!\n");
			int tmp=0;
			while(tmp<cloud->size())
			{
				int a = tmp/cloud->width;
				int b = tmp%cloud->width;
			if(a <= max_y && a >= mini_y && b <= max_x && b >= mini_x)
			{
				m_indices.push_back(tmp);
			}else{
				already_process_indices.push_back(tmp);
			}
			tmp++;
			}
			pcl::PointCloud<PointType> cluster;
			pcl::copyPointCloud (*cloud,m_indices,cluster);
			clusters.push_back (cluster);
			pcl::PointCloud<PointType> already_process_cluster;
			pcl::copyPointCloud (*cloud,already_process_indices,already_process_cluster);
			clusters.push_back (already_process_cluster);
			PCL_INFO ("Got %d euclidean cluster!\n", clusters.size ());
			PCL_INFO ("The size of cluster is %d!\n", cluster.size ());
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
		//viewer1->setBackgroundColor (255, 255, 255);
		viewer1->setBackgroundColor (0, 0, 0);
		viewer1->addPointCloud(cloud, "original_cloud");
		viewer1->addCoordinateSystem(0.1, 0);
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
	pcl::PointCloud<PointType>::CloudVectorType return_cluster()
	{
		return clusters;
	}

	void set_roi(int _mini_x, int _mini_y,int _max_x,int _max_y)
	{
		mini_x = _mini_x; 
		mini_y = _mini_y; 
		max_x = _max_x;
		max_y = _max_y;
	}


private:
	std::vector<pcl::PlanarRegion<PointType>, Eigen::aligned_allocator<pcl::PlanarRegion<PointType> > > regions;

	pcl::PointCloud<pcl::Label>::Ptr labels;
	std::vector<pcl::PointIndices> label_indices;
	std::vector<pcl::PointIndices> boundary_indices;
	pcl::PointCloud<PointType>::CloudVectorType clusters;
	pcl::EuclideanClusterComparator<PointType, pcl::Normal, pcl::Label>::Ptr euclidean_cluster_comparator_;

	//--------Class Usage------//
	int mini_x;
	int mini_y;
	int max_x;
	int max_y;

public:
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
};
