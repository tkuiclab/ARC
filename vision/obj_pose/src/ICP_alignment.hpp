#include <iostream>
#include <string>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>

class ICP_alignment  
{  
    public:  
  
    ICP_alignment ()  
    {  
       // Intialize the parameters in the ICP algorithm  
       //icp.setMaxCorrespondenceDistance(0.01);  
       //icp.setTransformationEpsilon(1e-7);  
       //icp.setEuclideanFitnessEpsilon(1);  
       icp.setMaximumIterations(100);  
    }  
  
    ~ICP_alignment () {}  
  
     void setSourceCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr source_cloud)  
    {  
        icp.setInputCloud(source_cloud);  
    }  
  
    void setTargetCloud (pcl::PointCloud<pcl::PointXYZ>::ConstPtr target_cloud)  
    {  
        std::cout << "set target!" << std::endl;
        icp.setInputTarget(target_cloud);  
    }  
      
    // Align the given template cloud to the target specified by setTargetCloud ()  
    void align (pcl::PointCloud<pcl::PointXYZ> &temp)  
    {  
        
      pcl::PointCloud<pcl::PointXYZ> registration_output;  
      icp.align (temp);  
  
      fitness_score =  icp.getFitnessScore();
      final_transformation = Eigen::Matrix4f::Identity ();
      final_transformation = icp.getFinalTransformation ();  
    }  
  
    float getScore()  
    {  
        return fitness_score;  
    }  
  
    Eigen::Matrix4f getMatrix()  
    {  
        return final_transformation;  
    }  
  private:  
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;  
    Eigen::Matrix4f final_transformation;  
    float fitness_score;  
};  