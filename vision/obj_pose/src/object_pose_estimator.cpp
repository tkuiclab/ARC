
#include "object_pose_estimator.hpp"
//#include "seg_plane_cam2obj.hpp"
#include "OrganizedSegmentation.h"
#include "cpc_segmentation.hpp"
#include "ICP_alignment.hpp"


using namespace ObjEstAction_namespace;

void ObjEstAction::goalCB()
{
  state = FOTO;
  obj_name = as_.acceptNewGoal()->object_name;
  ROS_INFO("Action calling! Goal=%s",obj_name.c_str());    
}

void ObjEstAction::preemptCB()
{
  ROS_INFO("%s: Preempted", action_name_.c_str());
  as_.setPreempted();
}

void ObjEstAction::cloudCB(const sensor_msgs::PointCloud2ConstPtr& input)
{
  if(state==FOTO)
  {
      pcl::fromROSMsg(*input,*cloud);
#ifdef SaveCloud
      pcl::PCDWriter writer1;
      std::stringstream ss1;
      // std::string sys_str;
      // sys_str = "rm  " +  path + "*.pcd";
      //std::cout << "[CMD] -> " << sys_str << std::endl;
      //system(sys_str.c_str());
      ss1 << path << "pcd_file/scene_cloud" << ".pcd";
      writer1.write<PT> (ss1.str (), *cloud, false);
      ROS_INFO("Save PCD to %s",ss1.str().c_str());
#endif
      state = CALL_RCNN;
  }
}

void ObjEstAction::get_roi(){
  // // Point clouds
  PCT::Ptr ROI_cloud (new PCT);
  my_ROICloud = PCT::Ptr (new PCT);
  roi_srv.request.object_name = obj_name;
  roi_client.call(roi_srv);
  if(roi_srv.response.result)
  {
    mini_x = roi_srv.response.bound_box[0];
    mini_y = roi_srv.response.bound_box[1];
    max_x = roi_srv.response.bound_box[2];
    max_y = roi_srv.response.bound_box[3];
  }else{
    ROS_INFO("Fail Detect!!!");
    preemptCB();
    state = NADA;
    return;
  }
  ROS_INFO("Get ROIl!");
  ROS_INFO("[mini_x: %d, mini_y: %d], [max_x: %d, max_y: %d]",mini_x,mini_y,max_x,max_y);
  ROI_cloud->width = max_x-mini_x;
  ROI_cloud->height = max_y-mini_y;
  ROI_cloud->is_dense = false;
  ROI_cloud->points.resize (ROI_cloud->width * ROI_cloud->height);

  int index;
  int index_tmp=0;

  for(int j=mini_y;j<max_y;j++)
  {
    for(int i=mini_x;i<max_x;i++)
    {
      index = j*cloud->width+i;
      ROI_cloud->points[index_tmp].x = cloud->points[index].x;
      ROI_cloud->points[index_tmp].y = cloud->points[index].y;
      ROI_cloud->points[index_tmp].z = cloud->points[index].z;
      ROI_cloud->points[index_tmp].rgba = cloud->points[index].rgba;
      index_tmp++;
    }
  }
  ROS_INFO("Save point cloud in ROI!\n");
  pcl::PCDWriter writer;
  tmp_path = path;
  tmp_path.append("pcd_file/test_pcd.pcd");
  writer.write<PT> (tmp_path, *ROI_cloud, false);
  std::cerr << "Saved " << ROI_cloud->points.size () << " data points to test_pcd.pcd." << std::endl;
  pcl::getMinMax3D(*ROI_cloud, min_p, max_p);
  // ROS_INFO("mini_x = %f",min_p.x);
  pcl::copyPointCloud(*ROI_cloud, *my_ROICloud);
  state = SEGMETATION;
}

void ObjEstAction::segmentation()
{
  ROS_INFO("Doing 3D Segmentation....");

  CPCSegmentation cpc_seg;
  if(scence_seg)
  {
    cpc_seg.setPointCloud(cloud);
    cpc_seg.do_segmentation();
    state = NADA;
  }else{
    cpc_seg.setPointCloud(my_ROICloud);
    cpc_seg.do_segmentation();
    Max_cluster = cpc_seg.get_BiggestCluster();
    state = ALIGMENT;
  }

  //----------------- Pub Segmentation Cloud to topic -----------------//
  pcl::PointCloud<pcl::PointXYZL>::Ptr segmented_pc_ptr = cpc_seg.getSegmentedPointCloud();
  pcl::PointCloud<pcl::PointXYZL> cloud2_;

  pcl::copyPointCloud(*segmented_pc_ptr, cloud2_);
  cloud2_.clear();
  BOOST_FOREACH (pcl::PointXYZL point, *segmented_pc_ptr) 
  {
      if (point.label == 0) continue;
      cloud2_.push_back(point);
  }
  pcl::toROSMsg(cloud2_, seg_msg);
  seg_msg.header.frame_id = "camera_link";
  segmented_pub_.publish(seg_msg);
}

void ObjEstAction::do_ICP()
{
  ROS_INFO("Aligning....");
  ICP_alignment my_icp;
  pcl::PointCloud<pcl::PointXYZ> temp2;
  transformation_matrix = Eigen::Matrix4d::Identity ();

  tmp_path = path;
  tmp_path.append("items/Crayons/Crayons.pcd");
  if(load_pcd(tmp_path))
  {
    ROS_INFO("Load Amazon Model success!");
    my_icp.setSourceCloud(Max_cluster);
    my_icp.setTargetCloud(model_PCD);
    my_icp.align(temp2);
    transformation_matrix = my_icp.getMatrix ();
    print4x4Matrix (transformation_matrix);
    //pcl::io::savePCDFile ("BIG_SEG.pcd", temp2, false);
    state = NADA;
  }else{
    state = NADA;
  }
}

void ObjEstAction::set_feedback(std::string msg,int progress)
{
  feedback_.msg = msg;
  feedback_.progress = progress;
  as_.publishFeedback(feedback_);
}

void ObjEstAction::print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

bool ObjEstAction::load_pcd(std::string pcd_filename)
{
  model_PCD = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ> ());
  ROS_INFO("Loading PCD....");
  if(pcl::io::loadPCDFile (pcd_filename, *model_PCD) < 0)
  {
    ROS_INFO("Error loading Amazon Model cloud");
    return true;
  }else{
    return true;
  }
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "obj_pose");
  ObjEstAction ObjEst(argc, argv,"obj_pose");
  ros::Rate loop_rate(10);
  while(ros::ok())
  {
    switch(state)
    {
      case NADA:
        break;
        
      case FOTO:
        ObjEst.set_feedback("Grabbing point cloud...",20);
        break;

      case CALL_RCNN:
        ObjEst.set_feedback("Getting ROI....",40);
        ObjEst.get_roi();
        break;

      case SEGMETATION:
        ObjEst.set_feedback("Doing segmentation....",60);
        ObjEst.segmentation();
        break;

      case ALIGMENT:
        ObjEst.set_feedback("Alignment....",80);
        ObjEst.do_ICP();
        break;
      default:
        ROS_INFO("Que!?");
        state = NADA;
      
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return (0);
}

