#ifndef _OBJECT_POSE_AUXILIARY_H_
#define _OBJECT_POSE_AUXILIARY_H_

#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>

// Types
typedef pcl::PointXYZRGBA PT;           //Point Type
typedef pcl::PointCloud<PT> PCT;

typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PC_NT;

typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

std::string path;

std::string AmazonModelList[40] =
{
  "Avery_Binder",
  "Ballons",
  "Band_Aid_Tape",
  "Bath_Sponge",
  "Black_Fashion_Gloves",
  "Burts_Bees_Baby_Wipes",
  "Colgate_Toothbrush_4PK",
  "Composition_Book",
  "Crayons",
  "Duct_Tape",
  "Epsom_Salts",
  "Expo_Eraser",
  "Fiskars_Scissors",
  "Flashlight",
  "Glue_Sticks",
  "Hand_Weight",
  "Hanes_Socks",
  "Hinged_Ruled_Index_Cards",
  "Ics_Cube_Tray",
  "Irish_Spring_Soap",
  "Laugh_Out_Loud_Jokes",
  "Marbles",
  "Measuring_Spoons",
  "Mesh_Cup",
  "Mouse_Traps",
  "Pie_Plates",
  "Plastic_Wine_Glass",
  "Poland_Spring_Water",
  "Reynolds_Wrap",
  "Robots_DVD",
  "Robots_Everywhere",
  "Scotch_Sponges",
  "Speed_Stick",
  "Table_Cloth",
  "Tennis_Ball_Container",
  "Ticonderoga_Pencils",
  "Tissue_Box",
  "Toilet_Brush",
  "White_Facecloth",
  "Windex"
};

std::string LabelList[40] =
{
    "avery_binder",
    "ballons",
    "band_aid_tape",
    "bath_sponge",
    "black_fashion_gloves",
    "burts_bees_baby_wipes",
    "colgate_toothbrush_4pk",
    "composition_book",
    "crayons",
    "duct_tape",
    "epsom_salts",
    "expo_eraser",
    "fiskars_scissors",
    "flashlight",
    "glue_sticks",
    "hand_weight",
    "hanes_socks",
    "hinged_ruled_index_cards",
    "ics_cube_tray",
    "irish_spring_soap",
    "laugh_out_loud_jokes",
    "marbles",
    "measuring_spoons",
    "mesh_cup",
    "mouse_traps",
    "pie_plates",
    "plastic_wine_glass",
    "poland_spring_water",
    "reynolds_wrap",
    "robots_dvd",
    "robots_everywhere",
    "scotch_sponges",
    "speed_stick",
    "table_cloth",
    "tennis_ball_container",
    "ticonderoga_pencils",
    "tissue_box",
    "toilet_brush",
    "white_facecloth",
    "windex"
};


void write_pcd_2_rospack(PCT::Ptr cloud, std::string f_name){
    path = ros::package::getPath("obj_pose");
    path.append("/pcd_file/");
    path.append(f_name);

    pcl::PCDWriter writer;
    writer.write<PT> (path, *cloud, false);

    std::cout << "Save PCD -> " << path << std::endl;
}



void write_pcd_2_rospack_normals(PC_NT::Ptr cloud, std::string f_name){
    path = ros::package::getPath("obj_pose");
    path.append("/pcd_file/");
    path.append(f_name);

    pcl::PCDWriter writer;
    writer.write<PointNT> (path, *cloud, true);

    std::cout << "Save PCD -> " << path << std::endl;
}

bool load_amazon_pcd_model(std::string pcd_filename, pcl::PointCloud<pcl::PointXYZ>::Ptr out_model_cloud)
{
  int index=0;
  std::stringstream ss1;
  for(int i =0;i<40;i++)
  {
    if(LabelList[i]==pcd_filename)
    {
      index = i;
    }
  }

  std::string path = ros::package::getPath("obj_pose");
  ss1 << path << "/items/" << AmazonModelList[index] << "/" << AmazonModelList[index] << "1.pcd";
  out_model_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ> ());
  ROS_INFO("Loading PCD....");
  ROS_INFO("PCD at %s",ss1.str().c_str());
  if(pcl::io::loadPCDFile (ss1.str(), *out_model_cloud) < 0)
  {
    ROS_ERROR("Error loading Amazon Model cloud");
    return false;
  }else{
    // pcl::VoxelGrid<pcl::PointXYZ> sor;
    // sor.setInputCloud (out_model_cloud);
    // sor.setLeafSize (0.001f, 0.001f, 0.001f);
    // sor.filter (*out_model_cloud);
    return true;
  }
}

#endif