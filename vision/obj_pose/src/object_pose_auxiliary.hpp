#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>

// Types
typedef pcl::PointXYZRGB PT;           //Point Type
typedef pcl::PointCloud<PT> PCT;

typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PC_NT;

typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;


std::string AmazonModelList[40] =
{
  "Avery_Binder",
  "Ballons",
  "Band_Aid_Tape",
  "Bath_Sponge",
  "Black_Fashion_Gloves",
  "Burls_Bees_Baby_Wipes",
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
  "avery1BinderWhite",
  "bagOfBalloons",
  "johnsonjohnsonPaperTape",
  "theBatheryDelicateBathSponge",
  "knitGlovesBlack",
  "burtsBeesBabyWipes",
  "colgateToothbrushs",
  "greenCompositionBook",
  "crayolaCrayons24",
  "scotchClothDuctTape",
  "drtealsEpsomSalts",  
  "expoEraser",  
  "fiskarScissors",  
  "arFlashlihgts",
  "elmersGlueSticks6Ct",  
  "neopreneWeightPink",
  "hanesWhitteSocks",  
  "spiralIndexCards",
  "steriliteIceCubeTray",
  "irishSpring",
  "laughOutLoundJokesForKids",
  "miniMarblesClearLustre",
  "targetBrandMeasuringSpoons",
  "meshPencilCup",
  "tomcatMousetraps",
  "reynoldsPiePans2ct",
  "plasticWineGlasses",
  "polandSpringsWaterBottle",
  "reynoldsWrap85Sqft",
  "dvdRobots",
  "robotsEverywhere",
  "scotchSponges",
  "speedStick2Pack",
  "tableCover",
  "wilsonTennisBalls",
  "ticonderogaPencils",
  "kleenexCoolTouchTissues",
  "cloroxToiletBrush",
  "whiteFaceCloth",
  "windexSprayBottle23oz"
};


void write_pcd_2_rospack(PCT::Ptr cloud, std::string f_name){
    std::string path = ros::package::getPath("obj_pose");
    path.append("/pcd_file/");
    path.append(f_name);

    pcl::PCDWriter writer;
    writer.write<PT> (path, *cloud, false);

    std::cout << "Save PCD -> " << path << std::endl;
}



void write_pcd_2_rospack_normals(PC_NT::Ptr cloud, std::string f_name){
    std::string path = ros::package::getPath("obj_pose");
    path.append("/pcd_file/");
    path.append(f_name);

    pcl::PCDWriter writer;
    writer.write<PointNT> (path, *cloud, false);

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
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (out_model_cloud);
    sor.setLeafSize (0.005f, 0.005f, 0.005f);
    sor.filter (*out_model_cloud);
    return true;
  }
}