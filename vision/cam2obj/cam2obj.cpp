
#include "cam2obj.hpp"




int main(int argc,char** argv){
   PCT::Ptr cloud(new PCT);

  g_argc = argc;
  g_argv = argv;

  std::string f_name = get_first_pcd_name(argc,argv);
  if(f_name == ""){
     std::cerr << "Please input PCD file" <<  std::endl;
     return -1;
  }

  pcl::PCDReader reader;
  reader.read (f_name + ".pcd", *cloud);

#ifdef SaveCloud
  write_pcd(cloud,"scene_colud.pcd");
#endif

  std::vector<int> index;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, index);
  
#ifdef SaveCloud
  write_pcd(cloud,"_rm_NaN.pcd");
#endif

  pass_through_from_arg(cloud, argc,argv, cloud);

#ifdef SaveCloud
  write_pcd(cloud,"_PassThrough.pcd");
#endif

  double x,y,z ,a_x,a_y,a_z;

  float near_points_percent = 0.1;
  if (pcl::console::find_switch (g_argc, g_argv, "-near")){
    pcl::console::parse (g_argc, g_argv, "-near", near_points_percent);
  }

  cam_2_obj_center(cloud, 
      x,y,z, 
      a_x,a_y,a_z,
      near_points_percent);
  

  std::cout << "(x, y, z) = " << "(" << x  << "," << y  << "," << z << ")" << std::endl;
  std::cout << "(r, p, y) = " << "(" << a_x  << "," << a_y  << "," << a_z << ")" << std::endl;
  

  return 0;
}