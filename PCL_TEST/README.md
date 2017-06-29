# cmd
/cam2obj data/1.pcd -pa
ss_z_max 0.27 -pass_x_min -0.4 -pass_x_max 0.4 -pass_y_min -0.25 -pas
s_y_max 0.25

# show all 
pcl_viewer -multiview 1 *.pcd

# show normals
pcl_viewer -normals 1 _mls.pcd

# show ax 
pcl_viewer -ax 1 _mls.pcd




./cam2obj data/3.pcd -pass_z_max 0.27 -pass_x_min -0.4 -pass_x_max 0.4 -pass_y_min -0.25 -pass_y_max 0.25


./cam2obj _ROI_lie.pcd -near 0.05 -pass_z_min 0.27 -pass_z_max 0.60 -pass_x_min -0.4 -pass_x_max 0.4 -pass_y_min -0.25 -pass_y_max 0.25


rosrun pcl_ros pointcloud_to_pcd input:=/camera/depth_registered/points
