# cmd

#
/cam2obj data/1.pcd -pass_z_max 0.27 -pass_x_min -0.4 -pass_x_max 0.4 -pass_y_min -0.25 -pas
s_y_max 0.25

# show all 
pcl_viewer -multiview 1 *.pcd

# show normals
pcl_viewer -normals 1 -ax 1 _mls.pcd
pcl_viewer -normals 1 _del_out_mean_normal.pcd

# show ax 
pcl_viewer -ax 1 _mls.pcd
pcl_viewer -ax 1 dvdRobotsCenterPitch*.pcd

# show ax & multiview
pcl_viewer -ax 1 -multiview 1 *.pcd

# desktop
./cam2obj data/3.pcd -pass_z_min 0.27 -pass_z_max 0.27 -pass_x_min -0.4 -pass_x_max 0.4 -pass_y_min -0.25 -pass_y_max 0.25

# robot 
./cam2obj _ROI_lie.pcd -near 0.05 -pass_z_min 0.27 -pass_z_max 0.60 -pass_x_min -0.4 -pass_x_max 0.4 -pass_y_min -0.25 -pass_y_max 0.25


rosrun pcl_ros pointcloud_to_pcd input:=/camera/depth_registered/points



# dvd Result ----Cam Roll-----
pcl_viewer -multiview 1 -ax 1 dvdRobotsCenterPitch*.pcd


./cam2obj robot_dvd/dvdRobotsCenterPitch20_1.pcd -near 0.05 -pass_z_min 0.27 -pass_z_max 0.55 -pass_x_min -0.4 -pass_x_max 0.4 -pass_y_min -0.25 -pass_y_max 0.25
reault --->>>
obj_normal=(0.092485,0.137987,0.988861)
 (roll, pitch, yaw) = (3.00295,3.04923,-3.13517) = 
 (172.056,174.708,-179.632)
(r, p) = (7.94384,5.34309)

./cam2obj robot_dvd/dvdRobotsCenterPitch40_1.pcd -near 0.05 -pass_z_min 0.27 -pass_z_max 0.55 -pass_x_min -0.4 -pass_x_max 0.4 -pass_y_min -0.25 -pass_y_max 0.25
reault --->>> 
obj_normal=(0.093531,0.591839,0.800025)
 (roll, pitch, yaw) = (143.507,174.631,-178.229)
 (r, p) = (36.4931,6.66815)

./cam2obj robot_dvd/dvdRobotsCenterPitch20_2.pcd -near 0.05 -pass_z_min 0.27 -pass_z_max 0.56 -pass_x_min -0.4 -pass_x_max 0.4 -pass_y_min -0.25 -pass_y_max 0.25
result ----->>
(roll, pitch, yaw) = (25.0001,-4.17989,0.927053)
 (r, p) = (25.0001,4.61023)

./cam2obj robot_dvd/dvdRobotsCenterPitch40_2.pcd -near 0.05 -pass_z_min 0.27 -pass_z_max 0.55 -pass_x_min -0.2 -pass_x_max 0.2 -pass_y_min -0.1 -pass_y_max 0.1
obj_normal=(-0.118330,-0.642154,0.757393)
(roll, pitch, yaw) = (40.2929,-6.79573,2.49567)
 (r, p) = (40.2929,8.87976)
 

# -------Cam Pitch Test---------#
./cam2obj p_test/dvdRobotsLeftRoll20_00001.pcd -near 0.05 -pass_z_min 0.27 -pass_z_max 0.55 -pass_x_min -0.15 -pass_x_max 0.15 -pass_y_min -0.12 -pass_y_max 0.12
 (roll, pitch, yaw) = (3.43265,-32.2226,0.991788)
 (r, p) = (3.43262,32.269)

./cam2obj p_test/dvdRobotsLeftRoll20_00002.pcd -near 0.05 -pass_z_min 0.27 -pass_z_max 0.55 -pass_x_min -0.15 -pass_x_max 0.15 -pass_y_min -0.12 -pass_y_max 0.12
(roll, pitch, yaw) = (12.7613,-28.9413,3.30609)
 (r, p) = (12.7613,29.5521)

./cam2obj p_test/dvdRobotsLeftRoll40_00001.pcd -near 0.05 -pass_z_min 0.27 -pass_z_max 0.55 -pass_x_min -0.15 -pass_x_max 0.15 -pass_y_min -0.12 -pass_y_max 0.12
(roll, pitch, yaw) = 
(0.222183,-0.777008,0.0912535) = (12.7301,-44.5193,5.22844)
 

 ./cam2obj p_test/dvdRobotsLeftRoll40_00002.pcd -near 0.05 -pass_z_min 0.27 -pass_z_max 0.55 -pass_x_min -0.15 -pass_x_max 0.15 -pass_y_min -0.12 -pass_y_max 0.12
(roll, pitch, yaw) = (0.137848,-0.783817,0.0570456) = 
(7.89808,-44.9094,3.26847)

./cam2obj p_test/dvdRobotsRightRoll20_00001.pcd -near 0.05 -pass_z_min 0.27 -pass_z_max 0.55 -pass_x_min -0.15 -pass_x_max 0.15 -pass_y_min -0.12 -pass_y_max 0.12
(roll, pitch, yaw) = 
(0.160161,0.476397,-0.0389668) = 
(9.17655,27.2955,-2.23263)


./cam2obj p_test/dvdRobotsRightRoll20_00002.pcd -near 0.05 -pass_z_min 0.27 -pass_z_max 0.55 -pass_x_min -0.15 -pass_x_max 0.15 -pass_y_min -0.12 -pass_y_max 0.12
 (roll, pitch, yaw) = 
 (0.174463,0.521492,-0.0466604) =  (9.99601,29.8793,-2.67344)

./cam2obj p_test/dvdRobotsRightRoll40_00001.pcd -near 0.05 -pass_z_min 0.27 -pass_z_max 0.55 -pass_x_min -0.15 -pass_x_max 0.15 -pass_y_min -0.12 -pass_y_max 0.12
(roll, pitch, yaw) = 
(0.145565,0.560154,-0.0419378) = (8.34023,32.0945,-2.40286)

./cam2obj p_test/dvdRobotsRightRoll40_00002.pcd -near 0.05 -pass_z_min 0.27 -pass_z_max 0.55 -pass_x_min -0.15 -pass_x_max 0.15 -pass_y_min -0.12 -pass_y_max 0.12
 (roll, pitch, yaw) = 
 (0.00192829,0.721007,-0.000726921) = 
 (0.110483,41.3107,-0.0416495)

 # -------Cam Syn Test---------#
 ./cam2obj syn_test/dvdRobotsSyn_00001.pcd -near 0.05 -pass_z_min 0.27 -pass_z_max 0.55 -pass_x_min -0.15 -pass_x_max 0.15 -pass_y_min -0.12 -pass_y_max 0.12
(roll, pitch, yaw) = (0.584375,0.376587,-0.114509) = (33.4822,21.5768,-6.5
6087)

 ./cam2obj syn_test/dvdRobotsSyn_00002.pcd -near 0.05 -pass_z_min 0.27 -pass_z_max 0.55 -pass_x_min -0.15 -pass_x_max 0.15 -pass_y_min -0.12 -pass_y_max 0.12
 (roll, pitch, yaw) = (0.584375,0.376587,-0.114509) = 
 (25.3662,-35.5045,8.24195)


./cam2obj syn_test/dvdRobotsSyn_00003.pcd -near 0.05 -pass_z_min 0.27 -pass_z_max 0.55 -pass_x_min -0.15 -pass_x_max 0.15 -pass_y_min -0.12 -pass_y_max 0.12
(roll, pitch, yaw) = (2.59254,-2.63216,2.99519) = (148.542,-150.811,171.61
2)

./cam2obj syn_test/dvdRobotsSyn_00004.pcd -near 0.05 -pass_z_min 0.27 -pass_z_max 0.55 -pass_x_min -0.15 -pass_x_max 0.15 -pass_y_min -0.12 -pass_y_max 0.12
(roll, pitch, yaw) = (2.69557,2.72108,-3.04487) = (154.445,155.906,-174.45
8)

./cam2obj syn_test/dvdRobotsSyn_00005.pcd -near 0.05 -pass_z_min 0.27 -pass_z_max 0.55 -pass_x_min -0.15 -pass_x_max 0.15 -pass_y_min -0.12 -pass_y_max 0.12
(roll, pitch, yaw) = (0.350396,0.231153,-0.0410944) = (20.0762,13.2441,-2.
35454)

./cam2obj syn_test/dvdRobotsSyn_00006.pcd -near 0.05 -pass_z_min 0.27 -pass_z_max 0.55 -pass_x_min -0.15 -pass_x_max 0.15 -pass_y_min -0.12 -pass_y_max 0.12
(roll, pitch, yaw) = (2.93837,3.06435,-3.13371) = (168.356,175.574,-179.54
9)

./cam2obj syn_test/dvdRobotsSyn_00007.pcd -near 0.05 -pass_z_min 0.27 -pass_z_max 0.55 -pass_x_min -0.15 -pass_x_max 0.15 -pass_y_min -0.12 -pass_y_max 0.12
(roll, pitch, yaw) = (3.01546,-2.74431,3.11617) = (172.773,-157.237,178.54
3)


./cam2obj syn_test/dvdRobotsSyn_00008.pcd -near 0.05 -pass_z_min 0.27 -pass_z_max 0.55 -pass_x_min -0.15 -pass_x_max 0.15 -pass_y_min -0.12 -pass_y_max 0.12
(roll, pitch, yaw) = (0.276954,-0.569533,0.0815475) = (15.8683,-32.6318,4.
67233)



# roll > 90 
./cam2obj syn_test/dvdRobotsSyn_00003.pcd -near 0.05 -pass_z_min 0.27 -pass_z_max 0.55 -pass_x_min -0.15 -pass_x_max 0.15 -pass_y_min -0.12 -pass_y_max 0.12
(roll, pitch, yaw) = (2.59254,-2.63216,2.99519) = (148.542,-150.811,171.61
2)

./cam2obj syn_test/dvdRobotsSyn_00004.pcd -near 0.05 -pass_z_min 0.27 -pass_z_max 0.55 -pass_x_min -0.15 -pass_x_max 0.15 -pass_y_min -0.12 -pass_y_max 0.12
(roll, pitch, yaw) = (2.69557,2.72108,-3.04487) = (154.445,155.906,-174.45
8)

./cam2obj syn_test/dvdRobotsSyn_00006.pcd -near 0.05 -pass_z_min 0.27 -pass_z_max 0.55 -pass_x_min -0.15 -pass_x_max 0.15 -pass_y_min -0.12 -pass_y_max 0.12
(roll, pitch, yaw) = (2.93837,3.06435,-3.13371) = (168.356,175.574,-179.54
9)

./cam2obj syn_test/dvdRobotsSyn_00007.pcd -near 0.05 -pass_z_min 0.27 -pass_z_max 0.55 -pass_x_min -0.15 -pass_x_max 0.15 -pass_y_min -0.12 -pass_y_max 0.12
(roll, pitch, yaw) = (3.01546,-2.74431,3.11617) = (172.773,-157.237,178.54
3)








./cam2obj  robot_dvd/dvdRobots_00001.pcd -near 0.05 -pass_z_min 0.27 -pass_z_max 0.55 -pass_x_min -0.15 -pass_x_max 0.15 -pass_y_min -0.12 -pass_y_max 0.12
