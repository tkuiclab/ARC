* The step to execute straregy 

# Open Control

```bash
sudo bash
roslaunch arc control.launch
```

# Run Strategy 

```bash
rosrun straregy s.py
```

# Run Vison 

```bash
roslaunch realsense_camera sr300_nodelet_rgbd.launch
rosrun obj_pose object_pose_estimator
rosrun darkflow_detect object_detect.py            
```

# obj_pose 
# for stow
rosrun obj_pose object_pose_estimator 
-pass_x_min -0.24 -pass_x_max 0.24 -pass_y_min -0.18 -pass_y_max 0.18 -pass_z_min 0.3 -pass_z_max 0.57 -near 0.1

rosrun obj_pose object_pose_estimator -pass_x_min -0.13 -pass_x_max 0.13 -pass_y_min 0 -pass_y_max 0.4 -pass_z_min 0.3 -pass_z_max 0.6 -near 0.1

# See obj_pose result 
# in pcd_file/ directory
pcl_viewer -multiview 1 *.pcd

# Open strategy Web

Open arc_ui/web/strategy.html


# About Suction
$ rosrun rosserial_python serial_node.py /dev/arc/arduino

$ rosservice call /robot_cmd "cmd: 'vacuumOn'" 
$ rosservice call /robot_cmd "cmd: 'vacuumOff'" 
$ rosservice call /robot_cmd "cmd: 'suctionUp'" 
$ rosservice call /robot_cmd "cmd: 'suctionDown'"
$ rosservice call /robot_cmd "cmd: '30'"

$ rosservice call /robot_cmd "cmd: 'calibration'"
$ rosservice call /robot_cmd "cmd: 'setMaxPos'"
$ rosservice call /robot_cmd "cmd: 'setMinPos'"

# swicth obj_pose topic
/camera/depth_registered/points:=/camera/depth/points 
rosrun obj_pose object_pose_estimator  -pass_z_min 0.3 -pass_z_max 0.6 -pass_y_min 0  -pass_y_max 0.4 -near 0.1


# Test Stow Task
roslaunch arc control.launch
roslaunch realsense_camera sr300_nodelet_rgbd.launch
rosrun darkflow_detect object_detect.py
rosrun obj_pose object_pose_estimator  -pass_z_min 0.3 -pass_y_max 0.6 -pass_y_min 0 -near 0.1

rosrun strategy s.py
open web

# Test t2o_robot.py
roslaunch arc control.launch
roslaunch realsense_camera sr300_nodelet_rgbd.launch
rosrun darkflow_detect object_detect.py
rosrun obj_pose object_pose_estimator
(rosrun obj_pose object_pose_estimator /camera/depth_registered/points:=/camera/depth/points)
rosrun strategy t2o_robot.py
