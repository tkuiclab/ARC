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

# See obj_pose result 
pcl_viewer -multiview 1 ~/arc_ws/src/vision/obj_pose/pcd_file/*

# Open strategy Web

Open arc_ui/web/strategy.html


# About Suction
$ rosrun rosserial_python serial_node.py /dev/arc/arduino

$ rosservice call /robot_cmd "cmd: 'vacuumOn'" 
$ rosservice call /robot_cmd "cmd: 'vacuumOff'" 
$ rosservice call /robot_cmd "cmd: 'suctionUp'" 
$ rosservice call /robot_cmd "cmd: 'suctionDown'"

$ rosservice call /robot_cmd "cmd: 'calibration'"
$ rosservice call /robot_cmd "cmd: 'setMaxPos'"
$ rosservice call /robot_cmd "cmd: 'setMinPos'"


 