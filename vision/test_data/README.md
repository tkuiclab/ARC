
# Test with object_detect(.jpg) & obj_pose(.pcd)

## Step 1 pub .jpg  with Image (_img)  &&  Topic (/image_topic)
```bash
rosrun hsv_vision pub_image _img:=/home/iclab/arc_ws/src/PCL_Test/SmoothNormal/pcd_testdata/left0003.jpg /image_topic:=/camera/rgb/image_raw
rosrun hsv_vision pub_image _img:=bin_expoEraser.png /image_topic:=/camera/rgb/image_raw
```

## Step 2 Run object_detect.py
```bash
rosrun darkflow_detect object_detect.py
```

## Step 3 Open obj_pose
```bash
#send pcd to topic
rosrun pcl_ros pcd_to_pointcloud  bin_expoEraser.pcd 0.5 /cloud_pcd:=/camera/depth_registered/points

#run object_pose_estimator in Bin
rosrun obj_pose object_pose_estimator -pass_x_min  -0.075 -pass_x_max 0.15 -pass_y_min -0.09 -pass_y_max 0.115
 -pass_z_max 0.68
```

## Step 4 Run Startegy

Modify s.py with 
s.stow.test_obj_pose('expoEraser')
'expoEraser' is item

```bash 
rosrun strategy s.py
```


## Save Topic to  JPG ###
rosrun image_view image_saver image:=/camera/rgb/image_raw

## pointcloud_to_pcd ##
rosrun pcl_ros pointcloud_to_pcd input:=/camera/depth_registered/points

rosrun pcl_ros pointcloud_to_pcd input:=/camera/depth/points


## pcd_to_pointcloud ##
$ rosrun pcl_ros pcd_to_pointcloud <file.pcd> [ <interval> ]
ex: 
$ rosrun pcl_ros pcd_to_pointcloud  1494491837667480.pcd 0.5 /cloud_pcd:=/camera/depth_registered/points

rosrun pcl_ros pcd_to_pointcloud  scene_cloud.pcd 0.5 /cloud_pcd:=/camera/depth_registered/points
