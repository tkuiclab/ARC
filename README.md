# ARC :bicyclist:
TKU M-Bot for Amazon Robotics Challenge (ARC)


### Install

* Mount device to /dev/arc/LM1, LM2, manipulator

```bash
sudo cp ~/arc_ws/src/arc/install/99-arc.rules /etc/udev/rules.d/
```

* [librealsense](./vision/installation_librealsense.md)


### Run 

```bash
# Open Control 
sudo bash
roslaunch arc control.launch

# Open Vision 
roslaunch realsense_camera sr300_nodelet_rgbd.launch
rosrun darkflow_detect object_detect.py

# Stow Task
rosrun obj_pose object_pose_estimator 

# Open Strategy
rosrun strategy s.py
```

Open arc_ui/web/strategy.html


### Manipulator

7-DOF Manipulator in manipulator_7a/

### Vision

Vision in vision/

### Strategy

Strategy in strategy/

### UI

UI  in arc_ui/

Web data in arc_ui/web/

### Git reset (if git collapse)

```bash
# Copy update & Reset remote respository
mkdir -p ~/arc_ws_remote/src ; git init; cd ~/arc_ws_remote/src
git fetch
git checkout develop
cd ~/arc_ws_remote
cp -rf src/ src_remote_fail/
cd ~/arc_ws_remote/src
git log
git reset --hard 583179f20ffe326a39e03786889281d3c4e1faa7
git push -f origin develop


# Copy & Paste only upadte
cd ~/arc_ws/src
cp -rf  ~/arc_ws_remote/src_remote_fail/manipulator_7a .
git add .               
git commit -a
git push orgin develop   

```