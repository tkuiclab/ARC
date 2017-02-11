# ARC
TKU M-Bot for Amazon Robotics Challenge (ARC)


## Install

* Install Hint for ros-kinetic-manipulator-H
```bash
$ sudo apt-get install ros-kinetic-qt-build
```


## Manipulator Control

### Using Gazebo
    
* Open Manipulator Manager With Simulator   
    arg: en_sim, en_gui

```bash
$ roslaunch manipulator_h_manager manipulator_h_manager.launch en_sim:=true
```

* Manipulator Gazebo    
    arg: paused
    
```bash
$ roslaunch manipulator_h_gazebo manipulator_h_gazebo.launch
```

* Manipulator Test GUI
```bash
$ rosrun manipulator_h_gui manipulator_h_gui
```

* Publish Test Cmd    
    data: [x, y, z(m), pitch, roll, yaw(deg)]

```bash
$ rostopic pub /robotis/base/cmd_msg std_msgs/Float64MultiArray -1 "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [0, 0.3, 0.15, -90, 0, 90]"
```

### Using Real Robot

* Starting A New Bash Shell With The Security Privilege of Root User
```bash
$ sudo bash
```

* Open Manipulator Manager Without Simulator Using Root
```bash
# roslaunch manipulator_h_manager manipulator_h_manager.launch en_sim:=false
```

:sunglasses:
