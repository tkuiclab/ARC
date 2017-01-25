# ARC
TKU M-Bot for Amazon Robotics Challenge (ARC)


## Manipulator Control


### Using Gazebo
    
* Open Manipulator Manager With Simulator
```bash
$ roslaunch manipulator_h_manager manipulator_h_manager.launch en_sim:=true
```

* Manipulator Gazebo
```bash
$ roslaunch manipulator_h_gazebo manipulator_h_gazebo.launch
```

* Manipulator Test GUI
```bash
$ rosrun manipulator_h_gui manipulator_h_gui
```

### Using Real Robot

* Starting A New Bash Shell With The Security Privilege of Root User 
```bash
$ sudo bash
```
* Open Manipulator Manager Without Simulator
```bash
# roslaunch manipulator_h_manager manipulator_h_manager.launch en_sim:=false
```

