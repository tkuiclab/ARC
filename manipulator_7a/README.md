# ARC :sunglasses:
TKU M-Bot for Amazon Robotics Challenge (ARC)


## Install

* Install Hint for ros-kinetic-manipulator-H

```bash
$ sudo apt-get install ros-kinetic-rosbridge-server ros-kinetic-rosserial
```

* Add the Rule for Device

```bash
$ sudo cp dev_rule/99-manipulator.rules /etc/udev/rules.d/
```


## Manipulator Control

### Using Gazebo
    
* Open Manipulator Manager With Simulator <br>
  arg: en_sim, en_gui, en_vac

```bash
$ roslaunch manipulator_h_manager manipulator_h_manager.launch en_sim:=true
```

* Manipulator Gazebo <br>
  arg: paused

```bash
$ roslaunch pro7a_description pro7a_gazebo.launch
```

* Publish Velocity <br>
  data: percent (%)

```bash
$ rostopic pub /robotis/base/set_velocity std_msgs/Float64 "data: 30" -1
```

* Publish Software Emergency Stop <br> 
  using software to stop manipulator motion.      
  **data: ''**

```bash
$ rostopic pub /robotis/base/set_mode_msg std_msgs/String "data: ''"  -1
```


### Using Real Robot

* Starting A New Bash Shell With The Security Privilege of Root User

```bash
$ sudo bash
```

* Open Manipulator Manager Without Simulator Using Root <br>
  arg: en_sim, en_gui, en_vac
```bash
# roslaunch manipulator_h_manager manipulator_h_manager.launch en_sim:=false
```
