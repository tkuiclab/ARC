# ARC :bicyclist:
TKU M-Bot for Amazon Robotics Challenge (ARC)


### Install

* Mount device to /dev/arc/LM1,LM2,manipulator

```bash
sudo cp ~/arc_ws/src/arc/install/99-arc.rules /etc/udev/rules.d/
```

* [librealsense](./vision/installation.md)

### Run 

```bash
sudo bash
roslaunch arc control.launch 
rosrun strategy s.py
rosservice call /task "task_name: 'pick'"
```

### Manipulator

7-DOF Manipulator in manipulator_7a/

### Vision

Vision in vision/

### Strategy

Strategy in strategy/

### UI

UI  in arc_ui/

Web data in arc_ui/web/
