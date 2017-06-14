* The step to execute straregy 

Step1.

    $ roslaunch arc control.launch

Step2.

    $ rosrun strategy s.py

step3.

    $ rosservice call /task "task_name: 'pick'"	(for pick task)<br>
    $ rosservice call /task "task_name: 'stow'"	(for stow task)<br>


# About Suction
$ rosrun rosserial_python serial_node.py /dev/arc/arduino

$ rosservice call /robot_cmd "cmd: 'vacuumOn'" 
$ rosservice call /robot_cmd "cmd: 'vacuumOff'" 
$ rosservice call /robot_cmd "cmd: 'suctionUp'" 
$ rosservice call /robot_cmd "cmd: 'suctionDown'"

$ rosservice call /robot_cmd "cmd: 'calibration'"
$ rosservice call /robot_cmd "cmd: 'setMaxPos'"
$ rosservice call /robot_cmd "cmd: 'setMinPos'"


 