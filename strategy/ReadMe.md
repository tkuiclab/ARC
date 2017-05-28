* The step to execute straregy 

Step1.
    $ roslaunch manipulator_h_manager b_manipulator_h_manager.launch (for version2)
    or
    $ roslaunch manipulator_h_manager manipulator_h_manager.launch (for version 1)


Step2.
    $ rosrun strategy s.py

step3.
   $ rosservice call /task "task_name: 'pick'"	(for pick task)
   $ rosservice call /task "task_name: 'stow'"	(for stow task)
