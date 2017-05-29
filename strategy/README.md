* The step to execute straregy 

Step1.

    $ roslaunch arc control.launch

Step2.

    $ rosrun strategy s.py

step3.

    $ rosservice call /task "task_name: 'pick'"	(for pick task)<br>
    $ rosservice call /task "task_name: 'stow'"	(for stow task)<br>
