# roboarm
Robotic manipulator simulation using the MoveIt! Motion Planning Framework

To configure and launch the simulation in your own workspace:
```
cd ~/your_catkin_ws/src
git clone https://github.com/daniel-s-ingram/roboarm.git
cd roboarm
mv roboarm_moveit_configuration ../
catkin build roboarm_moveit_configuration
source ~/your_catkin_ws/devel/setup.bash
roslaunch roboarm roboarm.launch
```
Building roboarm_moveit_configuration also builds roboarm.
