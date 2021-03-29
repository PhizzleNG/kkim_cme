# CME Launch Files

## Overview
Launch files in `cme_launch` help facilitate running various tasks with various robots.  
Some robots use the same launch files to help keep similarities between operation and control.

## Configuration
Most launch files include argument configurations.  
Some launch files will substitute default arguments with environment variables.  

## Shared Launch Files
Launch files are shared between robots depending on functions.  
Common functions such as navigation, manipulation, etc, are started from the same files.

Robot configurations may either use a symbolic link to these shared files or import them along with
robot-specific params, nodes, etc.

- [amcl.launch.xml]: run the amcl navigation stack
- [control.launch.xml]: start common controllers
- [move_base.launch.xml]: start robot movement and transform nodes
- [spawn_robot.launch.xml]: load the robot description with gazebo and manipulator support


[amcl.launch.xml]: /cme_launch/launch/include/amcl.launch.xml
[control.launch.xml]: /cme_launch/launch/include/control.launch.xml
[move_base.launch.xml]: /cme_launch/launch/include/move_base.launch.xml
[spawn_robot.launch.xml]: /cme_launch/launch/include/spawn_robot.launch.xml
[]: /cme_launch/launch/include/
[]: /cme_launch/launch/include/

## Individual Launch Files

