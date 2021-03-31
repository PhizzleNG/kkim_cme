# Construction Manipulation Environment (CME)
Project name is just random, helpful with package organization.

More documentation can be found in [cme_docs](cme_docs/docs).

## Quickstart
### Installation/setup
```
mkdir -p ~/cme_ws/src
cd ~/cme_ws/src
git clone git+ssh://git@github.com/MyNameIsCosmo/kkim_cme.git
bash kkim_cme/bootstrap_ws.sh
```

### Test the world
Simulate the world in Gazebo, with door control.
```
roslaunch cme_launch test_world.launch
./test_doors.sh
rosservice call /world/door_1/open
rosservice call /world/door_1/close
```

### Simulate a ClearPath Husky in the world
Simulate the world and a Husky robot with Gazebo.  
Launches Gazebo, RVIZ, the CME world, a robot, and door control.
```
roslaunch cme_launch full.launch gazebo:=true rviz:=true robot:=husky manipulator:=
ROS_NAMESPACE=/husky rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

### Simulate a Neobotix MM_400 in the world
Simulate the world and a robot with Gazebo.  
Launches Gazebo, RVIZ, the CME world, a robot, and door control.

!!! IMPORTANT !!!
The `pilz` manipulator is broken in the Gazebo simulation, use the `panda` manipulator for now.

```
roslaunch cme_launch full.launch gazebo:=true rviz:=true robot:=mm_400 manipulator:=panda
ROS_NAMESPACE=/mm_400 rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

#### Running gmapping
```
roslaunch cme_launch gmapping.launch robot:=husky
```

#### Running exploration
```
roslaunch cme_lanuch exploration.launch
```

#### Running keyboard teleop
```
ROS_NAMESPACE=/ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

## Tasks
Overview of xacro file at [complete_model.urdf.xacro](cme_description/urdf/complete_model.urdf.xacro).

### Doors
Doors are created using a xacro macro defined in [door.xacro.xml](cme_description/urdf/door.xacro.xml).  
Documentation on usage available in the door xacro document.

![](cme_docs/docs/assets/img/door_joint_trail.png)

### Light Switches
Light switches are created using a xacro macro defined in [light_switch.xacro.xml](cme_description/urdf/light_switch.xacro.xml).

![](cme_docs/docs/assets/img/light_switch.png)
