# Construction Manipulation Environment (CME)
Project name is just random, helpful with package organization.

## Quickstart
A quick-start script was added for reproducable builds.  
Run the following commands:
```
mkdir -p ~/cme_ws/src
cd ~/cme_ws/src
git clone git+ssh://git@github.com/MyNameIsCosmo/kkim_cme.git
bash kkim_cme/bootstrap_cme_ws.sh

source ~/cme_ws/devel/setup.bash
roslaunch cme_launcher test_world.launch
```

More documentation can be found in [cme_docs](cme_docs/docs).

## Tasks
Overview of xacro file at [complete_model.urdf.xacro](cme_description/urdf/complete_model.urdf.xacro).

### Doors
Doors are created using a xacro macro defined in [door.xacro.xml](cme_description/urdf/door.xacro.xml).  
Documentation on usage available in the door xacro document.

![](cme_docs/docs/assets/img/door_joint_trail.png)

### Light Switches
Light switches are created using a xacro macro defined in [light_switch.xacro.xml](cme_description/urdf/light_switch.xacro.xml).

![](cme_docs/docs/assets/img/light_switch.png)

## Todo
- [x] Organize packages and dependencies
- [x] Add door xacro
- [x] Add light xacro
- [ ] Joint controller for door/lights
- [ ] Service to open/close door
- [ ] Service to turn light on/off
- [ ] Husky + Kinova gen3 lite in simulation
- [ ] MM400 + Panda in simulation
- [ ] Gravity/collision checks
- [ ] Robot navigation around map
- [ ] Robot manipulator movement
- [ ] Manipulate to a given point (eg. door handle)
	- [ ] Move robot to closest point if out of reach
	- [ ] Collision check with manipulator
	- [ ] Interact with manipulator
- [ ] Finish documentation
