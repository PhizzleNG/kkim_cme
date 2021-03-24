# Construction Manipulation Environment (CME)
Project name is just random, helpful with package organization.

## Quickstart
A quick-start script was added for reproducable builds.  
Run the following commands:
```
mkdir -p ~/cme_ws/src
cd ~/cme_ws/src
catkin_init_workspace
git clone git+ssh://git@github.com/MyNameIsCosmo/kkim_cme.git
bash kkim_cme/bootstrap_cme_ws.sh

cd ~/cme_ws
catkin_make -j$(nproc)

roslaunch 
```

More documentation can be found in [cme_docs](cme_docs/docs).

## Tasks
1) [x] Organize packages and dependencies
2) [ ] Add door xacro
3) [ ] Add light xacro
4) [ ] Husky + Kinova gen3 lite in simulation
5) [ ] MM400 + Panda in simulation
6) [ ] Robot navigation around map
7) [ ] Robot manipulator movement
8) [ ] Service to open/close door
9) [ ] Service to turn light on/off
10) [ ] Manipulate to a given point (eg. door handle)
	1) [ ] Move robot to closest point if out of reach
	2) [ ] Collision check with manipulator
	3) [ ] Interact with manipulator
11) [ ] Finish documentation
