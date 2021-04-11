# TODO
- [ ] Fix joint control errors
- [ ] Patch doors to use gazebo control
- [ ] Verify gravity/collision checks
- [ ] Template robot actions
	- [ ] Create navigation action
	- [ ] Create manipulation action
	- [ ] Create door open action
	- [ ] Create light switch action
- [ ] namespaces? multimaster?
- [ ] reorg launch files
	- [ ] cme_launch/launch/robot/{control,...}.launch.xml
	- [ ] cme_launch/launch/manipulator/{control,...}.launch.xml
- [ ] env hooks
- [ ] rqt_launch


# 
- Action:
	- Turn on light in room X
		- Navigate from current room to light switch at room X
		- If door is closed, open door


stretch:
- skiros2: https://github.com/RVMI/skiros2
