# Gazebo

## Building the world
The provided `mp_model` world was built in xacro/urdf and iterated on via rviz.

To make this a proper Gazebo environment, it needs to be converted into an SDF,
and then added to an empty Gazebo world.

```
xacro --inorder $(rospack find cme_description)/urdf/complete_model.urdf.xacro > /tmp/complete_model.urdf
gz sdf -p /tmp/complete_model.urdf > complete_model.sdf
```


## Troubleshooting

### Gazebo freezes when loading the world
Gazebo may freeze when waiting for a service or a plugin to load.
This may happen if it is looking for or loading models, as well as waiting for ROS services to be available.

This may be verified by checking for:
- if `libgazebo_ros_control.so` is in your gazebo world file or your robot description
