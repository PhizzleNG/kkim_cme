# Troubleshooting

## Gazebo

### Errors out when starting

If you see the following errors in the Gazebo output, your hardware does not support Gazebo:
```
...
libGL error: failed to create drawable
libGL error: failed to create drawable
```

Gazebo requires a dedicated graphics card (or newer integrated graphics).

### Param.cc nan errors
Gazebo may throw errors when it is unable to parse a file properly, errors such as `key[pose]` or `key[acceleration]` and other variants may pop up from a similar `Error [Param.cc]` line.
```
...
Error [Param.cc:451] Unable to set value [-nan -nan -nan 0 -0 0] for key[pose]
```

To resolve this, you should ensure all your `<link>`s have inertia and mass properties, and any mass properties are greater than 0.
```
<link>
    <inertial>
        <mass value="1"/>
        <origin rpy="0 0 0" xyz="0 0 0"/>
        <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
    </inertial>
</link>
```
