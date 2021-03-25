# Troubleshooting

## Gazebo

### Errors out when starting

If you see the following errors in the Gazebo output, your hardware does not support Gazebo:
```
...
libGL error: failed to create drawable
libGL error: failed to create drawable
...
Error [Param.cc:451] Unable to set value [-nan -nan -nan 0 -0 0] for key[pose]
```

Gazebo requires a dedicated graphics card (or newer integrated graphics).
