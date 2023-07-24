# Sick Safevisionary ROS2


## Build and Install
```bash
colcon build --packages-select sick_safevisionary_ros2 --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

## Getting started
```bash
ros2 launch sick_safevisionary_ros2 driver_node.launch.py
ros2 lifecycle set /sick_safevisionary configure
ros2 lifecycle set /sick_safevisionary activate
```
