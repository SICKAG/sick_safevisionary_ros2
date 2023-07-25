# Sick Safevisionary ROS2


## Build and Install
We use *Boost*'s [lock-free](https://www.boost.org/doc/libs/1_82_0/doc/html/lockfree.html) data structures in this driver.
Install Boost with
```bash
sudo apt-get install libboost-all-dev
```
In a sourced terminal, run

```bash
colcon build --packages-select sick_safevisionary_ros2 --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

## Getting started
```bash
ros2 launch sick_safevisionary_ros2 driver_node.launch.py
ros2 lifecycle set /sick_safevisionary configure
ros2 lifecycle set /sick_safevisionary activate
```
