[![License](https://img.shields.io/badge/License-Apache_2.0-yellow.svg)](https://opensource.org/licenses/Apache-2.0)
![build badge](https://github.com/SICKAG/sick_safevisionary_ros2/actions/workflows/industrial_ci_humble_action.yml/badge.svg)

# Sick Safevisionary ROS2
This is the official ROS2 driver for the [Sick safeVisionary2](https://www.sick.com/de/en/safety-camera-sensors/safety-camera-sensors/safevisionary2/c/g568562) cameras.

## System dependencies
We use *Boost*'s [lock-free](https://www.boost.org/doc/libs/1_82_0/doc/html/lockfree.html) data structures in this driver.
You can install them with
```bash
sudo apt-get install libboost-all-dev
```

## Build and install
Switch to the `src` folder of your current ROS2 workspace and
```bash
git clone https://github.com/SICKAG/sick_safevisionary_ros2.git
git clone https://github.com/SICKAG/sick_safevisionary_base.git
rosdep install --from-paths ./ --ignore-src -y
cd ..
colcon build --packages-select sick_safevisionary_base sick_safevisionary_interfaces sick_safevisionary_driver  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Getting started
In a sourced terminal, start the driver with
```bash
ros2 launch sick_safevisionary_driver driver_node.launch.py
```

This driver implements a *lifecycle node* and needs two additional steps to actually publish data.
Open another sourced terminal and call
```bash
ros2 lifecycle set /sick_safevisionary configure
ros2 lifecycle set /sick_safevisionary activate
```
You can list the relevant topics with
```bash
ros2 topic list | grep sick_safevisionary
```
Here's [more information](./sick_safevisionary_driver/README.md) about driver's lifecycle behavior.
