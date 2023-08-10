# Sick Safevisionary Driver

This is the official ROS2 driver for the Sick *safeVisionary2* cameras.
See the [top-level readme](./../README.md) for getting started.


## Lifecycle states
This driver implements a thin ROS2 wrapper around the [sick_safevisionary_base](https://github.com/SICKAG/sick_safevisionary_base) library in form of a *lifecycle node*.
In contrast to conventional ROS2 nodes, lifecycle nodes give us more control about the
driver's individual states. This is handy e.g. when performing a clean reset at runtime.
You'll find more information on the individual states and state transitions [in this design article](https://design.ros2.org/articles/node_lifecycle.html).

This driver's behavior is roughly as follows:

| State    | Behavior |
| -------- | ------- |
| Unconfigured  | No topics are advertised and previously advertised topics are removed.|
| Inactive | The driver establishes a UDP data connection to the camera and processes sensor data without publishing. |
| Active    | The driver continuously publishes all camera data with a consistent time stamp across all topics. |
| Finalized    | The driver has been shutdown and all resources have been cleaned up. All previously advertised topics are removed. |

## Managing the lifecycle
ROS2 has a command line interface to trigger state transitions.
Here are the commands to get the */sick_safevisionary* node through its primary states:

```bash
ros2 lifecycle set /sick_safevisionary configure
ros2 lifecycle set /sick_safevisionary activate
ros2 lifecycle set /sick_safevisionary deactivate
ros2 lifecycle set /sick_safevisionary shutdown
```

Also see this [example](../sick_safevisionary_tests/integration_tests/integration_tests.py) from the integration tests how to do that in Python.
