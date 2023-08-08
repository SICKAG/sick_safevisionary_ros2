# Sick Safevisionary Tests
Integration tests and high-level concept validation for the Sick *safeVisionary2* driver.

Useful information on integration tests in ROS2 is available [here][1]

## Run tests manually
In the root of a sourced workspace, call
```bash
colcon test --packages-select sick_safevisionary_tests && colcon test-result --verbose
```
to run and inspect the integration tests locally.

You can clean the test results and outdated errors with
```bash
colcon test-result --delete-yes
```

[1]: https://github.com/ros2/launch/tree/master/launch_testing#quick-start-example
