^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sick_safevisionary_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.3 (2023-11-25)
------------------
* Use ROS version-dependent cv_bridge include
* Contributors: Stefan Scherzinger

1.0.2 (2023-11-23)
------------------
* Add rosdep key for `boost`
* Contributors: Stefan Scherzinger

1.0.1 (2023-11-22)
------------------
* Update maintainer tags
* Transition from configuring to active by itself once on launch (`#2 <https://github.com/SICKAG/sick_safevisionary_ros2/issues/2>`_)
  * Transition from configuring to active by itself once on launch
  * Initially deactivate and cleanup node during integration test
  * Added additional comments to clarify launch file
  Co-authored-by: Stefan Scherzinger <scherzin@fzi.de>
  * Updated ReadMe for auto lifecycle configuration
  * Updated driver ReadMe for auto lifecycle configuration
  Co-authored-by: Stefan Scherzinger <scherzin@fzi.de>
* Add a readme for the driver package
  Also rename the sensor in the tests package for consistency.
* Add a launch file parameter for CI testing
  We now use a launch file parameter to specify if we are running as part
  of a CI pipeline and don't check the UDP connection if that's the case.
  Also provide the `port` and `frame_id` as launch file parameters.
* Add configure check for the UDP connection
  The `configure` lifecycle transition now fails if there's no incoming
  sensor data, i.e. if something's wrong with the connection.
* Changed doxygen indentation
* Added doxygen comments
* Initialize shared pointer correctly
* Added license and maintainer infos
* Message variable cleanup
* added header to custom msgs
* Put pointcloud publisher to the end since its non const
* Check if intensity and points vector match
* Publish topics in node namespace
* Fixed dependency issues
* Publish depth,intensity and state map
* Publish field information
* Publish region of interest
* Publish ios
* Publish device status
* Naming consistency
* Publish Pointcloud
* Publish IMU data
* Only publish with a non-zero subscription count
  This keeps the driver's computations lean if nobody is listening.
* Make `frame_id` a parameter
  This allows users to set the `frame_id` of the published messages at
  runtime via the node's parameter interfaces.
* Use a unique pointer for the compound publisher
  That's cleaner and makes sure that the compound publisher is valid after construction.
* Add a publisher for camera info
  We use a separate class to encapsulate the different publishers so that
  the lifecycle part stays nice and clean.
* Format launch file with black
* Add an integration test for repetitive driver resets
* Add boilerplate code for integration tests
* Adjust includes from previous move
* Move relevant files into a subfolder
* Contributors: Marvin Große Besselmann, Stefan Scherzinger
