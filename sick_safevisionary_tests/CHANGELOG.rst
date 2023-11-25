^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sick_safevisionary_tests
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.3 (2023-11-25)
------------------

1.0.2 (2023-11-23)
------------------

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
* Use rclpy node features to list topics
* Add a readme for the driver package
  Also rename the sensor in the tests package for consistency.
* Add a launch file parameter for CI testing
  We now use a launch file parameter to specify if we are running as part
  of a CI pipeline and don't check the UDP connection if that's the case.
  Also provide the `port` and `frame_id` as launch file parameters.
* Added license and maintainer infos
* Fixed topic names in integration test
* Publish topics in node namespace
* Fixed error in unit test
* Add integration test for the publishers
* Add an integration test for repetitive driver resets
* Implement an integration test for lifecycle behavior
* Add readme on launching tests locally
* Add boilerplate code for integration tests
* Contributors: Marvin Große Besselmann, Stefan Scherzinger
