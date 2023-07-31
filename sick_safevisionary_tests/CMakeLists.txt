cmake_minimum_required(VERSION 3.8)
project(sick_safevisionary_tests)

find_package(ament_cmake REQUIRED)


if(BUILD_TESTING)
  find_package(sick_safevisionary_driver REQUIRED)
  find_package(launch_testing_ament_cmake)
  add_launch_test(integration_tests/integration_tests.py)
endif()

ament_package()
