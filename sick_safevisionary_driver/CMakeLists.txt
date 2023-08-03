cmake_minimum_required(VERSION 3.8)
project(sick_safevisionary_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if (NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: selecting 'Release'.")
  set(CMAKE_BUILD_TYPE Release)
endif()

# find dependencies
find_package(Boost REQUIRED COMPONENTS)
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(sick_safevisionary_base REQUIRED)
find_package(sick_safevisionary_interfaces REQUIRED)

#--------------------------------------------------------------------------------
# Build
#--------------------------------------------------------------------------------
add_executable(driver_node
  src/compound_publisher.cpp
  src/sick_safevisionary.cpp
  src/driver_node.cpp
)

target_include_directories(driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(driver_node
  cv_bridge
  lifecycle_msgs
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  sick_safevisionary_interfaces
)

target_link_libraries(driver_node
  sick_safevisionary_base::sick_safevisionary_base
  Boost::boost
)

#--------------------------------------------------------------------------------
# Install
#--------------------------------------------------------------------------------
install(TARGETS driver_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

#--------------------------------------------------------------------------------
# Test
#--------------------------------------------------------------------------------
if(BUILD_TESTING)
endif()

ament_package()
