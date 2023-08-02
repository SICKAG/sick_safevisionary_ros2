cmake_minimum_required(VERSION 3.8)
project(sick_safevisionary_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ActiveMonitoringCase.msg"
  "msg/CameraIO.msg"
  "msg/DeviceStatus.msg"
  "msg/FieldInformation.msg"
  "msg/FieldInformationArray.msg"
  "msg/GeneralStatus.msg"
  "msg/IOConfigured.msg"
  "msg/IODirection.msg"
  "msg/IOInputValues.msg"
  "msg/IOOSSDSState.msg"
  "msg/IOOutputValues.msg"
  "msg/ROI.msg"
  "msg/ROIArray.msg"
  "msg/ROIObservationResultData.msg"
  "msg/ROIObservationSafetyData.msg"
  DEPENDENCIES
    std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
