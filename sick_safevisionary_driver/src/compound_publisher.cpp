// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    compound_publisher.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2023/07/27
 *
 */
//-----------------------------------------------------------------------------

#include "sick_safevisionary_driver/compound_publisher.hpp"

namespace sick
{
CompoundPublisher::CompoundPublisher(rclcpp_lifecycle::LifecycleNode * node)
{
  camera_info_pub_ = node->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 1);
  pointcloud_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("points", 1);
  imu_pub_ = node->create_publisher<sensor_msgs::msg::Imu>("imu_data", 1);
  device_status_pub_ =
    node->create_publisher<sick_safevisionary_interfaces::msg::DeviceStatus>("device_status", 1);
  io_pub_ = node->create_publisher<sick_safevisionary_interfaces::msg::CameraIO>("camera_io", 1);
}

void CompoundPublisher::publish(
  const std_msgs::msg::Header & header, visionary::SafeVisionaryData & frame_data)
{
  if (camera_info_pub_->get_subscription_count() > 0) {
    publishCameraInfo(header, frame_data);
  }
  if (pointcloud_pub_->get_subscription_count() > 0) {
    publishPointCloud(header, frame_data);
  }
  if (imu_pub_->get_subscription_count() > 0) {
    publishIMUData(header, frame_data);
  }
  if (device_status_pub_->get_subscription_count() > 0) {
    publishDeviceStatus(header, frame_data);
  }
  if (io_pub_->get_subscription_count() > 0) {
    publishIOs(header, frame_data);
  }
}

void CompoundPublisher::activate()
{
  camera_info_pub_->on_activate();
  pointcloud_pub_->on_activate();
  imu_pub_->on_activate();
  device_status_pub_->on_activate();
  io_pub_->on_activate();
}

void CompoundPublisher::deactivate()
{
  camera_info_pub_->on_deactivate();
  pointcloud_pub_->on_deactivate();
  imu_pub_->on_deactivate();
  device_status_pub_->on_deactivate();
  io_pub_->on_deactivate();
}

void CompoundPublisher::reset()
{
  camera_info_pub_.reset();
  pointcloud_pub_.reset();
  imu_pub_.reset();
  device_status_pub_.reset();
  io_pub_.reset();
}

void CompoundPublisher::publishCameraInfo(
  const std_msgs::msg::Header & header, const visionary::SafeVisionaryData & frame_data)
{
  auto camera_info = sensor_msgs::msg::CameraInfo();
  camera_info.header = header;
  camera_info.height = frame_data.getHeight();
  camera_info.width = frame_data.getWidth();
  camera_info.d = std::vector<double>(5, 0);
  camera_info.d[0] = frame_data.getCameraParameters().k1;
  camera_info.d[1] = frame_data.getCameraParameters().k2;
  camera_info.d[2] = frame_data.getCameraParameters().p1;
  camera_info.d[3] = frame_data.getCameraParameters().p2;
  camera_info.d[4] = frame_data.getCameraParameters().k3;

  camera_info.k[0] = frame_data.getCameraParameters().fx;
  camera_info.k[2] = frame_data.getCameraParameters().cx;
  camera_info.k[4] = frame_data.getCameraParameters().fy;
  camera_info.k[5] = frame_data.getCameraParameters().cy;
  camera_info.k[8] = 1;
  camera_info_pub_->publish(camera_info);
}

void CompoundPublisher::publishPointCloud(
  const std_msgs::msg::Header & header, visionary::SafeVisionaryData & frame_data)
{
  sensor_msgs::msg::PointCloud2::Ptr cloud_msg(new sensor_msgs::msg::PointCloud2);
  cloud_msg->header = header;
  cloud_msg->height = frame_data.getHeight();
  cloud_msg->width = frame_data.getWidth();
  cloud_msg->is_dense = false;
  cloud_msg->is_bigendian = false;

  cloud_msg->fields.resize(4);
  cloud_msg->fields[0].name = "x";
  cloud_msg->fields[1].name = "y";
  cloud_msg->fields[2].name = "z";
  cloud_msg->fields[3].name = "intensity";

  int offset = 0;
  for (size_t i = 0; i < 3; ++i) {
    cloud_msg->fields[i].offset = offset;
    cloud_msg->fields[i].datatype = int(sensor_msgs::msg::PointField::FLOAT32);
    cloud_msg->fields[i].count = 1;
    offset += sizeof(float);
  }

  cloud_msg->fields[3].offset = offset;
  cloud_msg->fields[3].datatype = int(sensor_msgs::msg::PointField::UINT16);
  cloud_msg->fields[3].count = 1;
  offset += sizeof(uint16_t);

  cloud_msg->point_step = offset;
  cloud_msg->row_step = cloud_msg->point_step * cloud_msg->width;
  cloud_msg->data.resize(cloud_msg->height * cloud_msg->row_step);

  std::vector<visionary::PointXYZ> point_vec;
  frame_data.generatePointCloud(point_vec);
  frame_data.transformPointCloud(point_vec);

  std::vector<uint16_t>::const_iterator intensity_it = frame_data.getIntensityMap().begin();
  std::vector<visionary::PointXYZ>::const_iterator point_it = point_vec.begin();
  // TODO check if both vector sizes align

  for (size_t i = 0; i < point_vec.size(); ++i, ++intensity_it, ++point_it) {
    memcpy(
      &cloud_msg->data[i * cloud_msg->point_step + cloud_msg->fields[0].offset], &*point_it,
      sizeof(visionary::PointXYZ));
    memcpy(
      &cloud_msg->data[i * cloud_msg->point_step + cloud_msg->fields[3].offset], &*intensity_it,
      sizeof(uint16_t));
  }
  pointcloud_pub_->publish(*cloud_msg);
}

void CompoundPublisher::publishIMUData(
  const std_msgs::msg::Header & header, const visionary::SafeVisionaryData & frame_data)
{
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header = header;
  imu_msg.angular_velocity.x = frame_data.getIMUData().angularVelocity.X;
  imu_msg.angular_velocity.y = frame_data.getIMUData().angularVelocity.Y;
  imu_msg.angular_velocity.z = frame_data.getIMUData().angularVelocity.Z;
  imu_msg.linear_acceleration.x = frame_data.getIMUData().acceleration.X;
  imu_msg.linear_acceleration.y = frame_data.getIMUData().acceleration.Y;
  imu_msg.linear_acceleration.z = frame_data.getIMUData().acceleration.Z;
  imu_msg.orientation.x = frame_data.getIMUData().orientation.X;
  imu_msg.orientation.y = frame_data.getIMUData().orientation.Y;
  imu_msg.orientation.z = frame_data.getIMUData().orientation.Z;
  imu_msg.orientation.w = frame_data.getIMUData().orientation.W;
  imu_pub_->publish(imu_msg);
}

void CompoundPublisher::publishDeviceStatus(
  const std_msgs::msg::Header & header, const visionary::SafeVisionaryData & frame_data)
{
  sick_safevisionary_interfaces::msg::DeviceStatus status;
  status.status = static_cast<uint8_t>(frame_data.getDeviceStatus());
  status.general_status.application_error =
    frame_data.getDeviceStatusData().generalStatus.applicationError;
  status.general_status.contamination_error =
    frame_data.getDeviceStatusData().generalStatus.contaminationError;
  status.general_status.contamination_warning =
    frame_data.getDeviceStatusData().generalStatus.contaminationWarning;
  status.general_status.dead_zone_detection =
    frame_data.getDeviceStatusData().generalStatus.deadZoneDetection;
  status.general_status.device_error = frame_data.getDeviceStatusData().generalStatus.deviceError;
  status.general_status.temperature_warning =
    frame_data.getDeviceStatusData().generalStatus.temperatureWarning;
  status.general_status.run_mode_active =
    frame_data.getDeviceStatusData().generalStatus.runModeActive;
  status.general_status.wait_for_cluster =
    frame_data.getDeviceStatusData().generalStatus.waitForCluster;
  status.general_status.wait_for_input =
    frame_data.getDeviceStatusData().generalStatus.waitForInput;
  status.cop_non_safety_related = frame_data.getDeviceStatusData().COPNonSaftyRelated;
  status.cop_safety_related = frame_data.getDeviceStatusData().COPSaftyRelated;
  status.cop_reset_required = frame_data.getDeviceStatusData().COPResetRequired;
  status.active_monitoring_case.monitoring_case_1 =
    frame_data.getDeviceStatusData().activeMonitoringCase.currentCaseNumberMonitoringCase1;
  status.active_monitoring_case.monitoring_case_2 =
    frame_data.getDeviceStatusData().activeMonitoringCase.currentCaseNumberMonitoringCase2;
  status.active_monitoring_case.monitoring_case_3 =
    frame_data.getDeviceStatusData().activeMonitoringCase.currentCaseNumberMonitoringCase3;
  status.active_monitoring_case.monitoring_case_4 =
    frame_data.getDeviceStatusData().activeMonitoringCase.currentCaseNumberMonitoringCase4;
  status.contamination_level = frame_data.getDeviceStatusData().contaminationLevel;
  device_status_pub_->publish(status);
}

void CompoundPublisher::publishIOs(
  const std_msgs::msg::Header & header, const visionary::SafeVisionaryData & frame_data)
{
  sick_safevisionary_interfaces::msg::CameraIO camera_io;
  camera_io.configured.pin_5 =
    frame_data.getLocalIOData().universalIOConfigured.configuredUniIOPin5;
  camera_io.configured.pin_6 =
    frame_data.getLocalIOData().universalIOConfigured.configuredUniIOPin6;
  camera_io.configured.pin_7 =
    frame_data.getLocalIOData().universalIOConfigured.configuredUniIOPin7;
  camera_io.configured.pin_8 =
    frame_data.getLocalIOData().universalIOConfigured.configuredUniIOPin8;
  camera_io.direction.pin_5 =
    frame_data.getLocalIOData().universalIODirection.directionValueUniIOPin5;
  camera_io.direction.pin_6 =
    frame_data.getLocalIOData().universalIODirection.directionValueUniIOPin6;
  camera_io.direction.pin_7 =
    frame_data.getLocalIOData().universalIODirection.directionValueUniIOPin7;
  camera_io.direction.pin_8 =
    frame_data.getLocalIOData().universalIODirection.directionValueUniIOPin8;
  camera_io.input_values.pin_5 =
    frame_data.getLocalIOData().universalIOInputValue.logicalValueUniIOPin5;
  camera_io.input_values.pin_6 =
    frame_data.getLocalIOData().universalIOInputValue.logicalValueUniIOPin6;
  camera_io.input_values.pin_7 =
    frame_data.getLocalIOData().universalIOInputValue.logicalValueUniIOPin7;
  camera_io.input_values.pin_8 =
    frame_data.getLocalIOData().universalIOInputValue.logicalValueUniIOPin8;
  camera_io.output_values.pin_5 =
    frame_data.getLocalIOData().universalIOOutputValue.localOutput1Pin5;
  camera_io.output_values.pin_6 =
    frame_data.getLocalIOData().universalIOOutputValue.localOutput2Pin6;
  camera_io.output_values.pin_7 =
    frame_data.getLocalIOData().universalIOOutputValue.localOutput3Pin7;
  camera_io.output_values.pin_8 =
    frame_data.getLocalIOData().universalIOOutputValue.localOutput4Pin8;
  camera_io.ossds_state.ossd1a = frame_data.getLocalIOData().ossdsState.stateOSSD1A;
  camera_io.ossds_state.ossd1b = frame_data.getLocalIOData().ossdsState.stateOSSD1B;
  camera_io.ossds_state.ossd2a = frame_data.getLocalIOData().ossdsState.stateOSSD2A;
  camera_io.ossds_state.ossd2b = frame_data.getLocalIOData().ossdsState.stateOSSD2B;
  camera_io.ossds_dyn_count = frame_data.getLocalIOData().ossdsDynCount;
  camera_io.ossds_crc = frame_data.getLocalIOData().ossdsCRC;
  camera_io.ossds_io_status = frame_data.getLocalIOData().ossdsIOStatus;
  camera_io.dynamic_speed_a = frame_data.getLocalIOData().dynamicSpeedA;
  camera_io.dynamic_speed_b = frame_data.getLocalIOData().dynamicSpeedB;
  camera_io.dynamic_valid_flags = frame_data.getLocalIOData().DynamicValidFlags;
  io_pub_->publish(camera_io);
}
}  // namespace sick
