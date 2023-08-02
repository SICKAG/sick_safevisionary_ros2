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
  camera_info_pub_ = node->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 3);
  imu_pub_ = node->create_publisher<sensor_msgs::msg::Imu>("imu_data", 1);
}

void CompoundPublisher::publish(
  const std_msgs::msg::Header & header, const visionary::SafeVisionaryData & data)
{
  if (camera_info_pub_->get_subscription_count() > 0) {
    publishCameraInfo(header, data);
  }
  if (imu_pub_->get_subscription_count() > 0) {
    publishIMUData(header, data);
  }
}

void CompoundPublisher::activate()
{
  camera_info_pub_->on_activate();
  imu_pub_->on_activate();
}

void CompoundPublisher::deactivate()
{
  camera_info_pub_->on_deactivate();
  imu_pub_->on_deactivate();
}

void CompoundPublisher::reset()
{
  camera_info_pub_.reset();
  imu_pub_.reset();
}

void CompoundPublisher::publishCameraInfo(
  const std_msgs::msg::Header & header, const visionary::SafeVisionaryData & data)
{
  auto camera_info = sensor_msgs::msg::CameraInfo();
  camera_info.header = header;
  camera_info.height = data.getHeight();
  camera_info.width = data.getWidth();
  camera_info.d = std::vector<double>(5, 0);
  camera_info.d[0] = data.getCameraParameters().k1;
  camera_info.d[1] = data.getCameraParameters().k2;
  camera_info.d[2] = data.getCameraParameters().p1;
  camera_info.d[3] = data.getCameraParameters().p2;
  camera_info.d[4] = data.getCameraParameters().k3;

  camera_info.k[0] = data.getCameraParameters().fx;
  camera_info.k[2] = data.getCameraParameters().cx;
  camera_info.k[4] = data.getCameraParameters().fy;
  camera_info.k[5] = data.getCameraParameters().cy;
  camera_info.k[8] = 1;
  camera_info_pub_->publish(camera_info);
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
}  // namespace sick
