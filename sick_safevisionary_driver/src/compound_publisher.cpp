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
  pointcloud_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("points", 1);
  imu_pub_ = node->create_publisher<sensor_msgs::msg::Imu>("imu_data", 1);
}

void CompoundPublisher::publish(
  const std_msgs::msg::Header & header, const visionary::SafeVisionaryData & data)
{
  if (camera_info_pub_->get_subscription_count() > 0) {
    publishCameraInfo(header, data);
  }
  if (pointcloud_pub_->get_subscription_count() > 0) {
    publishPointCloud(header, data);
  }
  if (imu_pub_->get_subscription_count() > 0) {
    publishIMUData(header, data);
  }
}

void CompoundPublisher::activate()
{
  camera_info_pub_->on_activate();
  pointcloud_pub_->on_activate();
  imu_pub_->on_activate();
}

void CompoundPublisher::deactivate()
{
  camera_info_pub_->on_deactivate();
  pointcloud_pub_->on_deactivate();
  imu_pub_->on_deactivate();
}

void CompoundPublisher::reset()
{
  camera_info_pub_.reset();
  pointcloud_pub_.reset();
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
}  // namespace sick
