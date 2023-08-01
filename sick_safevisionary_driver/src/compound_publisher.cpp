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
}

void CompoundPublisher::publish(
  const std_msgs::msg::Header & header, const visionary::SafeVisionaryData & data)
{
  if (camera_info_pub_->get_subscription_count() > 0) {
    publishCameraInfo(header, data);
  }
}

void CompoundPublisher::activate() { camera_info_pub_->on_activate(); }

void CompoundPublisher::deactivate() { camera_info_pub_->on_deactivate(); }

void CompoundPublisher::reset() { camera_info_pub_.reset(); }

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

}  // namespace sick
