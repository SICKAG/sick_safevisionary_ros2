// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    compound_publisher.hpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2023/07/27
 *
 */
//-----------------------------------------------------------------------------

#ifndef COMPUND_PUBLISHER_HPP_INCLUDED
#define COMPUND_PUBLISHER_HPP_INCLUDED

#include <memory>

#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/detail/camera_info__struct.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sick_safevisionary_base/PointXYZ.h"
#include "sick_safevisionary_base/SafeVisionaryData.h"
#include "sick_safevisionary_interfaces/msg/camera_io.hpp"
#include "sick_safevisionary_interfaces/msg/device_status.hpp"
#include "sick_safevisionary_interfaces/msg/field_information_array.hpp"
#include "sick_safevisionary_interfaces/msg/roi_array.hpp"
#include "std_msgs/msg/header.hpp"

namespace sick
{
/**
 * @brief A compound publisher for the driver's topics
 *
 * This class encapsulates all necessary publishers to keep the driver's
 * lifecycle management nice and clear.
 */
class CompoundPublisher
{
public:
  // Support initialization with a lifecylce node
  CompoundPublisher(rclcpp_lifecycle::LifecycleNode * node);

  // Limit the rest
  CompoundPublisher() = delete;
  CompoundPublisher & operator=(CompoundPublisher &&) = delete;
  CompoundPublisher(const CompoundPublisher & other) = delete;
  CompoundPublisher(CompoundPublisher &&) = delete;
  CompoundPublisher & operator=(const CompoundPublisher &) = delete;

  /**
     * @brief Publish once with all registered publishers
     *
     * @param header Header for this data
     * @param data The sensor's data to publish
     */
  void publish(const std_msgs::msg::Header & header, visionary::SafeVisionaryData & frame_data);

  /**
     * @brief Call this function in the lifecycle node's `on_activate`.
     */
  void activate();

  /**
     * @brief Call this function in the lifecycle node's `on_deactivate`.
     */
  void deactivate();

  /**
     * @brief Reset all publishers
     *
     * This destroys all registered publishers such that their topics are no longer advertised.
     * Call this function in the lifecycle node's `on_cleanup` and `on_shutdown`.
     * It's save to call this function multiple times.
     */
  void reset();

private:
  void publishCameraInfo(
    const std_msgs::msg::Header & header, const visionary::SafeVisionaryData & frame_data);

  void publishPointCloud(
    const std_msgs::msg::Header & header, visionary::SafeVisionaryData & frame_data);

  void publishIMUData(
    const std_msgs::msg::Header & header, const visionary::SafeVisionaryData & frame_data);

  void publishDeviceStatus(
    const std_msgs::msg::Header & header, const visionary::SafeVisionaryData & frame_data);

  void publishIOs(
    const std_msgs::msg::Header & header, const visionary::SafeVisionaryData & frame_data);

  void publishROI(
    const std_msgs::msg::Header & header, const visionary::SafeVisionaryData & frame_data);

  void publishFieldInformation(
    const std_msgs::msg::Header & header, const visionary::SafeVisionaryData & frame_data);

  void publishDepthImage(
    const std_msgs::msg::Header & header, const visionary::SafeVisionaryData & frame_data);

  void publishIntensityImage(
    const std_msgs::msg::Header & header, const visionary::SafeVisionaryData & frame_data);

  void publishStateMap(
    const std_msgs::msg::Header & header, const visionary::SafeVisionaryData & frame_data);

  sensor_msgs::msg::Image::SharedPtr Vec16ToImage(
    const std_msgs::msg::Header & header, const visionary::SafeVisionaryData & frame_data,
    std::vector<uint16_t> vec);

  sensor_msgs::msg::Image::SharedPtr Vec8ToImage(
    const std_msgs::msg::Header & header, const visionary::SafeVisionaryData & frame_data,
    std::vector<uint8_t> vec);

  rclcpp_lifecycle::LifecycleNode * node_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CameraInfo>>
    camera_info_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>>
    pointcloud_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>> imu_pub_;
  std::shared_ptr<
    rclcpp_lifecycle::LifecyclePublisher<sick_safevisionary_interfaces::msg::DeviceStatus>>
    device_status_pub_;
  std::shared_ptr<
    rclcpp_lifecycle::LifecyclePublisher<sick_safevisionary_interfaces::msg::CameraIO>>
    io_pub_;
  std::shared_ptr<
    rclcpp_lifecycle::LifecyclePublisher<sick_safevisionary_interfaces::msg::ROIArray>>
    roi_pub_;

  std::shared_ptr<
    rclcpp_lifecycle::LifecyclePublisher<sick_safevisionary_interfaces::msg::FieldInformationArray>>
    field_pub_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>> depth_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>> intensity_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>> state_pub_;
};

}  // namespace sick

#endif
