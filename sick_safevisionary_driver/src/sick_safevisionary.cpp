// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    sick_safevisionary.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2023/07/20
 *
 */
//-----------------------------------------------------------------------------

#include "sick_safevisionary_driver/sick_safevisionary.hpp"

#include <chrono>
#include <memory>
#include <thread>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sick_safevisionary_base/SafeVisionaryData.h"

namespace sick
{
SickSafeVisionary::CallbackReturn SickSafeVisionary::on_configure(
  [[maybe_unused]] const rclcpp_lifecycle::State & previous_state)
{
  // Setup UDP connection to the camera
  int port = (this->has_parameter("port")) ? this->get_parameter("port").as_int()
                                           : this->declare_parameter("port", 6060);
  data_handle_ = std::make_shared<visionary::SafeVisionaryData>();
  data_stream_ = std::make_shared<visionary::SafeVisionaryDataStream>(data_handle_);
  if (!data_stream_->openUdpConnection(htons(port))) {
    RCLCPP_ERROR(this->get_logger(), "Could not open UDP connection on port: %i", port);
    return CallbackReturn::ERROR;
  }
  continue_ = true;
  data_publisher_ = std::make_unique<CompoundPublisher>(this);

  // Parameters
  if (!this->has_parameter("frame_id")) {
    this->declare_parameter("frame_id", "camera");
  };

  // Start an asynchronous receive thread with a lock-free producer.
  receive_thread_ = std::thread([&]() {
    while (continue_) {
      if (data_stream_->getNextBlobUdp()) {
        spsc_queue_.push(*data_handle_);
      } else {
        RCLCPP_WARN(this->get_logger(), "UDP stream incomplete, skipping this frame.");
      }
    }
  });

  // Start an asynchronous publish thread with a lock-free consumer.
  // We need to consume regardless of the current state, but publish only if we are active.
  publish_thread_ = std::thread([&]() {
    while (continue_) {
      auto data = visionary::SafeVisionaryData();
      if (
        spsc_queue_.pop(data) &&
        this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        auto header = std_msgs::msg::Header();
        header.frame_id = this->get_parameter("frame_id").as_string();
        header.stamp = this->now();
        data_publisher_->publish(header, data);
      } else {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
      }
    }
  });

  return CallbackReturn::SUCCESS;
}

SickSafeVisionary::CallbackReturn SickSafeVisionary::on_activate(
  [[maybe_unused]] const rclcpp_lifecycle::State & previous_state)
{
  data_publisher_->activate();
  return CallbackReturn::SUCCESS;
}

SickSafeVisionary::CallbackReturn SickSafeVisionary::on_deactivate(
  [[maybe_unused]] const rclcpp_lifecycle::State & previous_state)
{
  data_publisher_->deactivate();
  return CallbackReturn::SUCCESS;
}

SickSafeVisionary::CallbackReturn SickSafeVisionary::on_cleanup(
  [[maybe_unused]] const rclcpp_lifecycle::State & previous_state)
{
  reset();
  return CallbackReturn::SUCCESS;
}

SickSafeVisionary::CallbackReturn SickSafeVisionary::on_shutdown(
  [[maybe_unused]] const rclcpp_lifecycle::State & previous_state)
{
  reset();
  return CallbackReturn::SUCCESS;
}

void SickSafeVisionary::reset()
{
  continue_ = false;
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }
  if (publish_thread_.joinable()) {
    publish_thread_.join();
  }
  data_stream_->closeUdpConnection();

  data_publisher_->reset();
}

}  // namespace sick
