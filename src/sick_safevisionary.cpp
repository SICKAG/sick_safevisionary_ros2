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

#include "sick_safevisionary_ros2/sick_safevisionary.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sick_safevisionary_base/SafeVisionaryData.h"

namespace sick
{
SickSafeVisionary::CallbackReturn SickSafeVisionary::on_configure(
  [[maybe_unused]] const rclcpp_lifecycle::State & previous_state)
{
  data_handle_ = std::make_shared<visionary::SafeVisionaryData>();
  data_stream_ = std::make_shared<visionary::SafeVisionaryDataStream>(data_handle_);

  // Setup UDP connection to the camera
  if (!data_stream_->openUdpConnection(htons(6060))) {
    RCLCPP_ERROR(this->get_logger(), "Could not open UDP connection");
    return CallbackReturn::ERROR;
  }

  // Start an asynchronous receive thread with a lock-free producer.
  receive_thread_ = std::thread([&]() {
    while (this->get_current_state().label() != "Finalized") {
      if (!data_stream_->getNextBlobUdp()) {
        RCLCPP_WARN(this->get_logger(), "UDP stream incomplete, skipping this frame.");
      }
      spsc_queue_.push(*data_handle_);
    }
  });

  // Start an asynchronous publish thread with a lock-free consumer.
  // We need to consume regardless of the current state, but publish only if we are active.
  publish_thread_ = std::thread([&]() {
    while (this->get_current_state().label() != "Finalized") {
      auto tmp = visionary::SafeVisionaryData();
      spsc_queue_.pop(tmp);
      if (this->get_current_state().label() == "Active") {
        // TODO: publish
      }
    }
  });

  return CallbackReturn::SUCCESS;
}

SickSafeVisionary::CallbackReturn SickSafeVisionary::on_activate(
  [[maybe_unused]] const rclcpp_lifecycle::State & previous_state)
{
  return CallbackReturn::SUCCESS;
}

SickSafeVisionary::CallbackReturn SickSafeVisionary::on_deactivate(
  [[maybe_unused]] const rclcpp_lifecycle::State & previous_state)
{
  return CallbackReturn::SUCCESS;
}

SickSafeVisionary::CallbackReturn SickSafeVisionary::on_cleanup(
  [[maybe_unused]] const rclcpp_lifecycle::State & previous_state)
{
  return CallbackReturn::SUCCESS;
}

SickSafeVisionary::CallbackReturn SickSafeVisionary::on_shutdown(
  [[maybe_unused]] const rclcpp_lifecycle::State & previous_state)
{
  receive_thread_.join();
  publish_thread_.join();
  data_stream_->closeUdpConnection();
  return CallbackReturn::SUCCESS;
}
}  // namespace sick
