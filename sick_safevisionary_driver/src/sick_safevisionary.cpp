// -- BEGIN LICENSE BLOCK -----------------------------------------------------
/*!
*  Copyright (C) 2023, SICK AG, Waldkirch, Germany
*  Copyright (C) 2023, FZI Forschungszentrum Informatik, Karlsruhe, Germany
*
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*    http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    sick_safevisionary.cpp
 *
 * \author  Marvin Grosse Besselmann <grosse@fzi.de>
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2023-07-20
 *
 */
//-----------------------------------------------------------------------------

#include "sick_safevisionary_driver/sick_safevisionary.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sick_safevisionary_base/SafeVisionaryData.h"

namespace sick
{
namespace
{
const char * dataStreamErrorToString(visionary::DataStreamError error)
{
  switch (error)
  {
    case visionary::DataStreamError::OK:
      return "no stream error reported";
    case visionary::DataStreamError::DATA_RECEIVE_TIMEOUT:
      return "no sensor data received";
    case visionary::DataStreamError::CONNECTION_CLOSED:
      return "sensor connection closed";
    case visionary::DataStreamError::PARSE_XML_ERROR:
      return "XML metadata parse error";
    case visionary::DataStreamError::INVALID_VERSION_UDP_HEADER:
      return "invalid UDP header version";
    case visionary::DataStreamError::INVALID_PACKET_TYPE_UDP_HEADER:
      return "invalid UDP packet type";
    case visionary::DataStreamError::INVALID_LENGTH_UDP_HEADER:
      return "invalid UDP header length";
    case visionary::DataStreamError::INVALID_CRC_UDP_HEADER:
      return "invalid UDP header checksum";
    case visionary::DataStreamError::INVALID_BLOB_HEADER:
      return "invalid Blob header";
    case visionary::DataStreamError::INVALID_VERSION_BLOB_HEADER:
      return "invalid Blob header version";
    case visionary::DataStreamError::INVALID_PACKET_TYPE_BLOB_HEADER:
      return "invalid Blob packet type";
    case visionary::DataStreamError::INVALID_BLOB_ID:
      return "invalid Blob ID";
    case visionary::DataStreamError::INVALID_BLOB_NUMBER:
      return "invalid Blob number";
    case visionary::DataStreamError::INVALID_UDP_FRAGMENT_NUMBER:
      return "invalid UDP fragment number";
    case visionary::DataStreamError::DATA_SEGMENT_DEPTHMAP_ERROR:
      return "depth map segment parse error";
    case visionary::DataStreamError::DATA_SEGMENT_DEVICESTATUS_ERROR:
      return "Device Status segment parse error";
    case visionary::DataStreamError::DATA_SEGMENT_ROI_ERROR:
      return "ROI segment parse error";
    case visionary::DataStreamError::DATA_SEGMENT_LOCALIOS_ERROR:
      return "Local I/Os segment parse error";
    case visionary::DataStreamError::DATA_SEGMENT_FIELDINFORMATION_ERROR:
      return "Field Information segment parse error";
    case visionary::DataStreamError::DATA_SEGMENT_LOGICSIGNALS_ERROR:
      return "Logic Signals segment parse error";
    case visionary::DataStreamError::DATA_SEGMENT_IMU_ERROR:
      return "IMU segment parse error";
  }
  return "unknown data stream error";
}
}  // namespace

SickSafeVisionary::~SickSafeVisionary()
{
  reset();
}

SickSafeVisionary::CallbackReturn SickSafeVisionary::on_configure(
  [[maybe_unused]] const rclcpp_lifecycle::State & previous_state)
{
  tcp_ = (this->has_parameter("tcp")) ? this->get_parameter("tcp").as_bool()
                                       : this->declare_parameter("tcp", false);
  int port = (this->has_parameter("port")) ? this->get_parameter("port").as_int()
                                           : this->declare_parameter("port", tcp_ ? 2114 : 6060);
  const std::string device_ip =
    (this->has_parameter("device_ip"))
      ? this->get_parameter("device_ip").as_string()
      : this->declare_parameter("device_ip", "");
  const int stream_port = port;

  if (tcp_ && device_ip.empty()) {
    RCLCPP_WARN(this->get_logger(), "A device_ip is required when using the TCP stream.");
    return CallbackReturn::FAILURE;
  }

  data_handle_ = std::make_shared<visionary::SafeVisionaryData>();
  data_stream_ = std::make_shared<visionary::SafeVisionaryDataStream>(data_handle_);
  bool connection_open = false;
  if (tcp_) {
    connection_open = data_stream_->openTcpConnection(htons(port), device_ip);
  } else {
    connection_open = data_stream_->openUdpConnection(htons(port));
  }
  if (!connection_open) {
    RCLCPP_WARN(
      this->get_logger(), "Could not open %s connection on %s:%i.", tcp_ ? "TCP" : "UDP",
      tcp_ ? device_ip.c_str() : "local host", stream_port);
    return CallbackReturn::FAILURE;
  }

  // Check the connection when working with real hardware
  bool real_hw = (this->has_parameter("real_hw")) ? this->get_parameter("real_hw").as_bool()
                                                   : this->declare_parameter("real_hw", true);
  const bool first_blob_ok =
    !real_hw ||
    (tcp_ ? data_stream_->getNextBlobTcp(receive_buffer_packet_size_)
          : data_stream_->getNextBlobUdp());
  if (!first_blob_ok) {
    RCLCPP_WARN(
      this->get_logger(), "Sensor startup failed: %s on %s port %i.",
      dataStreamErrorToString(data_stream_->getLastError()), tcp_ ? "TCP" : "UDP", stream_port);
    if (tcp_) {
      data_stream_->closeTcpConnection();
    } else {
      data_stream_->closeUdpConnection();
    }
    return CallbackReturn::FAILURE;
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
      if (
        tcp_ ? data_stream_->getNextBlobTcp(receive_buffer_packet_size_)
             : data_stream_->getNextBlobUdp()) {
        spsc_queue_.push(*data_handle_);
      } else {
        RCLCPP_WARN(
          this->get_logger(), "%s stream incomplete, skipping this frame.", tcp_ ? "TCP" : "UDP");
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
  if (data_stream_) {
    if (tcp_) {
      data_stream_->closeTcpConnection();
    } else {
      data_stream_->closeUdpConnection();
    }
    data_stream_.reset();
  }

  if (data_publisher_) {
    data_publisher_->reset();
    data_publisher_.reset();
  }
}

}  // namespace sick
