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
/*!\file    sick_safevisionary_ros2.hpp
 *
 * \author  Marvin Grosse Besselmann <grosse@fzi.de>
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2023-06-22
 *
 */
//-----------------------------------------------------------------------------

#ifndef SICK_SAFE_VISIONARY_HPP_INCLUDED
#define SICK_SAFE_VISIONARY_HPP_INCLUDED

#include <boost/lockfree/policies.hpp>
#include <boost/lockfree/spsc_queue.hpp>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sick_safevisionary_base/SafeVisionaryData.h"
#include "sick_safevisionary_base/SafeVisionaryDataStream.h"
#include "sick_safevisionary_driver/compound_publisher.hpp"

namespace sick
{
/**
 * @brief The primary driver class for the Sick SafeVisionary camera
 *
 * This class implements a *lifecycle node* to provide advanced options for managing the driver's internal state.
 * An overview of these states is given [here](https://design.ros2.org/articles/node_lifecycle.html)
 *
 * This driver's behavior is as follows:
 *
 * State: Unconfigured  -> Directly after starting the driver. Nothing happens yet.
 * State: Inactive      -> Connection to the camera established. Data is consumed, but not published to ROS2 yet.
 * State: Active        -> Data is published to ROS2.
 * State: Finalized     -> All resources are cleaned up.
 *
 * The data communication with the library is based on the *single producer,
 * single consumer* (spsc) pattern and uses Boost's lock-free, thread-safe buffer to publish
 * the camera's data to ROS2.
 *
 */
class SickSafeVisionary : public rclcpp_lifecycle::LifecycleNode
{
public:
  SickSafeVisionary() : LifecycleNode("sick_safevisionary") {}

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

private:
  void reset();

  std::shared_ptr<visionary::SafeVisionaryData> data_handle_;
  std::shared_ptr<visionary::SafeVisionaryDataStream> data_stream_;
  std::thread receive_thread_;
  std::thread publish_thread_;
  boost::lockfree::spsc_queue<visionary::SafeVisionaryData, boost::lockfree::capacity<10>>
    spsc_queue_;
  std::atomic<bool> continue_{false};

  std::unique_ptr<CompoundPublisher> data_publisher_;
};
}  // namespace sick

#endif
