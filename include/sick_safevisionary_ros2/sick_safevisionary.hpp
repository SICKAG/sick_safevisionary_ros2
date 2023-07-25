// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    sick_safevisionary_ros2.hpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2023/06/22
 *
 */
//-----------------------------------------------------------------------------

#include <boost/lockfree/policies.hpp>
#include <boost/lockfree/spsc_queue.hpp>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sick_safevisionary_base/SafeVisionaryData.h"
#include "sick_safevisionary_base/SafeVisionaryDataStream.h"

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
};
}  // namespace sick
