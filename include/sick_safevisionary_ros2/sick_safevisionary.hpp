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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

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
};
