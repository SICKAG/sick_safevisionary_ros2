// -- BEGIN LICENSE BLOCK -----------------------------------------------------
// -- END LICENSE BLOCK -------------------------------------------------------

//-----------------------------------------------------------------------------
/*!\file    driver_node.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2023/06/22
 *
 */
//-----------------------------------------------------------------------------

#include "sick_safevisionary_ros2/sick_safevisionary.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sick::SickSafeVisionary>()->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
