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

class SickSafeVisionary : public rclcpp::Node
{
  public:
    SickSafeVisionary()
      : Node("sick_safevisionary")
    {
    }
};
