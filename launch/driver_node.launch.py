# - BEGIN LICENSE BLOCK -------------------------------------------------------
# - END LICENSE BLOCK ---------------------------------------------------------

# -----------------------------------------------------------------------------
# \file    driver_node.launch.py
#
# \author  Stefan Scherzinger <scherzin@fzi.de>
# \date    2023/06/22
#
# -----------------------------------------------------------------------------

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    driver_node = Node(
        package="sick_safevisionary_ros2",
        executable="driver_node",
        parameters=[
            {"a": "/spacenav/twist"},
            {"b": "/target_wrench"},
            {"c": "world"},
            {"d": 50},
        ],
        prefix="screen -d -m gdb -command=/home/scherzin/.ros/my_debug_log --ex run --args",
        output="both",
    )


    return LaunchDescription([driver_node])
