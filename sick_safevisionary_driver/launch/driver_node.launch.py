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
        # prefix="screen -d -m gdb -command=/home/scherzin/.ros/my_debug_log \
        # --ex run --args",
        output="both",
        parameters=[
            {"port": 6060}
        ]
    )

    return LaunchDescription([driver_node])
