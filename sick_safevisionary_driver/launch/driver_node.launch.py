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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Launch arguments
    port = DeclareLaunchArgument(
        "port",
        default_value="6060",
        description="Receiving UDP port on the driver's PC.",
    )
    frame_id = DeclareLaunchArgument(
        "frame_id",
        default_value="camera",
        description="`frame_id` in the published messages' header.",
    )
    real_hw = DeclareLaunchArgument(
        "real_hw",
        default_value="True",
        description="Whether this driver reads data from real hardware.",
    )
    args = [port, frame_id, real_hw]

    # The Sick safeVisionary2 driver
    driver_node = Node(
        package="sick_safevisionary_driver",
        executable="driver_node",
        # prefix="screen -d -m gdb -command=/home/scherzin/.ros/my_debug_log \
        # --ex run --args",
        output="both",
        parameters=[
            {"port": LaunchConfiguration("port")},
            {"frame_id": LaunchConfiguration("frame_id")},
            {"real_hw": LaunchConfiguration("real_hw")},
        ],
    )

    return LaunchDescription(args + [driver_node])
