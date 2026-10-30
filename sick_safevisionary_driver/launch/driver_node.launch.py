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
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    # Launch arguments
    tcp = DeclareLaunchArgument(
        "tcp",
        default_value="False",
        description="Use the camera's TCP measurement stream instead of UDP.",
    )
    port = DeclareLaunchArgument(
        "port",
        default_value=PythonExpression(
            ["'2114' if '", LaunchConfiguration("tcp"), "'.lower() == 'true' else '6060'"]
        ),
        description="UDP receive port or TCP stream port, selected by `tcp`.",
    )
    device_ip = DeclareLaunchArgument(
        "device_ip",
        default_value="",
        description="Camera IP address used by the TCP stream.",
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
    args = [tcp, port, device_ip, frame_id, real_hw]

    # The SICK safeVisionary2 Driver
    driver_node = LifecycleNode(
        package="sick_safevisionary_driver",
        executable="driver_node",
        name="sick_safevisionary",
        namespace="",
        # prefix="screen -d -m gdb -command=/home/scherzin/.ros/my_debug_log \
        # --ex run --args",
        output="both",
        parameters=[
            {"port": LaunchConfiguration("port")},
            {"tcp": LaunchConfiguration("tcp")},
            {"device_ip": LaunchConfiguration("device_ip")},
            {"frame_id": LaunchConfiguration("frame_id")},
            {"real_hw": LaunchConfiguration("real_hw")},
        ],
    )

    # Configure the driver once it's launched
    configure_trans_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(driver_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    # Activate the driver once it's configured correctly
    node_activation_handle = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=driver_node,
            start_state="configuring",
            goal_state="inactive",
            handle_once=True,
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(driver_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    ld = LaunchDescription(args + [driver_node])
    ld.add_action(configure_trans_event)
    ld.add_action(node_activation_handle)
    return ld
