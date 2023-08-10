#!/usr/bin/env python3
import unittest

import launch_testing.actions
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State
import time


def generate_test_description():
    setup = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("sick_safevisionary_driver"),
                "launch",
                "driver_node.launch.py",
            ]
        ),
        launch_arguments={"real_hw": "False"}.items(),
    )
    until_ready = 3.0  # sec
    return LaunchDescription(
        [
            setup,
            TimerAction(
                period=until_ready, actions=[launch_testing.actions.ReadyToTest()]
            ),
        ]
    )


class IntegrationTest(unittest.TestCase):
    """An integration test for basic lifecycle behavior

    We test whether the driver's lifecycle behavior is as expected
    """

    def __init__(self, *args):
        super().__init__(*args)

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_lifecycle")
        cls.setup_interfaces(cls)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def setup_interfaces(self):
        """Setup interfaces for ROS2 services"""

        timeout = rclpy.time.Duration(seconds=5)
        self.change_state_client = self.node.create_client(
            ChangeState, "/sick_safevisionary/change_state"
        )

        self.change_state_client.wait_for_service(timeout.nanoseconds / 1e9)

        self.get_state_client = self.node.create_client(
            GetState, "/sick_safevisionary/get_state"
        )
        self.get_state_client.wait_for_service(timeout.nanoseconds / 1e9)

    def test_1_driver_reset(self):
        """Test if the driver supports resets

        We repetitively call `configure` and `cleanup` and check if that works.
        """
        self.change_state(Transition.TRANSITION_DEACTIVATE)
        self.change_state(Transition.TRANSITION_CLEANUP)
        for _ in range(3):
            self.change_state(Transition.TRANSITION_CONFIGURE)
            self.change_state(Transition.TRANSITION_CLEANUP)

        self.assertTrue(
            self.check_state(State.PRIMARY_STATE_UNCONFIGURED),
            "The driver supports repetitive resets.",
        )

    def test_2_publishers(self):
        """Test if all publishers behave correctly

        We check if all relevant topics are advertised after `configure` and
        whether they disappear after `cleanup`.
        """
        topics = [
            "/sick_safevisionary/camera_info",
            "/sick_safevisionary/points",
            "/sick_safevisionary/imu_data",
            "/sick_safevisionary/device_status",
            "/sick_safevisionary/camera_io",
            "/sick_safevisionary/region_of_interest",
            "/sick_safevisionary/fields",
            "/sick_safevisionary/depth",
            "/sick_safevisionary/intensity",
            "/sick_safevisionary/state",
        ]

        def list_topics():
            topics = self.node.get_topic_names_and_types()
            advertised = [t[0] for t in topics]
            return advertised

        # After configuration
        self.change_state(Transition.TRANSITION_CONFIGURE)
        time.sleep(1)  # wait some time to take effect
        topic_list = list_topics()
        for topic in topics:
            self.assertTrue(
                topic in topic_list,
                f"{topic} is advertised correctly after configuration.",
            )

        # After cleanup
        self.change_state(Transition.TRANSITION_CLEANUP)
        time.sleep(1)
        topic_list = list_topics()
        for topic in topics:
            self.assertTrue(
                topic not in topic_list, f"{topic} is removed correctly after cleanup."
            )

    def test_3_lifecycle(self):
        """Test all primary lifecycle states

        Test whether `configure` -> `activate` -> `deactivate` -> `shutdown` works.
        """
        # Startup
        self.assertTrue(
            self.check_state(State.PRIMARY_STATE_UNCONFIGURED),
            "The driver starts correctly.",
        )

        # configure
        self.change_state(Transition.TRANSITION_CONFIGURE)
        self.assertTrue(
            self.check_state(State.PRIMARY_STATE_INACTIVE),
            "The driver configures correctly.",
        )

        # activate
        self.change_state(Transition.TRANSITION_ACTIVATE)
        self.assertTrue(
            self.check_state(State.PRIMARY_STATE_ACTIVE),
            "The driver activates correctly.",
        )

        # deactivate
        self.change_state(Transition.TRANSITION_DEACTIVATE)
        self.assertTrue(
            self.check_state(State.PRIMARY_STATE_INACTIVE),
            "The driver deactivates correctly.",
        )

        # shutdown
        self.change_state(Transition.TRANSITION_INACTIVE_SHUTDOWN)
        self.assertTrue(
            self.check_state(State.PRIMARY_STATE_FINALIZED),
            "The inactive driver shuts down correctly.",
        )

    def change_state(self, transition_id):
        """Change the driver's current state

        Trigger the driver's state via the given transition id.
        """
        req = ChangeState.Request()
        req.transition.id = transition_id
        future = self.change_state_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result()

    def check_state(self, state_id):
        """Check the driver's state

        Return True if the driver's state has `state_id`, else False.
        """
        req = GetState.Request()
        future = self.get_state_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result().current_state.id == state_id
