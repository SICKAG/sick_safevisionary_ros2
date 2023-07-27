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


def generate_test_description():
    setup = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("sick_safevisionary_driver"),
                "launch",
                "driver_node.launch.py",
            ]
        )
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
        if not self.change_state_client.wait_for_service(timeout.nanoseconds / 1e9):
            self.fail("Service /sick_safevisionary/change_state not available")

        self.get_state_client = self.node.create_client(
            GetState, "/sick_safevisionary/get_state"
        )
        if not self.get_state_client.wait_for_service(timeout.nanoseconds / 1e9):
            self.fail("Service /sick_safevisionary/get_state not available")

    def test_lifecycle(self):
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
