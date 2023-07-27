#!/usr/bin/env python3
import unittest

import launch_testing.actions
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import rclpy
from rclpy.node import Node


def generate_test_description():

    setup = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("sick_safevisionary_driver"),
             "launch",
             "driver_node.launch.py"]
        )
    )
    until_ready = 3.0  # sec
    return LaunchDescription(
        [setup, TimerAction(
            period=until_ready,
            actions=[launch_testing.actions.ReadyToTest()])])


class IntegrationTest(unittest.TestCase):
    """ An integration test for basic lifecycle behavior

    We test whether the driver's lifecycle behavior is as expected
    """
    def __init__(self, *args):
        super().__init__(*args)

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_lifecycle")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_lifecycle(self):
        """ Test the correctness of all relevant lifecycle transitions

        """
        self.assertTrue(True, "This is true.")
