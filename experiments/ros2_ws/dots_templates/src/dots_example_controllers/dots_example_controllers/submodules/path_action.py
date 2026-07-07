# ------------------------------------------------------------------
# PATH ACTION CLIENT
#
# This module provides a lightweight wrapper around the ROS2 Action
# interface used for path execution. It handles the complete action lifecycle:
#
# Goal submission → Goal acceptance/rejection → Path execution
# → Result reception → Optional cancellation
#
# The wrapper exposes a simplified state machine that can be queried
# by higher-level navigation controllers.
# ------------------------------------------------------------------

import sys
import numpy as np
import random
import struct
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import math
from std_msgs.msg import Float64
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
import dots_devel_interfaces.action
import dots_devel_interfaces.msg

# ------------------------------------------------------------------
# ROS2 path action interface
#
# This helper class manages communication with the path-following
# action server. It keeps track of:
# - Whether a goal is currently active
# - Whether execution has completed
# - Whether a goal was rejected
# - The current action goal handle
# ------------------------------------------------------------------

class PathAction:
    def __init__(self, node):
        self.node = node
        self.action_client  = ActionClient(self.node, dots_devel_interfaces.action.Path, 'path')
        self.goal_done      = False
        self.busy           = False
        self.goal_rejected  = False
        self.goal_handle    = None

    # ------------------------------------------------------------------
    # Sends a new path-following request to the action server.
    #
    # The provided path is wrapped into a ROS2 Action goal and submitted
    # asynchronously. Subsequent updates are received through callback
    # functions registered with the ActionClient.
    # ------------------------------------------------------------------
    def send_goal(self, goal):
        self.node.get_logger().info('Send goal %f %f %f' % (
            goal.path[0].position.x, goal.path[0].position.y, goal.path[0].position.th))
        goal_msg = dots_devel_interfaces.action.Path.Goal()
        goal_msg.path = goal

        self.action_client.wait_for_server()
        self.node.get_logger().info('Got server')

        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
        self.busy           = True
        self.goal_rejected  = False
        self.goal_done      = False

    # ------------------------------------------------------------------
    # Handles the server response to a goal request.
    #
    # Once the server evaluates the request, it either:
    #
    # - Accepts the goal and starts execution
    # - Rejects the goal immediately
    #
    # If accepted, a result callback is registered so completion can be
    # detected asynchronously.
    # ------------------------------------------------------------------
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info('Goal rejected')
            self.goal_rejected  = True
            self.goal_done      = False
            self.busy           = False
            return
        self.node.get_logger().info('Goal accepted')
        self.goal_handle = goal_handle
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)
        self.goal_rejected  = False
        self.goal_done      = False
        self.busy           = True

    # ------------------------------------------------------------------
    # Invoked when path execution finishes.
    #
    # Reaching this callback means the action server has completed its
    # work and returned a final result. The internal execution state is
    # updated so higher-level logic can proceed with the next task.
    # ------------------------------------------------------------------

    def get_result_callback(self, future):
        result = future.result().result
        self.goal_done      = True
        self.goal_handle    = None
        self.busy           = False
        self.node.get_logger().info('Got action complete result')

    # ------------------------------------------------------------------
    # Requests cancellation of the currently active goal.
    #
    # Cancellation is performed asynchronously through the ROS2 Action
    # protocol. The final outcome is reported by the cancel callback.
    # ------------------------------------------------------------------

    def send_cancel(self):
        if self.goal_handle:
            self.node.get_logger().info('Sending goal cancel')
            self.send_cancel_future = self.goal_handle.cancel_goal_async()
            self.send_cancel_future.add_done_callback(self.cancel_done)
    
    # ------------------------------------------------------------------
    # Processes the cancellation response from the action server.
    #
    # A successful cancellation stops the active path execution and
    # returns the client to an idle state. If cancellation fails, the
    # action may still continue running on the server.
    # ------------------------------------------------------------------

    def cancel_done(self, future):
        if len(future.result().goals_canceling) > 0:
            self.node.get_logger().info('Goal cancelled')
        else:
            self.node.get_logger().info('Goal cancel failed')
        self.goal_done      = False
        self.busy           = False
        self.goal_handle    = None