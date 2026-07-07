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
import json
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
from datetime import datetime

# ------------------------------------------------------------------
# ROS2 path action interface
#
# This helper class manages communication with the path-following
# action server while also publishing execution metadata for
# experiment recording and offline analysis.
#
# Besides handling the complete action lifecycle, the class logs:
# - Submitted path goals
# - Goal acceptance/rejection events
# - Goal completion events
# - Goal cancellation requests and results
#
# The published information can be recorded through ROSbag and later
# used to reconstruct navigation behavior during experiments.
# ------------------------------------------------------------------

class PathAction:
    def __init__(self, node):
        self.node = node
        self.action_client  = ActionClient(self.node, dots_devel_interfaces.action.Path, 'path')
        self.goal_done      = False
        self.busy           = False
        self.goal_rejected  = False
        self.goal_handle    = None

        # ROSbag experiment topics - provide detailed metadata about path goals and execution status
        
        # Goals submitted to the action server
        self.path_goals_pub = self.node.create_publisher(String,"/path_goals",10)

        # Action lifecycle events and execution status
        self.path_action_status_pub = self.node.create_publisher(String,"/path_action_status",10)

        # Sequential identifier assigned to each submitted goal
        self.goal_counter = 0

        # Stores metadata associated with the currently active goal
        self.current_goal_metadata = None

    # ------------------------------------------------------------------
    # Sends a new path-following request to the action server.
    #
    # Before submitting the action goal, the path information is
    # serialized and published to a dedicated ROS topic so it can
    # be recorded through ROSbag.
    #
    # The published metadata includes: Goal and robot identifier, number 
    # of waypoints and position and velocity information for each waypoint
    #
    # After publishing the metadata, the goal is submitted
    # asynchronously to the action server.
    # ------------------------------------------------------------------

    def send_goal(self, goal):

        self.goal_counter += 1

        robot_name = self.node.get_namespace()[1:]

        self.node.get_logger().info('Send goal %f %f %f' % (
            goal.path[0].position.x, goal.path[0].position.y, goal.path[0].position.th))
        
        # Build goal metadata structure for experiment logging.
        goal_data = {
            "timestamp": str(self.node.get_clock().now().nanoseconds),
            "robot": robot_name,
            "goal_id": self.goal_counter,
            "num_waypoints": len(goal.path),
            "waypoints": []
        }

        for idx, wp in enumerate(goal.path):
            goal_data["waypoints"].append({
                "index": idx,
                "x": wp.position.x,
                "y": wp.position.y,
                "th": wp.position.th,
                "x": wp.velocity.x,
                "y": wp.velocity.y,
                "w": wp.velocity.w,
                "x": wp.max_velocity.x,
                "y": wp.max_velocity.y,
                "w": wp.max_velocity.w
            })

        self.current_goal_metadata = goal_data

        msg = String()
        msg.data = json.dumps(goal_data)

        self.path_goals_pub.publish(msg)

        self.publish_action_status("goal_sent")

        # Submit action request to the path execution server.
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
    # The action server may either:
    #
    # - Accept the goal and start execution
    # - Reject the goal immediately
    #
    # All state transitions are published so they can be tracked
    # during experiment replay and analysis.
    # ------------------------------------------------------------------

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:

            self.node.get_logger().info('Goal rejected')
            self.goal_rejected  = True
            self.goal_done      = False
            self.busy           = False

            self.publish_action_status("goal_rejected")

            return
        
        self.node.get_logger().info('Goal accepted')

        self.publish_action_status("goal_accepted")

        self.goal_handle = goal_handle

        self.get_result_future = goal_handle.get_result_async()

        self.get_result_future.add_done_callback(self.get_result_callback)

        self.goal_rejected  = False
        self.goal_done      = False
        self.busy           = True

    # ------------------------------------------------------------------
    # Invoked when path execution finishes.
    #
    # Once the action server reports completion, the internal state
    # is updated and a completion event is published.
    #
    # The result object can be extended in the future if additional
    # execution metrics become available from the action server.
    # ------------------------------------------------------------------

    def get_result_callback(self, future):

        result = future.result().result

        self.goal_done      = True
        self.goal_handle    = None
        self.busy           = False

        self.node.get_logger().info('Got action complete result')

        self.publish_action_status("goal_completed")

    # ------------------------------------------------------------------
    # Requests cancellation of the currently active goal.
    #
    # Cancellation is performed asynchronously through the ROS2
    # Action protocol. A cancellation request event is published
    # immediately before contacting the action server.
    # ------------------------------------------------------------------

    def send_cancel(self):

        if self.goal_handle:

            self.node.get_logger().info('Sending goal cancel')

            self.publish_action_status("goal_cancel_requested")

            self.send_cancel_future = self.goal_handle.cancel_goal_async()
            self.send_cancel_future.add_done_callback(self.cancel_done)
    
    # ------------------------------------------------------------------
    # Processes the cancellation response from the action server.
    #
    # A successful response indicates that the action server has
    # acknowledged the cancellation request.
    #
    # The outcome is published to the experiment logging topic so
    # cancellation behavior can be analyzed offline.
    # ------------------------------------------------------------------

    def cancel_done(self, future):

        if len(future.result().goals_canceling) > 0:
            self.node.get_logger().info('Goal cancelled')

            self.publish_action_status("goal_cancelled")

        else:
            self.node.get_logger().info('Goal cancel failed')

            self.publish_action_status("goal_cancel_failed")


        self.goal_done      = False
        self.busy           = False
        self.goal_handle    = None

    # ------------------------------------------------------------------
    # Publishes an action lifecycle event.
    #
    # This information is intended for ROSbag recording and allows
    # complete reconstruction of the action execution timeline.
    # ------------------------------------------------------------------

    def publish_action_status(self, event_name, extra_data=None):

        status = {
            "timestamp": str(self.node.get_clock().now().nanoseconds),
            "robot": self.node.get_namespace()[1:],
            "event": event_name,
            "busy": self.busy,
            "goal_done": self.goal_done,
            "goal_rejected": self.goal_rejected,
        }

        if self.current_goal_metadata is not None:
            status["goal_id"] = self.current_goal_metadata["goal_id"]

        if extra_data is not None:
            status["extra_data"] = extra_data

        msg = String()
        msg.data = json.dumps(status)

        self.path_action_status_pub.publish(msg)