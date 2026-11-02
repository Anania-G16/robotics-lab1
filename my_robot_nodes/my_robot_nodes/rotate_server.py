#!/usr/bin/env python3
"""
Action server that rotates diffdrive_bot in place to a target angle.

Measures actual rotation using wheel odometry, handles angle wrapping around
+/- pi, publishes feedback on the angle turned so far, and stops the robot
upon completion or cancellation.
"""

import math
import time

from geometry_msgs.msg import Twist
from my_robot_interfaces.action import Rotate
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node


def yaw_from_quaternion(q):
    """Rotation about the vertical axis, in rad, of an orientation quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_difference(target, current):
    """Target - current, wrapped to [-pi, pi] so the robot turns the short way."""
    return math.atan2(math.sin(target - current), math.cos(target - current))


class RotateServer(Node):

    def __init__(self):
        super().__init__('rotate_server')
        self.declare_parameter('turn_speed', 0.5)

        self.yaw = None

        group = ReentrantCallbackGroup()
        self.cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10, callback_group=group)
        self.action_server = ActionServer(
            self,
            Rotate,
            'rotate',
            execute_callback=self.execute,
            goal_callback=self.on_goal,
            cancel_callback=lambda goal_handle: CancelResponse.ACCEPT,
            callback_group=group,
        )

    def on_odom(self, msg: Odometry):
        self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)

    def on_goal(self, goal):
        # Reject invalid zero-angle goals or if odometry is not yet available
        if abs(goal.angle) < 1e-6 or self.yaw is None:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute(self, goal_handle):
        target_angle = goal_handle.request.angle
        self.get_logger().info(f'Rotating {target_angle:.2f} rad')

        turn_speed = self.get_parameter('turn_speed').value
        direction = 1.0 if target_angle > 0.0 else -1.0

        twist = Twist()
        twist.angular.z = direction * abs(float(turn_speed))

        feedback = Rotate.Feedback()
        result = Rotate.Result()

        total_turned = 0.0
        last_yaw = self.yaw

        try:
            while rclpy.ok():
                current_yaw = self.yaw
                delta = angle_difference(current_yaw, last_yaw)
                total_turned += delta
                last_yaw = current_yaw

                feedback.turned = total_turned
                goal_handle.publish_feedback(feedback)

                if goal_handle.is_cancel_requested:
                    self.cmd_vel.publish(Twist())
                    goal_handle.canceled()
                    self.get_logger().info(f'Canceled after {total_turned:.2f} rad')
                    result.turned = total_turned
                    return result

                # Check if rotation goal reached
                if target_angle > 0.0 and total_turned >= target_angle:
                    break
                if target_angle < 0.0 and total_turned <= target_angle:
                    break

                self.cmd_vel.publish(twist)
                time.sleep(0.02)

            self.cmd_vel.publish(Twist())
            goal_handle.succeed()
            result.turned = total_turned
            self.get_logger().info(f'Rotation finished, turned {result.turned:.2f} rad')
            return result
        finally:
            self.cmd_vel.publish(Twist())


def main():
    rclpy.init()
    node = RotateServer()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
