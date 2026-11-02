#!/usr/bin/env python3
"""
Action client that drives diffdrive_bot in a square.

Sequentially drives forward for a configurable side length and rotates 90 degrees
counterclockwise, repeating four times. Aborts and reports the failing side if any
action fails.
"""

import math

from action_msgs.msg import GoalStatus
from lab_interfaces.action import DriveDistance
from my_robot_interfaces.action import Rotate
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class SquareDriver(Node):

    def __init__(self):
        super().__init__('square_driver')
        self.declare_parameter('side_length', 0.5)

        self.drive_client = ActionClient(self, DriveDistance, 'drive_distance')
        self.rotate_client = ActionClient(self, Rotate, 'rotate')

    def wait_for_servers(self):
        self.get_logger().info('Waiting for /drive_distance action server...')
        while rclpy.ok() and not self.drive_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Still waiting for /drive_distance action server...')
        if not rclpy.ok():
            return False

        self.get_logger().info('Waiting for /rotate action server...')
        while rclpy.ok() and not self.rotate_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Still waiting for /rotate action server...')
        if not rclpy.ok():
            return False

        self.get_logger().info('Both action servers connected.')
        return True

    def drive(self, distance):
        goal = DriveDistance.Goal()
        goal.distance = float(distance)
        self.get_logger().info(f'Sending drive goal: {distance:.2f} m')
        send_future = self.drive_client.send_goal_async(
            goal, feedback_callback=self.on_drive_feedback
        )
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Drive distance goal was rejected by server')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result_response = result_future.result()
        if (
            result_response.status != GoalStatus.STATUS_SUCCEEDED
            or not result_response.result.reached
        ):
            self.get_logger().error(
                f'Drive distance goal did not succeed. Status: {result_response.status}, '
                f'reached: {result_response.result.reached}'
            )
            return False
        return True

    def rotate(self, angle):
        goal = Rotate.Goal()
        goal.angle = float(angle)
        self.get_logger().info(f'Sending rotate goal: {angle:.2f} rad')
        send_future = self.rotate_client.send_goal_async(
            goal, feedback_callback=self.on_rotate_feedback
        )
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Rotate goal was rejected by server')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result_response = result_future.result()
        if result_response.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f'Rotate goal did not succeed. Status: {result_response.status}'
            )
            return False
        return True

    def on_drive_feedback(self, feedback_msg):
        self.get_logger().info(
            f'Drive feedback: travelled {feedback_msg.feedback.travelled:.2f} m'
        )

    def on_rotate_feedback(self, feedback_msg):
        self.get_logger().info(
            f'Rotate feedback: turned {feedback_msg.feedback.turned:.2f} rad'
        )

    def run(self):
        if not self.wait_for_servers():
            return False

        side_length = float(self.get_parameter('side_length').value)
        quarter_turn = math.pi / 2.0  # 90 degrees counterclockwise

        self.get_logger().info(
            f'Starting square driving pattern with side_length={side_length:.2f} m'
        )

        for side in range(1, 5):
            self.get_logger().info(f'=== Side {side}/4: Driving forward ===')
            if not self.drive(side_length):
                self.get_logger().error(f'Failed on side {side} while driving forward')
                return False

            self.get_logger().info(f'=== Side {side}/4: Rotating counterclockwise ===')
            if not self.rotate(quarter_turn):
                self.get_logger().error(f'Failed on side {side} while rotating')
                return False

        self.get_logger().info('Square completed successfully!')
        return True


def main():
    rclpy.init()
    node = SquareDriver()
    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
