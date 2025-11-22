#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data

class PathToNav2Follower(Node):
    def __init__(self):
        super().__init__('path_to_nav2_follower')

        # Subscribe to the custom A* path
        self.subscription = self.create_subscription(
            Path,
            '/planned_path',
            self.path_callback,
            qos_profile_sensor_data
        )

        # Action client to send path to Nav2's FollowPath behavior
        self._action_client = ActionClient(self, FollowPath, '/follow_path')

        self.get_logger().info("Path to Nav2 Follower node started. Waiting for path...")

    def path_callback(self, path_msg):
        self.get_logger().info("Received path from A*, sending to Nav2...")

        goal_msg = FollowPath.Goal()
        goal_msg.path = path_msg
        goal_msg.controller_id = ''       # Leave default or set to your controller ID
        goal_msg.goal_checker_id = ''     # Optional

        self._action_client.wait_for_server()
        self._send_goal(goal_msg)

    def _send_goal(self, goal_msg):
        self._future = self._action_client.send_goal_async(goal_msg)
        self._future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('FollowPath goal was rejected by Nav2')
            return

        self.get_logger().info('FollowPath goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation completed with status: {result.result}')

def main(args=None):
    rclpy.init(args=args)
    node = PathToNav2Follower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
