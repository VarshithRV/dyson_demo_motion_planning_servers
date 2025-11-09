#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from ur16e_move_server.action import MoveToPose
import time

def main():
    rclpy.init()
    node = rclpy.create_node('move_goal_client')
    client = ActionClient(node, MoveToPose, 'move_to_pose')

    if not client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error("Server not available")
        return

    goal = MoveToPose.Goal()
    goal.target_pose = PoseStamped()
    goal.target_pose.header.frame_id = 'base_link'
    goal.target_pose.header.stamp = node.get_clock().now().to_msg()
    goal.target_pose.pose.position.x = 0.5
    goal.target_pose.pose.position.y = 0.5
    goal.target_pose.pose.position.z = 0.01
    goal.target_pose.pose.orientation.w = 1.0

    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future)
    
    time.sleep(10)

    goal_handle = future.result()
    if not goal_handle.accepted:
        node.get_logger().error('Goal rejected')
        return

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    
    result = result_future.result().result
    if result.result:
        node.get_logger().info("Success!")
    else:
        node.get_logger().error("Failed!")

if __name__ == '__main__':
    main()
