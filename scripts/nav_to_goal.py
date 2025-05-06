#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener
import tf_transformations
import math

class SendGoal(Node):
    def __init__(self):
        super().__init__('send_goal_node')

        # TF setup to query map → camera_link
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Nav2 action client
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Wait 1s between attempts, allow TF to populate
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        try:
            now = rclpy.time.Time()
            # Get current pose of robot in map frame
            transform = self.tf_buffer.lookup_transform(
                'map', 'camera_link', now
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y

            # Get yaw (theta) from quaternion
            q = transform.transform.rotation
            _, _, theta = tf_transformations.euler_from_quaternion(
                [q.x, q.y, q.z, q.w]
            )

            # Compute 1 meter ahead based on current heading
            goal_x = x + 1.0 * math.cos(theta)
            goal_y = y + 1.0 * math.sin(theta)
            goal_theta = theta  # maintain current orientation

            self.send_goal(goal_x, goal_y, goal_theta)
            self.get_logger().info(f"✅ Sent goal 1m ahead: x={goal_x:.2f}, y={goal_y:.2f}, θ={goal_theta:.2f}")
            
            # Only send goal once
            self.timer.cancel()

        except Exception as e:
            self.get_logger().warn(f"Waiting for TF: {str(e)}")

    def send_goal(self, x, y, theta):
        # Convert theta to quaternion
        q = tf_transformations.quaternion_from_euler(0, 0, theta)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]

        self.client.wait_for_server()
        self.client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SendGoal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
