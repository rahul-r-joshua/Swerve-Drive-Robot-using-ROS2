#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Path, Odometry
from tf_transformations import euler_from_quaternion
import signal
import threading
import time

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower_swerve')

        self.path_sub = self.create_subscription(Path, '/current_path', self.path_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/rocker_controller/odom', self.odom_callback, 10)

        self.cmd_pub = self.create_publisher(TwistStamped, '/rocker_controller/cmd_vel', 10)
        self.trace_pub = self.create_publisher(Path, '/robot_trace_path', 10)

        self.create_timer(0.05, self.control_loop)

        self.path_poses = []
        self.current_target_index = 0
        self.reached_goal = False

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.trace_path = Path()
        self.trace_path.header.frame_id = 'odom'

        self.max_linear_speed = 1.0   
        self.max_angular_speed = 2.0  
        self.goal_tolerance = 0.1
        self.last_logged_index = -1

    def path_callback(self, msg: Path):
        if len(msg.poses) == len(self.path_poses):
            return
        self.path_poses = msg.poses
        self.current_target_index = 0
        self.reached_goal = False
        self.get_logger().info(f'Received path with {len(self.path_poses)} waypoints.')

    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        _, _, self.current_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'odom'
        pose.pose = msg.pose.pose
        self.trace_path.poses.append(pose)
        self.trace_path.header.stamp = pose.header.stamp
        self.trace_pub.publish(self.trace_path)

    def control_loop(self):
        if self.reached_goal or not self.path_poses:
            return

        target = self.path_poses[self.current_target_index]
        tx = target.pose.position.x
        ty = target.pose.position.y

        dx = tx - self.current_x
        dy = ty - self.current_y
        distance = math.hypot(dx, dy)

        if distance < self.goal_tolerance:
            self.get_logger().info(f'Reached waypoint {self.current_target_index}')
            self.current_target_index += 1
            if self.current_target_index >= len(self.path_poses):
                self.get_logger().info("All waypoints completed!")
                self.reached_goal = True
                self.stop_robot()
            return

        if self.current_target_index != self.last_logged_index:
            self.get_logger().info(f'Moving towards waypoint {self.current_target_index}')
            self.last_logged_index = self.current_target_index

        angle_to_target = math.atan2(dy, dx)
        yaw_error = angle_to_target - self.current_yaw
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        body_dx = math.cos(-self.current_yaw) * dx - math.sin(-self.current_yaw) * dy
        body_dy = math.sin(-self.current_yaw) * dx + math.cos(-self.current_yaw) * dy

        direction_magnitude = math.hypot(body_dx, body_dy)
        if direction_magnitude > 0.01:
            norm_dx = body_dx / direction_magnitude
            norm_dy = body_dy / direction_magnitude
        else:
            norm_dx = 0.0
            norm_dy = 0.0

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = self.max_linear_speed * norm_dx
        cmd.twist.linear.y = self.max_linear_speed * norm_dy
        cmd.twist.angular.z = max(min(2.0 * yaw_error, self.max_angular_speed), -self.max_angular_speed)

        self.cmd_pub.publish(cmd)

    def stop_robot(self):
        stop_cmd = TwistStamped()
        stop_cmd.header.stamp = self.get_clock().now().to_msg()
        stop_cmd.twist.linear.x = 0.0
        stop_cmd.twist.linear.y = 0.0
        stop_cmd.twist.angular.z = 0.0
        self.cmd_pub.publish(stop_cmd)
        self.get_logger().info("Published stop command.")

def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    shutdown_requested = threading.Event()

    def handle_sigint(signum, frame):
        node.get_logger().info("Ctrl+C received! Sending stop command...")
        node.stop_robot()
        time.sleep(0.3)
        shutdown_requested.set()

    signal.signal(signal.SIGINT, handle_sigint)

    try:
        while rclpy.ok() and not shutdown_requested.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.get_logger().info("Shutting down node cleanly.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
