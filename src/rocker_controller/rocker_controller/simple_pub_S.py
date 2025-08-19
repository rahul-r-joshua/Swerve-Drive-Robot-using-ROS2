#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.constants import S_TO_NS
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf_transformations import quaternion_from_euler


class SmoothSPathPublisher(Node):
    def __init__(self):
        super().__init__('smooth_s_path_publisher')

        self.path_pub = self.create_publisher(Path, '/current_path', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/current_pose', 10)

        self.total_duration = 10.0    # seconds
        self.amplitude = 2.0          # S height
        self.path_length = 5.0        # total x-distance of the S

        self.path = Path()
        self.path.header.frame_id = 'odom'

        self.start_time = self.get_clock().now()
        self.finished = False

        self.create_timer(0.05, self.timer_cb)

    def timer_cb(self):
        now = self.get_clock().now()
        elapsed_sec = (now - self.start_time).nanoseconds / S_TO_NS

        if elapsed_sec > self.total_duration:
            if not self.finished:
                self.get_logger().info("Completed big smooth S path.")
                self.finished = True
            return

        # Normalized time (0 to 1)
        t = elapsed_sec / self.total_duration

        # S-shaped path using sin^3
        x = self.path_length * t
        y = self.amplitude * (math.sin(math.pi * t) ** 3)  # Smooth S-shape

        # Derivatives for yaw (dy/dt)
        dx = self.path_length / self.total_duration
        dy = self.amplitude * 3 * (math.sin(math.pi * t) ** 2) * math.cos(math.pi * t) * math.pi / self.total_duration

        yaw = math.atan2(dy, dx)

        pose = PoseStamped()
        pose.header.stamp = now.to_msg()
        pose.header.frame_id = 'odom'
        pose.pose.position.x = x
        pose.pose.position.y = y

        qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        self.pose_pub.publish(pose)
        self.path.poses.append(pose)
        self.path.header.stamp = pose.header.stamp
        self.path_pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = SmoothSPathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
