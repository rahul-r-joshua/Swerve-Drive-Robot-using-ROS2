#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.constants import S_TO_NS
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf_transformations import quaternion_from_euler


class CirclePathPublisher(Node):
    def __init__(self):
        super().__init__('circle_path_publisher')

        self.path_pub = self.create_publisher(Path, '/current_path', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/current_pose', 10)

        self.total_time = 32.0    
        self.radius = 3.0          
        self.angular_speed = 0.2    
        self.finished = False

        self.t_start: Time = self.get_clock().now()
        self.t_prev: Time = self.t_start

        self.path = Path()
        self.path.header.frame_id = 'odom'

        self.create_timer(0.05, self.timer_cb)

    def timer_cb(self):
        now: Time = self.get_clock().now()
        elapsed_sec = (now - self.t_start).nanoseconds / S_TO_NS

        if elapsed_sec > self.total_time:
            if not self.finished:
                self.get_logger().info('Finished circular path.')
            self.finished = True
            return

        theta = elapsed_sec * self.angular_speed
        x = self.radius * math.cos(theta)
        y = self.radius * math.sin(theta)
        yaw = theta + math.pi / 2  

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

        self.path.header.stamp = pose.header.stamp
        self.path.poses.append(pose)
        self.path_pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = CirclePathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
