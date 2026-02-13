#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TwistStamped, Quaternion, TransformStamped
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf_transformations


class SwerveControllerAndOdometry(Node):

    def __init__(self):
        super().__init__('swerve_controller_and_odometry')

        # Subscriptions
        self.create_subscription(TwistStamped, 'rocker_controller/cmd_vel', self.cmd_vel_st_cb, 10)
        self.create_subscription(JointState, 'joint_states', self.joint_state_cb, 50)

        # Publishers
        self.steering_pub = self.create_publisher(Float64MultiArray, 'swerve_steering_controller/commands', 10)
        self.velocity_pub = self.create_publisher(Float64MultiArray, 'simple_velocity_controller/commands', 10)
        self.rocker_pub = self.create_publisher(Float64MultiArray, 'rocker_controller/commands', 10)
        self.odom_pub = self.create_publisher(Odometry, 'rocker_controller/odom', 10)
        self.twist_pub = self.create_publisher(Twist, 'measured_twist', 10)
        self.twist_pub_st = self.create_publisher(TwistStamped, 'measured_twist_stamped', 10)

        # TF broadcasters
        self.tf_br = TransformBroadcaster(self)
        self.static_br = StaticTransformBroadcaster(self)

        # Publish static transforms once
        self.publish_static_transforms()

        # Robot parameters
        self.wheel_base = 0.40
        self.track_width = 0.22
        self.wheel_radius = 0.05

        self.steering_names = ['steer_fl_joint', 'steer_fr_joint', 'steer_rl_joint', 'steer_rr_joint']
        self.wheel_names = ['fl_wheel_joint', 'fr_wheel_joint', 'rl_wheel_joint', 'rr_wheel_joint']

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = self.get_clock().now()

        self.pos = {}
        self.vel = {}

    # ==============================
    # Static TF
    # ==============================
    def publish_static_transforms(self):

        static_transforms = []

        # base_link → diff_link
        t1 = TransformStamped()
        t1.header.stamp = self.get_clock().now().to_msg()
        t1.header.frame_id = 'base_link'
        t1.child_frame_id = 'diff_link'
        t1.transform.translation.x = 0.0719999263371985
        t1.transform.translation.y = -1.02373561229607E-05
        t1.transform.translation.z = 0.0659981514243161
        t1.transform.rotation.w = 1.0
        static_transforms.append(t1)

        # diff_link → turnbuckle_left
        q_left = tf_transformations.quaternion_from_euler(1.5708, 0.0, 1.5708)

        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'diff_link'
        t2.child_frame_id = 'turnbuckle_left'
        t2.transform.translation.x = -0.04188
        t2.transform.translation.y = 0.111
        t2.transform.translation.z = 0.022998
        t2.transform.rotation.x = q_left[0]
        t2.transform.rotation.y = q_left[1]
        t2.transform.rotation.z = q_left[2]
        t2.transform.rotation.w = q_left[3]
        static_transforms.append(t2)

        # diff_link → turnbuckle_right
        q_right = tf_transformations.quaternion_from_euler(1.5708, 0.0, 1.5708)

        t3 = TransformStamped()
        t3.header.stamp = self.get_clock().now().to_msg()
        t3.header.frame_id = 'diff_link'
        t3.child_frame_id = 'turnbuckle_right'
        t3.transform.translation.x = -0.04188
        t3.transform.translation.y = -0.111
        t3.transform.translation.z = 0.022998
        t3.transform.rotation.x = q_right[0]
        t3.transform.rotation.y = q_right[1]
        t3.transform.rotation.z = q_right[2]
        t3.transform.rotation.w = q_right[3]
        static_transforms.append(t3)

        self.static_br.sendTransform(static_transforms)

    # ==============================
    # Utility functions
    # ==============================
    @staticmethod
    def _wrap_pi(a):
        return math.atan2(math.sin(a), math.cos(a))

    def _normalize(self, ang):
        ang = self._wrap_pi(ang)
        if abs(ang) > math.pi / 2:
            return ang - math.copysign(math.pi, ang), True
        return ang, False

    def _fk(self, vx, vy, wz):
        wb2 = self.wheel_base / 2.0
        tw2 = self.track_width / 2.0

        vec = {
            'fl': (vx - wz * tw2, vy + wz * wb2),
            'fr': (vx + wz * tw2, vy + wz * wb2),
            'rl': (vx - wz * tw2, vy - wz * wb2),
            'rr': (vx + wz * tw2, vy - wz * wb2)
        }

        angs, spds = [], []
        for k in ['fl', 'fr', 'rl', 'rr']:
            vx_i, vy_i = vec[k]
            s = math.hypot(vx_i, vy_i)
            a = math.atan2(vy_i, vx_i)
            a, rev = self._normalize(a)
            spds.append(-s if rev else s)
            angs.append(a)

        return angs, spds

    # ==============================
    # Callbacks
    # ==============================
    def cmd_vel_st_cb(self, msg: TwistStamped):
        angs, spds = self._fk(msg.twist.linear.x,
                              msg.twist.linear.y,
                              msg.twist.angular.z)

        self.steering_pub.publish(Float64MultiArray(data=angs))
        self.velocity_pub.publish(Float64MultiArray(data=[spds[1], spds[0], spds[2], spds[3]]))
        self.rocker_pub.publish(Float64MultiArray(data=[0.0, 0.0]))

    def joint_state_cb(self, msg: JointState):

        for i, n in enumerate(msg.name):
            self.pos[n] = msg.position[i]
            if i < len(msg.velocity):
                self.vel[n] = msg.velocity[i]

        if not all(n in self.pos for n in self.steering_names):
            return
        if not all(n in self.vel for n in self.wheel_names):
            return

        phi = [self.pos[n] for n in self.steering_names]
        w = [self.vel[n] for n in self.wheel_names]

        B = []
        for a, w_i in zip(phi, w):
            B.append(w_i * self.wheel_radius * math.cos(a))
            B.append(w_i * self.wheel_radius * math.sin(a))
        B = np.asarray(B)

        rx = self.wheel_base / 2.0
        ry = self.track_width / 2.0

        A = np.array([
            [1, 0, -ry], [0, 1, rx],
            [1, 0,  ry], [0, 1, rx],
            [1, 0, -ry], [0, 1, -rx],
            [1, 0,  ry], [0, 1, -rx]
        ])

        vx, vy, wz = np.linalg.lstsq(A, B, rcond=None)[0]

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        self.last_time = now
        now_msg = now.to_msg()

        self.x += (vx * math.cos(self.yaw) - vy * math.sin(self.yaw)) * dt
        self.y += (vx * math.sin(self.yaw) + vy * math.cos(self.yaw)) * dt
        self.yaw = self._wrap_pi(self.yaw + wz * dt)

        qx, qy, qz, qw = tf_transformations.quaternion_from_euler(0, 0, self.yaw)
        quat = Quaternion(x=qx, y=qy, z=qz, w=qw)

        od = Odometry()
        od.header.stamp = now_msg
        od.header.frame_id = 'odom'
        od.child_frame_id = 'base_footprint'
        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.orientation = quat
        od.twist.twist.linear.x = vx
        od.twist.twist.linear.y = vy
        od.twist.twist.angular.z = wz

        self.odom_pub.publish(od)

        t = TransformStamped()
        t.header.stamp = now_msg
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = quat

        self.tf_br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = SwerveControllerAndOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
s
