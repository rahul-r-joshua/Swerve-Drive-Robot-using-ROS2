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
import tf_transformations


class SwerveControllerAndOdometry(Node):
    def __init__(self):
        super().__init__('swerve_controller_and_odometry')

        self.create_subscription(TwistStamped,'rocker_controller/cmd_vel',self.cmd_vel_st_cb, 10)
        self.create_subscription(JointState,'joint_states',self.joint_state_cb,50)

        self.steering_pub = self.create_publisher(Float64MultiArray,'swerve_steering_controller/commands',10)
        self.velocity_pub = self.create_publisher(Float64MultiArray,'simple_velocity_controller/commands',10)
        self.rocker_pub   = self.create_publisher(Float64MultiArray,'rocker_controller/commands',10)
        self.odom_pub     = self.create_publisher(Odometry,'rocker_controller/odom',10)
        self.twist_pub    = self.create_publisher(Twist,'measured_twist',10)
        self.twist_pub_st = self.create_publisher(TwistStamped,'measured_twist_stamped',10)

        self.tf_br = TransformBroadcaster(self)

        self.wheel_base   = 0.40
        self.track_width  = 0.22
        self.wheel_radius = 0.05

        self.turnbuckle_offsets = {
            'turnbuckle_left':  ( 0.15,  0.00, 0.05), 
            'turnbuckle_right': (-0.15,  0.00, 0.05)
        }
        self.diff_link_offset = (0.072, 0.0, 0.066)

        self.steering_names = ['steer_fl_joint', 'steer_fr_joint', 'steer_rl_joint', 'steer_rr_joint']  
        self.wheel_names    = ['fl_wheel_joint', 'fr_wheel_joint', 'rl_wheel_joint', 'rr_wheel_joint']  
        self.x = self.y = self.yaw = 0.0
        self.last_time = self.get_clock().now()

        self.pos, self.vel = {}, {}

    @staticmethod
    def _wrap_pi(a):
        return math.atan2(math.sin(a), math.cos(a))

    def _normalize(self, ang):
        ang = self._wrap_pi(ang)
        if abs(ang) > math.pi/2:
            return ang - math.copysign(math.pi, ang), True
        return ang, False

    def _fk(self, vx, vy, wz):
        wb2, tw2 = self.wheel_base/2.0, self.track_width/2.0
        vec = {'fl': (vx - wz*tw2, vy + wz*wb2),
               'fr': (vx + wz*tw2, vy + wz*wb2),
               'rl': (vx - wz*tw2, vy - wz*wb2),
               'rr': (vx + wz*tw2, vy - wz*wb2)}
        angs, spds = [], []
        for k in ['fl','fr','rl','rr']:
            vx_i, vy_i = vec[k]
            s  = math.hypot(vx_i, vy_i)
            a  = math.atan2(vy_i, vx_i)
            a, rev = self._normalize(a)
            spds.append(-s if rev else s)
            angs.append(a)
        return angs, spds

    def _send_cmd(self, vx, vy, wz):
        angs, spds = self._fk(vx, vy, wz)
        self.steering_pub.publish(Float64MultiArray(data=angs))
        self.velocity_pub.publish(Float64MultiArray(data=[spds[1], spds[0], spds[2], spds[3]]))
        self.rocker_pub.publish(Float64MultiArray(data=[0.0, 0.0]))


    def cmd_vel_st_cb(self, msg: TwistStamped):
        self._send_cmd(msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z)

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
        w   = [self.vel[n] for n in self.wheel_names]

        B = [val for a, w_i in zip(phi, w) for val in (w_i*self.wheel_radius*math.cos(a),
                                                       w_i*self.wheel_radius*math.sin(a))]
        B = np.asarray(B)

        rx, ry = self.wheel_base/2.0, self.track_width/2.0
        A = np.array([
            [1, 0, -ry], [0, 1,  rx],
            [1, 0,  ry], [0, 1,  rx],
            [1, 0, -ry], [0, 1, -rx],
            [1, 0,  ry], [0, 1, -rx]
        ])
        vx, vy, wz = np.linalg.lstsq(A, B, rcond=None)[0]

        now_msg = self.get_clock().now().to_msg()
        dt  = (rclpy.time.Time.from_msg(now_msg) - self.last_time).nanoseconds * 1e-9
        if dt <= 0:
            return
        self.last_time = rclpy.time.Time.from_msg(now_msg)

        self.x   += (vx*math.cos(self.yaw) - vy*math.sin(self.yaw)) * dt
        self.y   += (vx*math.sin(self.yaw) + vy*math.cos(self.yaw)) * dt
        self.yaw  = self._wrap_pi(self.yaw + wz*dt)

        qx, qy, qz, qw = tf_transformations.quaternion_from_euler(0, 0, self.yaw)
        quat = Quaternion(x=qx, y=qy, z=qz, w=qw)

        od = Odometry()
        od.header.stamp = now_msg
        od.header.frame_id = 'odom'
        od.child_frame_id  = 'base_footprint'
        od.pose.pose.position.x = self.x
        od.pose.pose.position.y = self.y
        od.pose.pose.orientation = quat
        od.twist.twist.linear.x  = vx
        od.twist.twist.linear.y  = vy
        od.twist.twist.angular.z = wz
        self.odom_pub.publish(od)

        tw = Twist()
        tw.linear.x, tw.linear.y, tw.angular.z = vx, vy, wz
        self.twist_pub.publish(tw)
        self.twist_pub_st.publish(TwistStamped(header=od.header, twist=tw))

        t = TransformStamped()
        t.header.stamp    = now_msg
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = quat
        self.tf_br.sendTransform(t)

        for name, (x, y, z) in self.turnbuckle_offsets.items():
            tf = TransformStamped()
            tf.header.stamp = now_msg
            tf.header.frame_id = 'base_link'
            tf.child_frame_id = name
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.translation.z = z
            tf.transform.rotation.w = 1.0  
            self.tf_br.sendTransform(tf)

        tf = TransformStamped()
        tf.header.stamp = now_msg
        tf.header.frame_id = 'base_link'
        tf.child_frame_id = 'diff_link'
        tf.transform.translation.x = self.diff_link_offset[0]
        tf.transform.translation.y = self.diff_link_offset[1]
        tf.transform.translation.z = self.diff_link_offset[2]
        tf.transform.rotation.w = 1.0
        self.tf_br.sendTransform(tf)

def main(args=None):
    rclpy.init(args=args)
    node = SwerveControllerAndOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
