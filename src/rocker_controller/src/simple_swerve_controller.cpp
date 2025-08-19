#include "rocker_controller/simple_swerve_controller.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

namespace simple_swerve_controller
{

void SimpleSwerveController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  plugin_name_ = name;

  auto node = node_.lock();
  clock_ = node->get_clock();
  logger_ = node->get_logger();


  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".wheel_base", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".track_width", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".wheel_radius", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".kp", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".kd", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_linear_vel", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(1.0));

  node->get_parameter(plugin_name_ + ".wheel_base", wheel_base_);
  node->get_parameter(plugin_name_ + ".track_width", track_width_);
  node->get_parameter(plugin_name_ + ".wheel_radius", wheel_radius_);
  node->get_parameter(plugin_name_ + ".kp", kp_);
  node->get_parameter(plugin_name_ + ".kd", kd_);
  node->get_parameter(plugin_name_ + ".max_linear_vel", max_linear_vel_);
  node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);

  odom_pub_ = node->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
  last_cycle_time_ = clock_->now();
}

void SimpleSwerveController::cleanup()
{
  odom_pub_.reset();
  tf_broadcaster_.reset();
}

void SimpleSwerveController::activate()
{
  odom_pub_->on_activate();
}

void SimpleSwerveController::deactivate()
{
  odom_pub_->on_deactivate();
}

void SimpleSwerveController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
}

geometry_msgs::msg::TwistStamped SimpleSwerveController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & current_pose,
  const geometry_msgs::msg::Twist & /*velocity*/,
  nav2_core::GoalChecker * /*goal_checker*/)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.header.frame_id = "base_link";

  if (global_plan_.poses.empty()) {
    RCLCPP_WARN(logger_, "Global plan is empty");
    return cmd_vel;
  }

  const auto & goal = global_plan_.poses.back().pose;
  const auto & current = current_pose.pose;

  double dx = goal.position.x - current.position.x;
  double dy = goal.position.y - current.position.y;
  double dist = std::hypot(dx, dy);

  double vx = 0.0, vy = 0.0, wz = 0.0;
  if (dist > 0.05) {
    vx = kp_ * dx;
    vy = kp_ * dy;
  }

  cmd_vel.twist.linear.x = std::clamp(vx, -max_linear_vel_, max_linear_vel_);
  cmd_vel.twist.linear.y = std::clamp(vy, -max_linear_vel_, max_linear_vel_);
  cmd_vel.twist.angular.z = std::clamp(wz, -max_angular_vel_, max_angular_vel_);


  rclcpp::Time now = clock_->now();
  double dt = (now - last_cycle_time_).seconds();
  last_cycle_time_ = now;

  double delta_x = vx * std::cos(yaw_) * dt - vy * std::sin(yaw_) * dt;
  double delta_y = vx * std::sin(yaw_) * dt + vy * std::cos(yaw_) * dt;
  double delta_yaw = wz * dt;

  x_ += delta_x;
  y_ += delta_y;
  yaw_ += delta_yaw;

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw_);
  geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q);


  geometry_msgs::msg::TransformStamped odom_tf;
  odom_tf.header.stamp = now;
  odom_tf.header.frame_id = "odom";
  odom_tf.child_frame_id = "base_footprint";
  odom_tf.transform.translation.x = x_;
  odom_tf.transform.translation.y = y_;
  odom_tf.transform.translation.z = 0.0;
  odom_tf.transform.rotation = odom_quat;
  tf_broadcaster_->sendTransform(odom_tf);

  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = now;
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "base_footprint";
  odom_msg.pose.pose.position.x = x_;
  odom_msg.pose.pose.position.y = y_;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = odom_quat;
  odom_msg.twist.twist.linear.x = vx;
  odom_msg.twist.twist.linear.y = vy;
  odom_msg.twist.twist.angular.z = wz;

  odom_pub_->publish(odom_msg);

  return cmd_vel;
}

void SimpleSwerveController::setSpeedLimit(const double &, const bool &) {}

}  // namespace rocker_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(simple_swerve_controller::SimpleSwerveController, nav2_core::Controller)