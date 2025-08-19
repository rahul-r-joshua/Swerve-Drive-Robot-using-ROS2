#ifndef ROCKER_CONTROLLER__SIMPLE_SWERVE_CONTROLLER_HPP_
#define ROCKER_CONTROLLER__SIMPLE_SWERVE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"

namespace simple_swerve_controller
{

class SimpleSwerveController : public nav2_core::Controller
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void activate() override;
  void deactivate() override;
  void cleanup() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;


private:
  void computeFK(double vx, double vy, double wz, std::vector<double>& angles, std::vector<double>& speeds);
  std::pair<double, bool> normalize(double ang);
  double wrapPi(double a);
  geometry_msgs::msg::PoseStamped getNextPose(const geometry_msgs::msg::PoseStamped & robot_pose);
  bool transformPlan(const std::string & frame);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_{rclcpp::get_logger("SimpleNav2Controller")};
  rclcpp::Clock::SharedPtr clock_;


  nav_msgs::msg::Path global_plan_;
  std::string plugin_name_;

  double wheel_base_, track_width_, wheel_radius_;
  double kp_, kd_, max_linear_vel_, max_angular_vel_, step_size_;

  rclcpp::Time last_cycle_time_;
  double prev_linear_error_ = 0.0, prev_angular_error_ = 0.0;

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  double x_ = 0.0, y_ = 0.0, yaw_ = 0.0;
};

}  // namespace rocker_controller

#endif  // ROCKER_CONTROLLER__SIMPLE_SWERVE_CONTROLLER_HPP_
