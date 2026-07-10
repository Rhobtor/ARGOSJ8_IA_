#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <cmath>
#include <string>
#include <vector>

// check_goal: compara posición del robot (via TF, frame 'map') con el goal activo.
// MOTIVO del TF: /fixposition/odometry da posición en frame 'odom' (ENU/ECEF)
// que NO coincide con 'map'. Los goals siempre se publican en 'map'.
// Usando TF obtenemos la misma posición que usa el MPPI.

class GoalReachedNode : public rclcpp::Node
{
public:
  GoalReachedNode()
  : Node("check_goal"), goal_received_(false), current_goal_index_(0)
  {
    this->declare_parameter("goal_threshold_xy", 3.5);  // robot converge a ~3.3m con local-waypoint activo; 3.5m da margen
    this->declare_parameter("goal_topic", std::string("/goal_local"));
    this->declare_parameter("goal_reached_topic", std::string("goal_reached"));
    this->declare_parameter("robot_frame", std::string("base_link"));
    this->declare_parameter("goal_frame",  std::string("map"));
    this->declare_parameter("goal_is_relative", false);

    tol_xy_      = this->get_parameter("goal_threshold_xy").as_double();
    goal_topic_  = this->get_parameter("goal_topic").as_string();
    goal_reached_topic_ = this->get_parameter("goal_reached_topic").as_string();
    robot_frame_ = this->get_parameter("robot_frame").as_string();
    goal_frame_  = this->get_parameter("goal_frame").as_string();
    goal_is_relative_ = this->get_parameter("goal_is_relative").as_bool();

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      goal_topic_, 10,
      std::bind(&GoalReachedNode::goalCallback, this, std::placeholders::_1));

    goal_reached_pub_ = this->create_publisher<std_msgs::msg::Bool>(goal_reached_topic_, 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&GoalReachedNode::controlLoop, this));

    RCLCPP_INFO(get_logger(),
      "check_goal: topic='%s' robot_frame='%s' goal_frame='%s' relative=%s threshold_xy=%.2f m",
      goal_topic_.c_str(), robot_frame_.c_str(), goal_frame_.c_str(), goal_is_relative_ ? "true" : "false", tol_xy_);
  }

private:
  static bool samePose(const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b)
  {
    constexpr double eps = 1e-6;
    return std::fabs(a.position.x - b.position.x) < eps &&
           std::fabs(a.position.y - b.position.y) < eps &&
           std::fabs(a.position.z - b.position.z) < eps;
  }

  bool sameGoals(const std::vector<geometry_msgs::msg::Pose> & poses) const
  {
    if (poses.size() != goal_poses_.size()) {
      return false;
    }

    for (size_t i = 0; i < poses.size(); ++i) {
      if (!samePose(poses[i], goal_poses_[i])) {
        return false;
      }
    }
    return true;
  }

  void goalCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    if (msg->poses.empty()) return;

    if (!sameGoals(msg->poses)) {
      goal_poses_ = msg->poses;
      current_goal_index_ = 0;
    }

    goal_frame_  = msg->header.frame_id.empty() ? goal_frame_ : msg->header.frame_id;
    goal_received_ = true;
    const auto & active_goal = goal_poses_.at(current_goal_index_);
    RCLCPP_INFO(get_logger(), "Nuevo goal activo %zu/%zu: [%.2f, %.2f] frame=%s",
                current_goal_index_ + 1, goal_poses_.size(), active_goal.position.x, active_goal.position.y, goal_frame_.c_str());
  }

  void controlLoop()
  {
    if (!goal_received_ || goal_poses_.empty()) return;

    const auto & goal_pose = goal_poses_.at(current_goal_index_);

    double dist_xy = 0.0;
    double robot_x = 0.0;
    double robot_y = 0.0;
    double goal_x = goal_pose.position.x;
    double goal_y = goal_pose.position.y;

    if (goal_is_relative_) {
      dist_xy = std::hypot(goal_x, goal_y);
    } else {
      geometry_msgs::msg::TransformStamped tf_robot;
      try {
        tf_robot = tf_buffer_->lookupTransform(
            goal_frame_, robot_frame_,
            tf2::TimePointZero,
            tf2::durationFromSec(0.1));
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "TF lookup %s→%s falló: %s", robot_frame_.c_str(), goal_frame_.c_str(), ex.what());
        return;
      }

      robot_x = tf_robot.transform.translation.x;
      robot_y = tf_robot.transform.translation.y;
      dist_xy = std::hypot(goal_x - robot_x, goal_y - robot_y);
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      "Robot: [%.2f, %.2f] Goal: [%.2f, %.2f] dist_xy: %.2f idx=%zu/%zu",
      robot_x, robot_y, goal_x, goal_y, dist_xy, current_goal_index_ + 1, goal_poses_.size());

    if (dist_xy < tol_xy_) {
      if (current_goal_index_ + 1 < goal_poses_.size()) {
        ++current_goal_index_;
        const auto & next_goal = goal_poses_.at(current_goal_index_);
        RCLCPP_INFO(get_logger(), "Waypoint alcanzado. Siguiente %zu/%zu: [%.2f, %.2f]",
                    current_goal_index_ + 1, goal_poses_.size(), next_goal.position.x, next_goal.position.y);
        return;
      }

      std_msgs::msg::Bool msg;
      msg.data = true;
      goal_reached_pub_->publish(msg);
      RCLCPP_INFO(get_logger(), "Goal alcanzado. dist_xy=%.2f m", dist_xy);
      goal_received_ = false;
      goal_poses_.clear();
      current_goal_index_ = 0;
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr goal_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::vector<geometry_msgs::msg::Pose> goal_poses_;
  bool   goal_received_;
  size_t current_goal_index_;
  double tol_xy_;
  std::string goal_topic_;
  std::string goal_reached_topic_;
  std::string robot_frame_;
  std::string goal_frame_;
  bool goal_is_relative_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoalReachedNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
