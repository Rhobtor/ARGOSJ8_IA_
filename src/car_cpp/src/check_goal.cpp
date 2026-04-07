#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <cmath>

// check_goal: compara posición del robot (via TF, frame 'map') con el goal activo.
// MOTIVO del TF: /fixposition/odometry da posición en frame 'odom' (ENU/ECEF)
// que NO coincide con 'map'. Los goals siempre se publican en 'map'.
// Usando TF obtenemos la misma posición que usa el MPPI.

class GoalReachedNode : public rclcpp::Node
{
public:
  GoalReachedNode()
  : Node("check_goal"), goal_received_(false)
  {
    this->declare_parameter("goal_threshold_xy", 3.5);  // robot converge a ~3.3m con local-waypoint activo; 3.5m da margen
    this->declare_parameter("robot_frame", std::string("base_link"));
    this->declare_parameter("goal_frame",  std::string("map"));

    tol_xy_      = this->get_parameter("goal_threshold_xy").as_double();
    robot_frame_ = this->get_parameter("robot_frame").as_string();
    goal_frame_  = this->get_parameter("goal_frame").as_string();

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "goal", 10,
      std::bind(&GoalReachedNode::goalCallback, this, std::placeholders::_1));

    goal_reached_pub_ = this->create_publisher<std_msgs::msg::Bool>("goal_reached", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&GoalReachedNode::controlLoop, this));

    RCLCPP_INFO(get_logger(),
      "check_goal: robot_frame='%s' goal_frame='%s' threshold_xy=%.2f m",
      robot_frame_.c_str(), goal_frame_.c_str(), tol_xy_);
  }

private:
  void goalCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    if (msg->poses.empty()) return;
    goal_pose_   = msg->poses[0];
    goal_frame_  = msg->header.frame_id.empty() ? goal_frame_ : msg->header.frame_id;
    goal_received_ = true;
    RCLCPP_INFO(get_logger(), "Nuevo goal recibido: [%.2f, %.2f] frame=%s",
                goal_pose_.position.x, goal_pose_.position.y, goal_frame_.c_str());
  }

  void controlLoop()
  {
    if (!goal_received_) return;

    // Obtener posición del robot en el frame del goal via TF
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

    const double robot_x = tf_robot.transform.translation.x;
    const double robot_y = tf_robot.transform.translation.y;
    const double goal_x  = goal_pose_.position.x;
    const double goal_y  = goal_pose_.position.y;

    const double dist_xy = std::hypot(goal_x - robot_x, goal_y - robot_y);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      "Robot: [%.2f, %.2f] Goal: [%.2f, %.2f] dist_xy: %.2f",
      robot_x, robot_y, goal_x, goal_y, dist_xy);

    if (dist_xy < tol_xy_) {
      std_msgs::msg::Bool msg;
      msg.data = true;
      goal_reached_pub_->publish(msg);
      RCLCPP_INFO(get_logger(), "Goal alcanzado. dist_xy=%.2f m", dist_xy);
      goal_received_ = false;
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr goal_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr goal_reached_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  geometry_msgs::msg::Pose goal_pose_;
  bool   goal_received_;
  double tol_xy_;
  std::string robot_frame_;
  std::string goal_frame_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoalReachedNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
