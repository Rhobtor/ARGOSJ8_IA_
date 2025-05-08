
#include "ctl_mission/TeleoperationNode.hpp"

TeleoperationNode::TeleoperationNode(const std::string &node_name, bool intra_process_comms)
: rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)) {
   // Declare parameters with namespace
    this->declare_parameter("max_joy_msg_delay", 1.0);
    this->declare_parameter("joystick_topic_name", "joy");
    this->declare_parameter("cmd_vel_topic_name", "cmd_vel_test");
    this->declare_parameter("queue_size", 10);
    this->declare_parameter("dead_man_button", 5);
    this->declare_parameter("x_axis_button", 1);
    this->declare_parameter("z_axis_button", 3);
    this->declare_parameter("x_multiplier", 1.0);
    this->declare_parameter("z_multiplier", 1.0);

    // Initialize variables
    last_joy_msg_timestamp_ = this->now();
    dead_man_ = false;
    x_axis_ = 0.0;
    z_axis_ = 0.0;  
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
TeleoperationNode::on_configure(const rclcpp_lifecycle::State &) {
  // Get parameters with namespace
  this->get_parameter("max_joy_msg_delay", max_joy_msg_delay_);
  std::string joy_topic, cmd_vel_topic;
  int queue_size;
  this->get_parameter("joystick_topic_name", joy_topic);
  this->get_parameter("cmd_vel_topic_name", cmd_vel_topic);
  this->get_parameter("queue_size", queue_size);
  this->get_parameter("dead_man_button", dead_man_button_);
  this->get_parameter("x_axis_button", x_axis_button_);
  this->get_parameter("z_axis_button", z_axis_button_);
  this->get_parameter("x_multiplier", x_multiplier);
  this->get_parameter("z_multiplier", z_multiplier);
  std::cout<<"Joy topic "<< joy_topic <<std::endl;
  std::cout<<"Vel topic "<< cmd_vel_topic <<std::endl;

  pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
  sub_joy_ = this->create_subscription<sensor_msgs::msg::Joy>(
    joy_topic, 10, std::bind(&TeleoperationNode::joy_callback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "on_configure() is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
TeleoperationNode::on_activate(const rclcpp_lifecycle::State & state) {
  LifecycleNode::on_activate(state);
  timer_ = this->create_wall_timer(0.02s, [this]() { this->refresh_cmd_vel(); });
  RCLCPP_INFO(this->get_logger(), "on_activate() is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
TeleoperationNode::on_deactivate(const rclcpp_lifecycle::State & state) {
  LifecycleNode::on_deactivate(state);
  timer_.reset();
  RCLCPP_INFO(this->get_logger(), "on_deactivate() is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
TeleoperationNode::on_cleanup(const rclcpp_lifecycle::State &) {
  timer_.reset();
  pub_cmd_vel_.reset();
  RCLCPP_INFO(this->get_logger(), "on_cleanup() is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
TeleoperationNode::on_shutdown(const rclcpp_lifecycle::State & state) {
  timer_.reset();
  pub_cmd_vel_.reset();
  RCLCPP_INFO(this->get_logger(), "on_shutdown() is called from state %s.", state.label().c_str());
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void TeleoperationNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
  dead_man_ = bool(msg->buttons[dead_man_button_]);
  x_axis_ = x_multiplier * msg->axes[x_axis_button_];
  z_axis_ = z_multiplier * msg->axes[z_axis_button_];
  last_joy_msg_timestamp_ = this->now();
}

void TeleoperationNode::refresh_cmd_vel() {
  rclcpp::Time current_time = this->now();
  double elapsed_time = (current_time - last_joy_msg_timestamp_).seconds();

  if (elapsed_time > max_joy_msg_delay_) {
    RCLCPP_WARN(this->get_logger(), "Joystick message is too old (%.2f seconds). Sending 0,0 commands.", elapsed_time);
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
    pub_cmd_vel_->publish(cmd_vel);
  } else {
    geometry_msgs::msg::Twist cmd_vel;
    if(dead_man_){
      cmd_vel.linear.x = x_axis_;
      cmd_vel.angular.z = z_axis_;
    }
    else{
      cmd_vel.linear.x = 0;
      cmd_vel.angular.z = 0;
    }
    pub_cmd_vel_->publish(cmd_vel);
  }
}
