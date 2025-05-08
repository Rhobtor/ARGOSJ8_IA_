#include "ctl_mission/ReadyNode.hpp"

ReadyNode::ReadyNode(const std::string &node_name, bool intra_process_comms)
: rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)) {
  this->declare_parameter("cmd_vel_topic_name", "cmd_vel");

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ReadyNode::on_configure(const rclcpp_lifecycle::State &) {
    this->get_parameter("cmd_vel_topic_name", cmd_vel_topic);

  pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
  RCLCPP_INFO(this->get_logger(), "on_configure() is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ReadyNode::on_activate(const rclcpp_lifecycle::State & state) {
  LifecycleNode::on_activate(state);
  timer_ = this->create_wall_timer(0.02s, [this]() { this->refresh_cmd_vel(); });
  RCLCPP_INFO(this->get_logger(), "on_activate() is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ReadyNode::on_deactivate(const rclcpp_lifecycle::State & state) {
  LifecycleNode::on_deactivate(state);
  timer_.reset();
  RCLCPP_INFO(this->get_logger(), "on_deactivate() is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ReadyNode::on_cleanup(const rclcpp_lifecycle::State &) {
  timer_.reset();
  pub_cmd_vel_.reset();
  RCLCPP_INFO(this->get_logger(), "on_cleanup() is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ReadyNode::on_shutdown(const rclcpp_lifecycle::State & state) {
  timer_.reset();
  pub_cmd_vel_.reset();
  RCLCPP_INFO(this->get_logger(), "on_shutdown() is called from state %s.", state.label().c_str());
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void ReadyNode::refresh_cmd_vel() {
  cmd_msg_.linear.x = 0;
  cmd_msg_.angular.z = 0;
  pub_cmd_vel_->publish(cmd_msg_);
}
