/**
 * @file mppi_sac_relay_node.cpp
 * @brief Lifecycle state node that relays external AI controller commands to secured cmd_vel.
 *
 * ## Intent
 * The MPPI+SAC controller itself is developed outside this repository.
 * This node is only a *bridge* so the mission FSM can:
 *   - enter a dedicated "AI control" mode
 *   - forward Twist commands to the same channel used by the rest of the stack
 *     (`secured_cmd_vel` by default)
 *
 * ## ROS contract
 * ### Subscriptions
 * - `ai_cmd_topic_name` (default: `mppi_sac/cmd_vel`) [geometry_msgs/msg/Twist]
 *
 * ### Publications
 * - `secured_cmd_vel_topic_name` (default: `secured_cmd_vel`) [geometry_msgs/msg/Twist]
 *
 * ### Parameters
 * - `queue_size` (default: 10)
 *
 * ## Lifecycle behavior
 * - on_configure(): create subscription + lifecycle publisher.
 * - on_activate(): activate publisher so messages can flow.
 * - on_deactivate(): publisher deactivated (relay stops).
 */

#include "ctl_mission/MppiSacRelayNode.hpp"

MppiSacRelayNode::MppiSacRelayNode(const std::string & node_name, bool intra_process_comms)
: rclcpp_lifecycle::LifecycleNode(
    node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
{
  this->declare_parameter("ai_cmd_topic_name", "mppi_sac/cmd_vel");
  this->declare_parameter("secured_cmd_vel_topic_name", "secured_cmd_vel");
  this->declare_parameter("queue_size", 10);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MppiSacRelayNode::on_configure(const rclcpp_lifecycle::State &)
{
  this->get_parameter("ai_cmd_topic_name", ai_cmd_topic_);
  this->get_parameter("secured_cmd_vel_topic_name", secured_cmd_topic_);
  this->get_parameter("queue_size", queue_size_);

  pub_secured_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(secured_cmd_topic_, queue_size_);
  sub_ai_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
    ai_cmd_topic_, queue_size_, std::bind(&MppiSacRelayNode::on_ai_cmd, this, std::placeholders::_1));

  RCLCPP_INFO(
    this->get_logger(),
    "Configured MPPI/SAC relay. Subscribing to '%s', publishing to '%s'",
    ai_cmd_topic_.c_str(), secured_cmd_topic_.c_str());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MppiSacRelayNode::on_activate(const rclcpp_lifecycle::State & state)
{
  LifecycleNode::on_activate(state);
  if (pub_secured_cmd_vel_) {
    pub_secured_cmd_vel_->on_activate();
  }
  RCLCPP_INFO(this->get_logger(), "MPPI/SAC relay activated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MppiSacRelayNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  if (pub_secured_cmd_vel_) {
    pub_secured_cmd_vel_->on_deactivate();
  }
  LifecycleNode::on_deactivate(state);
  RCLCPP_INFO(this->get_logger(), "MPPI/SAC relay deactivated");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MppiSacRelayNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  sub_ai_cmd_.reset();
  pub_secured_cmd_vel_.reset();
  RCLCPP_INFO(this->get_logger(), "MPPI/SAC relay cleaned up");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MppiSacRelayNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  sub_ai_cmd_.reset();
  pub_secured_cmd_vel_.reset();
  RCLCPP_INFO(this->get_logger(), "MPPI/SAC relay shutdown from state %s", state.label().c_str());
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void MppiSacRelayNode::on_ai_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // Only publish if active.
  // LifecyclePublisher internally checks activation state when calling publish().
  if (pub_secured_cmd_vel_) {
    pub_secured_cmd_vel_->publish(*msg);
  }
}
