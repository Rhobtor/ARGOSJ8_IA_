// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from argj801_ctl_platform_interfaces:msg/CmdThrottleMsg.idl
// generated code does not contain a copyright notice

#ifndef ARGJ801_CTL_PLATFORM_INTERFACES__MSG__DETAIL__CMD_THROTTLE_MSG__BUILDER_HPP_
#define ARGJ801_CTL_PLATFORM_INTERFACES__MSG__DETAIL__CMD_THROTTLE_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "argj801_ctl_platform_interfaces/msg/detail/cmd_throttle_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace argj801_ctl_platform_interfaces
{

namespace msg
{

namespace builder
{

class Init_CmdThrottleMsg_steering
{
public:
  explicit Init_CmdThrottleMsg_steering(::argj801_ctl_platform_interfaces::msg::CmdThrottleMsg & msg)
  : msg_(msg)
  {}
  ::argj801_ctl_platform_interfaces::msg::CmdThrottleMsg steering(::argj801_ctl_platform_interfaces::msg::CmdThrottleMsg::_steering_type arg)
  {
    msg_.steering = std::move(arg);
    return std::move(msg_);
  }

private:
  ::argj801_ctl_platform_interfaces::msg::CmdThrottleMsg msg_;
};

class Init_CmdThrottleMsg_throttle
{
public:
  Init_CmdThrottleMsg_throttle()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CmdThrottleMsg_steering throttle(::argj801_ctl_platform_interfaces::msg::CmdThrottleMsg::_throttle_type arg)
  {
    msg_.throttle = std::move(arg);
    return Init_CmdThrottleMsg_steering(msg_);
  }

private:
  ::argj801_ctl_platform_interfaces::msg::CmdThrottleMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::argj801_ctl_platform_interfaces::msg::CmdThrottleMsg>()
{
  return argj801_ctl_platform_interfaces::msg::builder::Init_CmdThrottleMsg_throttle();
}

}  // namespace argj801_ctl_platform_interfaces

#endif  // ARGJ801_CTL_PLATFORM_INTERFACES__MSG__DETAIL__CMD_THROTTLE_MSG__BUILDER_HPP_
