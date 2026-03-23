// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from argj801_ctl_platform_interfaces:msg/KinematicDebugMsg.idl
// generated code does not contain a copyright notice

#ifndef ARGJ801_CTL_PLATFORM_INTERFACES__MSG__DETAIL__KINEMATIC_DEBUG_MSG__BUILDER_HPP_
#define ARGJ801_CTL_PLATFORM_INTERFACES__MSG__DETAIL__KINEMATIC_DEBUG_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "argj801_ctl_platform_interfaces/msg/detail/kinematic_debug_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace argj801_ctl_platform_interfaces
{

namespace msg
{

namespace builder
{

class Init_KinematicDebugMsg_steering_limited
{
public:
  explicit Init_KinematicDebugMsg_steering_limited(::argj801_ctl_platform_interfaces::msg::KinematicDebugMsg & msg)
  : msg_(msg)
  {}
  ::argj801_ctl_platform_interfaces::msg::KinematicDebugMsg steering_limited(::argj801_ctl_platform_interfaces::msg::KinematicDebugMsg::_steering_limited_type arg)
  {
    msg_.steering_limited = std::move(arg);
    return std::move(msg_);
  }

private:
  ::argj801_ctl_platform_interfaces::msg::KinematicDebugMsg msg_;
};

class Init_KinematicDebugMsg_throttle_limited
{
public:
  explicit Init_KinematicDebugMsg_throttle_limited(::argj801_ctl_platform_interfaces::msg::KinematicDebugMsg & msg)
  : msg_(msg)
  {}
  Init_KinematicDebugMsg_steering_limited throttle_limited(::argj801_ctl_platform_interfaces::msg::KinematicDebugMsg::_throttle_limited_type arg)
  {
    msg_.throttle_limited = std::move(arg);
    return Init_KinematicDebugMsg_steering_limited(msg_);
  }

private:
  ::argj801_ctl_platform_interfaces::msg::KinematicDebugMsg msg_;
};

class Init_KinematicDebugMsg_requested_steering_acc
{
public:
  explicit Init_KinematicDebugMsg_requested_steering_acc(::argj801_ctl_platform_interfaces::msg::KinematicDebugMsg & msg)
  : msg_(msg)
  {}
  Init_KinematicDebugMsg_throttle_limited requested_steering_acc(::argj801_ctl_platform_interfaces::msg::KinematicDebugMsg::_requested_steering_acc_type arg)
  {
    msg_.requested_steering_acc = std::move(arg);
    return Init_KinematicDebugMsg_throttle_limited(msg_);
  }

private:
  ::argj801_ctl_platform_interfaces::msg::KinematicDebugMsg msg_;
};

class Init_KinematicDebugMsg_requested_throttle_acc
{
public:
  explicit Init_KinematicDebugMsg_requested_throttle_acc(::argj801_ctl_platform_interfaces::msg::KinematicDebugMsg & msg)
  : msg_(msg)
  {}
  Init_KinematicDebugMsg_requested_steering_acc requested_throttle_acc(::argj801_ctl_platform_interfaces::msg::KinematicDebugMsg::_requested_throttle_acc_type arg)
  {
    msg_.requested_throttle_acc = std::move(arg);
    return Init_KinematicDebugMsg_requested_steering_acc(msg_);
  }

private:
  ::argj801_ctl_platform_interfaces::msg::KinematicDebugMsg msg_;
};

class Init_KinematicDebugMsg_steering
{
public:
  explicit Init_KinematicDebugMsg_steering(::argj801_ctl_platform_interfaces::msg::KinematicDebugMsg & msg)
  : msg_(msg)
  {}
  Init_KinematicDebugMsg_requested_throttle_acc steering(::argj801_ctl_platform_interfaces::msg::KinematicDebugMsg::_steering_type arg)
  {
    msg_.steering = std::move(arg);
    return Init_KinematicDebugMsg_requested_throttle_acc(msg_);
  }

private:
  ::argj801_ctl_platform_interfaces::msg::KinematicDebugMsg msg_;
};

class Init_KinematicDebugMsg_throttle
{
public:
  Init_KinematicDebugMsg_throttle()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_KinematicDebugMsg_steering throttle(::argj801_ctl_platform_interfaces::msg::KinematicDebugMsg::_throttle_type arg)
  {
    msg_.throttle = std::move(arg);
    return Init_KinematicDebugMsg_steering(msg_);
  }

private:
  ::argj801_ctl_platform_interfaces::msg::KinematicDebugMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::argj801_ctl_platform_interfaces::msg::KinematicDebugMsg>()
{
  return argj801_ctl_platform_interfaces::msg::builder::Init_KinematicDebugMsg_throttle();
}

}  // namespace argj801_ctl_platform_interfaces

#endif  // ARGJ801_CTL_PLATFORM_INTERFACES__MSG__DETAIL__KINEMATIC_DEBUG_MSG__BUILDER_HPP_
