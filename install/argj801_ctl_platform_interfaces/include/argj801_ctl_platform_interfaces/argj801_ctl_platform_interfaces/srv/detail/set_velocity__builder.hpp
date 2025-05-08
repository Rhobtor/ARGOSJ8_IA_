// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from argj801_ctl_platform_interfaces:srv/SetVelocity.idl
// generated code does not contain a copyright notice

#ifndef ARGJ801_CTL_PLATFORM_INTERFACES__SRV__DETAIL__SET_VELOCITY__BUILDER_HPP_
#define ARGJ801_CTL_PLATFORM_INTERFACES__SRV__DETAIL__SET_VELOCITY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "argj801_ctl_platform_interfaces/srv/detail/set_velocity__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace argj801_ctl_platform_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetVelocity_Request_new_velocity
{
public:
  Init_SetVelocity_Request_new_velocity()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::argj801_ctl_platform_interfaces::srv::SetVelocity_Request new_velocity(::argj801_ctl_platform_interfaces::srv::SetVelocity_Request::_new_velocity_type arg)
  {
    msg_.new_velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::argj801_ctl_platform_interfaces::srv::SetVelocity_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::argj801_ctl_platform_interfaces::srv::SetVelocity_Request>()
{
  return argj801_ctl_platform_interfaces::srv::builder::Init_SetVelocity_Request_new_velocity();
}

}  // namespace argj801_ctl_platform_interfaces


namespace argj801_ctl_platform_interfaces
{

namespace srv
{

namespace builder
{

class Init_SetVelocity_Response_actual_velocity
{
public:
  Init_SetVelocity_Response_actual_velocity()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::argj801_ctl_platform_interfaces::srv::SetVelocity_Response actual_velocity(::argj801_ctl_platform_interfaces::srv::SetVelocity_Response::_actual_velocity_type arg)
  {
    msg_.actual_velocity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::argj801_ctl_platform_interfaces::srv::SetVelocity_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::argj801_ctl_platform_interfaces::srv::SetVelocity_Response>()
{
  return argj801_ctl_platform_interfaces::srv::builder::Init_SetVelocity_Response_actual_velocity();
}

}  // namespace argj801_ctl_platform_interfaces

#endif  // ARGJ801_CTL_PLATFORM_INTERFACES__SRV__DETAIL__SET_VELOCITY__BUILDER_HPP_
