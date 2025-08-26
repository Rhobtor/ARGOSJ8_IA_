// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from argj801_ctl_platform_interfaces:srv/EmergencyStop.idl
// generated code does not contain a copyright notice

#ifndef ARGJ801_CTL_PLATFORM_INTERFACES__SRV__DETAIL__EMERGENCY_STOP__BUILDER_HPP_
#define ARGJ801_CTL_PLATFORM_INTERFACES__SRV__DETAIL__EMERGENCY_STOP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "argj801_ctl_platform_interfaces/srv/detail/emergency_stop__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace argj801_ctl_platform_interfaces
{

namespace srv
{

namespace builder
{

class Init_EmergencyStop_Request_stop
{
public:
  Init_EmergencyStop_Request_stop()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::argj801_ctl_platform_interfaces::srv::EmergencyStop_Request stop(::argj801_ctl_platform_interfaces::srv::EmergencyStop_Request::_stop_type arg)
  {
    msg_.stop = std::move(arg);
    return std::move(msg_);
  }

private:
  ::argj801_ctl_platform_interfaces::srv::EmergencyStop_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::argj801_ctl_platform_interfaces::srv::EmergencyStop_Request>()
{
  return argj801_ctl_platform_interfaces::srv::builder::Init_EmergencyStop_Request_stop();
}

}  // namespace argj801_ctl_platform_interfaces


namespace argj801_ctl_platform_interfaces
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::argj801_ctl_platform_interfaces::srv::EmergencyStop_Response>()
{
  return ::argj801_ctl_platform_interfaces::srv::EmergencyStop_Response(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace argj801_ctl_platform_interfaces

#endif  // ARGJ801_CTL_PLATFORM_INTERFACES__SRV__DETAIL__EMERGENCY_STOP__BUILDER_HPP_
