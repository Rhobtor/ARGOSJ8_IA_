// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from argj801_ctl_platform_interfaces:msg/CmdThrottleMsg.idl
// generated code does not contain a copyright notice

#ifndef ARGJ801_CTL_PLATFORM_INTERFACES__MSG__DETAIL__CMD_THROTTLE_MSG__TRAITS_HPP_
#define ARGJ801_CTL_PLATFORM_INTERFACES__MSG__DETAIL__CMD_THROTTLE_MSG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "argj801_ctl_platform_interfaces/msg/detail/cmd_throttle_msg__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace argj801_ctl_platform_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const CmdThrottleMsg & msg,
  std::ostream & out)
{
  out << "{";
  // member: throttle
  {
    out << "throttle: ";
    rosidl_generator_traits::value_to_yaml(msg.throttle, out);
    out << ", ";
  }

  // member: steering
  {
    out << "steering: ";
    rosidl_generator_traits::value_to_yaml(msg.steering, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CmdThrottleMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: throttle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "throttle: ";
    rosidl_generator_traits::value_to_yaml(msg.throttle, out);
    out << "\n";
  }

  // member: steering
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "steering: ";
    rosidl_generator_traits::value_to_yaml(msg.steering, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CmdThrottleMsg & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace argj801_ctl_platform_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use argj801_ctl_platform_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const argj801_ctl_platform_interfaces::msg::CmdThrottleMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  argj801_ctl_platform_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use argj801_ctl_platform_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const argj801_ctl_platform_interfaces::msg::CmdThrottleMsg & msg)
{
  return argj801_ctl_platform_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<argj801_ctl_platform_interfaces::msg::CmdThrottleMsg>()
{
  return "argj801_ctl_platform_interfaces::msg::CmdThrottleMsg";
}

template<>
inline const char * name<argj801_ctl_platform_interfaces::msg::CmdThrottleMsg>()
{
  return "argj801_ctl_platform_interfaces/msg/CmdThrottleMsg";
}

template<>
struct has_fixed_size<argj801_ctl_platform_interfaces::msg::CmdThrottleMsg>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<argj801_ctl_platform_interfaces::msg::CmdThrottleMsg>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<argj801_ctl_platform_interfaces::msg::CmdThrottleMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ARGJ801_CTL_PLATFORM_INTERFACES__MSG__DETAIL__CMD_THROTTLE_MSG__TRAITS_HPP_
