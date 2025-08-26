// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from argj801_ctl_platform_interfaces:srv/EmergencyStop.idl
// generated code does not contain a copyright notice

#ifndef ARGJ801_CTL_PLATFORM_INTERFACES__SRV__DETAIL__EMERGENCY_STOP__TRAITS_HPP_
#define ARGJ801_CTL_PLATFORM_INTERFACES__SRV__DETAIL__EMERGENCY_STOP__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "argj801_ctl_platform_interfaces/srv/detail/emergency_stop__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace argj801_ctl_platform_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const EmergencyStop_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: stop
  {
    out << "stop: ";
    rosidl_generator_traits::value_to_yaml(msg.stop, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const EmergencyStop_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: stop
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stop: ";
    rosidl_generator_traits::value_to_yaml(msg.stop, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const EmergencyStop_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace argj801_ctl_platform_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use argj801_ctl_platform_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const argj801_ctl_platform_interfaces::srv::EmergencyStop_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  argj801_ctl_platform_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use argj801_ctl_platform_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const argj801_ctl_platform_interfaces::srv::EmergencyStop_Request & msg)
{
  return argj801_ctl_platform_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<argj801_ctl_platform_interfaces::srv::EmergencyStop_Request>()
{
  return "argj801_ctl_platform_interfaces::srv::EmergencyStop_Request";
}

template<>
inline const char * name<argj801_ctl_platform_interfaces::srv::EmergencyStop_Request>()
{
  return "argj801_ctl_platform_interfaces/srv/EmergencyStop_Request";
}

template<>
struct has_fixed_size<argj801_ctl_platform_interfaces::srv::EmergencyStop_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<argj801_ctl_platform_interfaces::srv::EmergencyStop_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<argj801_ctl_platform_interfaces::srv::EmergencyStop_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace argj801_ctl_platform_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const EmergencyStop_Response & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const EmergencyStop_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const EmergencyStop_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace argj801_ctl_platform_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use argj801_ctl_platform_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const argj801_ctl_platform_interfaces::srv::EmergencyStop_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  argj801_ctl_platform_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use argj801_ctl_platform_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const argj801_ctl_platform_interfaces::srv::EmergencyStop_Response & msg)
{
  return argj801_ctl_platform_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<argj801_ctl_platform_interfaces::srv::EmergencyStop_Response>()
{
  return "argj801_ctl_platform_interfaces::srv::EmergencyStop_Response";
}

template<>
inline const char * name<argj801_ctl_platform_interfaces::srv::EmergencyStop_Response>()
{
  return "argj801_ctl_platform_interfaces/srv/EmergencyStop_Response";
}

template<>
struct has_fixed_size<argj801_ctl_platform_interfaces::srv::EmergencyStop_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<argj801_ctl_platform_interfaces::srv::EmergencyStop_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<argj801_ctl_platform_interfaces::srv::EmergencyStop_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<argj801_ctl_platform_interfaces::srv::EmergencyStop>()
{
  return "argj801_ctl_platform_interfaces::srv::EmergencyStop";
}

template<>
inline const char * name<argj801_ctl_platform_interfaces::srv::EmergencyStop>()
{
  return "argj801_ctl_platform_interfaces/srv/EmergencyStop";
}

template<>
struct has_fixed_size<argj801_ctl_platform_interfaces::srv::EmergencyStop>
  : std::integral_constant<
    bool,
    has_fixed_size<argj801_ctl_platform_interfaces::srv::EmergencyStop_Request>::value &&
    has_fixed_size<argj801_ctl_platform_interfaces::srv::EmergencyStop_Response>::value
  >
{
};

template<>
struct has_bounded_size<argj801_ctl_platform_interfaces::srv::EmergencyStop>
  : std::integral_constant<
    bool,
    has_bounded_size<argj801_ctl_platform_interfaces::srv::EmergencyStop_Request>::value &&
    has_bounded_size<argj801_ctl_platform_interfaces::srv::EmergencyStop_Response>::value
  >
{
};

template<>
struct is_service<argj801_ctl_platform_interfaces::srv::EmergencyStop>
  : std::true_type
{
};

template<>
struct is_service_request<argj801_ctl_platform_interfaces::srv::EmergencyStop_Request>
  : std::true_type
{
};

template<>
struct is_service_response<argj801_ctl_platform_interfaces::srv::EmergencyStop_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ARGJ801_CTL_PLATFORM_INTERFACES__SRV__DETAIL__EMERGENCY_STOP__TRAITS_HPP_
