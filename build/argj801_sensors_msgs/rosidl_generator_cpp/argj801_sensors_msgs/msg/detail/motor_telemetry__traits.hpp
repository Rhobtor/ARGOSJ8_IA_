// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from argj801_sensors_msgs:msg/MotorTelemetry.idl
// generated code does not contain a copyright notice

#ifndef ARGJ801_SENSORS_MSGS__MSG__DETAIL__MOTOR_TELEMETRY__TRAITS_HPP_
#define ARGJ801_SENSORS_MSGS__MSG__DETAIL__MOTOR_TELEMETRY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "argj801_sensors_msgs/msg/detail/motor_telemetry__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace argj801_sensors_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const MotorTelemetry & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: endocer
  {
    out << "endocer: ";
    rosidl_generator_traits::value_to_yaml(msg.endocer, out);
    out << ", ";
  }

  // member: rpm
  {
    out << "rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.rpm, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MotorTelemetry & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: endocer
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "endocer: ";
    rosidl_generator_traits::value_to_yaml(msg.endocer, out);
    out << "\n";
  }

  // member: rpm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.rpm, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MotorTelemetry & msg, bool use_flow_style = false)
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

}  // namespace argj801_sensors_msgs

namespace rosidl_generator_traits
{

[[deprecated("use argj801_sensors_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const argj801_sensors_msgs::msg::MotorTelemetry & msg,
  std::ostream & out, size_t indentation = 0)
{
  argj801_sensors_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use argj801_sensors_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const argj801_sensors_msgs::msg::MotorTelemetry & msg)
{
  return argj801_sensors_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<argj801_sensors_msgs::msg::MotorTelemetry>()
{
  return "argj801_sensors_msgs::msg::MotorTelemetry";
}

template<>
inline const char * name<argj801_sensors_msgs::msg::MotorTelemetry>()
{
  return "argj801_sensors_msgs/msg/MotorTelemetry";
}

template<>
struct has_fixed_size<argj801_sensors_msgs::msg::MotorTelemetry>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<argj801_sensors_msgs::msg::MotorTelemetry>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<argj801_sensors_msgs::msg::MotorTelemetry>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ARGJ801_SENSORS_MSGS__MSG__DETAIL__MOTOR_TELEMETRY__TRAITS_HPP_
