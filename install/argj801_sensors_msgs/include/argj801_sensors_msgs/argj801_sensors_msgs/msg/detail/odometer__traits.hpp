// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from argj801_sensors_msgs:msg/Odometer.idl
// generated code does not contain a copyright notice

#ifndef ARGJ801_SENSORS_MSGS__MSG__DETAIL__ODOMETER__TRAITS_HPP_
#define ARGJ801_SENSORS_MSGS__MSG__DETAIL__ODOMETER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "argj801_sensors_msgs/msg/detail/odometer__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace argj801_sensors_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Odometer & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: odometer
  {
    out << "odometer: ";
    rosidl_generator_traits::value_to_yaml(msg.odometer, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Odometer & msg,
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

  // member: odometer
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "odometer: ";
    rosidl_generator_traits::value_to_yaml(msg.odometer, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Odometer & msg, bool use_flow_style = false)
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
  const argj801_sensors_msgs::msg::Odometer & msg,
  std::ostream & out, size_t indentation = 0)
{
  argj801_sensors_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use argj801_sensors_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const argj801_sensors_msgs::msg::Odometer & msg)
{
  return argj801_sensors_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<argj801_sensors_msgs::msg::Odometer>()
{
  return "argj801_sensors_msgs::msg::Odometer";
}

template<>
inline const char * name<argj801_sensors_msgs::msg::Odometer>()
{
  return "argj801_sensors_msgs/msg/Odometer";
}

template<>
struct has_fixed_size<argj801_sensors_msgs::msg::Odometer>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<argj801_sensors_msgs::msg::Odometer>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<argj801_sensors_msgs::msg::Odometer>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ARGJ801_SENSORS_MSGS__MSG__DETAIL__ODOMETER__TRAITS_HPP_
