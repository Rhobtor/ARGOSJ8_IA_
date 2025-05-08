// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from argj801_sensors_msgs:msg/Odometer.idl
// generated code does not contain a copyright notice

#ifndef ARGJ801_SENSORS_MSGS__MSG__DETAIL__ODOMETER__BUILDER_HPP_
#define ARGJ801_SENSORS_MSGS__MSG__DETAIL__ODOMETER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "argj801_sensors_msgs/msg/detail/odometer__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace argj801_sensors_msgs
{

namespace msg
{

namespace builder
{

class Init_Odometer_odometer
{
public:
  explicit Init_Odometer_odometer(::argj801_sensors_msgs::msg::Odometer & msg)
  : msg_(msg)
  {}
  ::argj801_sensors_msgs::msg::Odometer odometer(::argj801_sensors_msgs::msg::Odometer::_odometer_type arg)
  {
    msg_.odometer = std::move(arg);
    return std::move(msg_);
  }

private:
  ::argj801_sensors_msgs::msg::Odometer msg_;
};

class Init_Odometer_header
{
public:
  Init_Odometer_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Odometer_odometer header(::argj801_sensors_msgs::msg::Odometer::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Odometer_odometer(msg_);
  }

private:
  ::argj801_sensors_msgs::msg::Odometer msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::argj801_sensors_msgs::msg::Odometer>()
{
  return argj801_sensors_msgs::msg::builder::Init_Odometer_header();
}

}  // namespace argj801_sensors_msgs

#endif  // ARGJ801_SENSORS_MSGS__MSG__DETAIL__ODOMETER__BUILDER_HPP_
