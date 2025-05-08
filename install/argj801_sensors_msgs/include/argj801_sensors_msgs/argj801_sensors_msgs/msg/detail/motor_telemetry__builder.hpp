// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from argj801_sensors_msgs:msg/MotorTelemetry.idl
// generated code does not contain a copyright notice

#ifndef ARGJ801_SENSORS_MSGS__MSG__DETAIL__MOTOR_TELEMETRY__BUILDER_HPP_
#define ARGJ801_SENSORS_MSGS__MSG__DETAIL__MOTOR_TELEMETRY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "argj801_sensors_msgs/msg/detail/motor_telemetry__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace argj801_sensors_msgs
{

namespace msg
{

namespace builder
{

class Init_MotorTelemetry_rpm
{
public:
  explicit Init_MotorTelemetry_rpm(::argj801_sensors_msgs::msg::MotorTelemetry & msg)
  : msg_(msg)
  {}
  ::argj801_sensors_msgs::msg::MotorTelemetry rpm(::argj801_sensors_msgs::msg::MotorTelemetry::_rpm_type arg)
  {
    msg_.rpm = std::move(arg);
    return std::move(msg_);
  }

private:
  ::argj801_sensors_msgs::msg::MotorTelemetry msg_;
};

class Init_MotorTelemetry_endocer
{
public:
  explicit Init_MotorTelemetry_endocer(::argj801_sensors_msgs::msg::MotorTelemetry & msg)
  : msg_(msg)
  {}
  Init_MotorTelemetry_rpm endocer(::argj801_sensors_msgs::msg::MotorTelemetry::_endocer_type arg)
  {
    msg_.endocer = std::move(arg);
    return Init_MotorTelemetry_rpm(msg_);
  }

private:
  ::argj801_sensors_msgs::msg::MotorTelemetry msg_;
};

class Init_MotorTelemetry_header
{
public:
  Init_MotorTelemetry_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorTelemetry_endocer header(::argj801_sensors_msgs::msg::MotorTelemetry::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MotorTelemetry_endocer(msg_);
  }

private:
  ::argj801_sensors_msgs::msg::MotorTelemetry msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::argj801_sensors_msgs::msg::MotorTelemetry>()
{
  return argj801_sensors_msgs::msg::builder::Init_MotorTelemetry_header();
}

}  // namespace argj801_sensors_msgs

#endif  // ARGJ801_SENSORS_MSGS__MSG__DETAIL__MOTOR_TELEMETRY__BUILDER_HPP_
