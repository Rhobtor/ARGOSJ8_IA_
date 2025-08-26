// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from argj801_sensors_msgs:msg/MotorTelemetry.idl
// generated code does not contain a copyright notice

#ifndef ARGJ801_SENSORS_MSGS__MSG__DETAIL__MOTOR_TELEMETRY__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define ARGJ801_SENSORS_MSGS__MSG__DETAIL__MOTOR_TELEMETRY__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "argj801_sensors_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "argj801_sensors_msgs/msg/detail/motor_telemetry__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace argj801_sensors_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_argj801_sensors_msgs
cdr_serialize(
  const argj801_sensors_msgs::msg::MotorTelemetry & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_argj801_sensors_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  argj801_sensors_msgs::msg::MotorTelemetry & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_argj801_sensors_msgs
get_serialized_size(
  const argj801_sensors_msgs::msg::MotorTelemetry & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_argj801_sensors_msgs
max_serialized_size_MotorTelemetry(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace argj801_sensors_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_argj801_sensors_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, argj801_sensors_msgs, msg, MotorTelemetry)();

#ifdef __cplusplus
}
#endif

#endif  // ARGJ801_SENSORS_MSGS__MSG__DETAIL__MOTOR_TELEMETRY__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
