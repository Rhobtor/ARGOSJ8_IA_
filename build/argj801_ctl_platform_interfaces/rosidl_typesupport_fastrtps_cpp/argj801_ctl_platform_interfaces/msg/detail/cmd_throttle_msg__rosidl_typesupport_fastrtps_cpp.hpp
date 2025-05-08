// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from argj801_ctl_platform_interfaces:msg/CmdThrottleMsg.idl
// generated code does not contain a copyright notice

#ifndef ARGJ801_CTL_PLATFORM_INTERFACES__MSG__DETAIL__CMD_THROTTLE_MSG__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define ARGJ801_CTL_PLATFORM_INTERFACES__MSG__DETAIL__CMD_THROTTLE_MSG__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "argj801_ctl_platform_interfaces/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "argj801_ctl_platform_interfaces/msg/detail/cmd_throttle_msg__struct.hpp"

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

namespace argj801_ctl_platform_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_argj801_ctl_platform_interfaces
cdr_serialize(
  const argj801_ctl_platform_interfaces::msg::CmdThrottleMsg & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_argj801_ctl_platform_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  argj801_ctl_platform_interfaces::msg::CmdThrottleMsg & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_argj801_ctl_platform_interfaces
get_serialized_size(
  const argj801_ctl_platform_interfaces::msg::CmdThrottleMsg & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_argj801_ctl_platform_interfaces
max_serialized_size_CmdThrottleMsg(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace argj801_ctl_platform_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_argj801_ctl_platform_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, argj801_ctl_platform_interfaces, msg, CmdThrottleMsg)();

#ifdef __cplusplus
}
#endif

#endif  // ARGJ801_CTL_PLATFORM_INTERFACES__MSG__DETAIL__CMD_THROTTLE_MSG__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
