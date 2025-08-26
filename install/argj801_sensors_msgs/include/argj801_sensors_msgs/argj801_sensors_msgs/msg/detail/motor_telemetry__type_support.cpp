// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from argj801_sensors_msgs:msg/MotorTelemetry.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "argj801_sensors_msgs/msg/detail/motor_telemetry__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace argj801_sensors_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void MotorTelemetry_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) argj801_sensors_msgs::msg::MotorTelemetry(_init);
}

void MotorTelemetry_fini_function(void * message_memory)
{
  auto typed_message = static_cast<argj801_sensors_msgs::msg::MotorTelemetry *>(message_memory);
  typed_message->~MotorTelemetry();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember MotorTelemetry_message_member_array[3] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(argj801_sensors_msgs::msg::MotorTelemetry, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "endocer",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(argj801_sensors_msgs::msg::MotorTelemetry, endocer),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "rpm",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(argj801_sensors_msgs::msg::MotorTelemetry, rpm),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers MotorTelemetry_message_members = {
  "argj801_sensors_msgs::msg",  // message namespace
  "MotorTelemetry",  // message name
  3,  // number of fields
  sizeof(argj801_sensors_msgs::msg::MotorTelemetry),
  MotorTelemetry_message_member_array,  // message members
  MotorTelemetry_init_function,  // function to initialize message memory (memory has to be allocated)
  MotorTelemetry_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t MotorTelemetry_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &MotorTelemetry_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace argj801_sensors_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<argj801_sensors_msgs::msg::MotorTelemetry>()
{
  return &::argj801_sensors_msgs::msg::rosidl_typesupport_introspection_cpp::MotorTelemetry_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, argj801_sensors_msgs, msg, MotorTelemetry)() {
  return &::argj801_sensors_msgs::msg::rosidl_typesupport_introspection_cpp::MotorTelemetry_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
