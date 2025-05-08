// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from argj801_sensors_msgs:msg/Odometer.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "argj801_sensors_msgs/msg/detail/odometer__rosidl_typesupport_introspection_c.h"
#include "argj801_sensors_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "argj801_sensors_msgs/msg/detail/odometer__functions.h"
#include "argj801_sensors_msgs/msg/detail/odometer__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void argj801_sensors_msgs__msg__Odometer__rosidl_typesupport_introspection_c__Odometer_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  argj801_sensors_msgs__msg__Odometer__init(message_memory);
}

void argj801_sensors_msgs__msg__Odometer__rosidl_typesupport_introspection_c__Odometer_fini_function(void * message_memory)
{
  argj801_sensors_msgs__msg__Odometer__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember argj801_sensors_msgs__msg__Odometer__rosidl_typesupport_introspection_c__Odometer_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(argj801_sensors_msgs__msg__Odometer, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "odometer",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(argj801_sensors_msgs__msg__Odometer, odometer),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers argj801_sensors_msgs__msg__Odometer__rosidl_typesupport_introspection_c__Odometer_message_members = {
  "argj801_sensors_msgs__msg",  // message namespace
  "Odometer",  // message name
  2,  // number of fields
  sizeof(argj801_sensors_msgs__msg__Odometer),
  argj801_sensors_msgs__msg__Odometer__rosidl_typesupport_introspection_c__Odometer_message_member_array,  // message members
  argj801_sensors_msgs__msg__Odometer__rosidl_typesupport_introspection_c__Odometer_init_function,  // function to initialize message memory (memory has to be allocated)
  argj801_sensors_msgs__msg__Odometer__rosidl_typesupport_introspection_c__Odometer_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t argj801_sensors_msgs__msg__Odometer__rosidl_typesupport_introspection_c__Odometer_message_type_support_handle = {
  0,
  &argj801_sensors_msgs__msg__Odometer__rosidl_typesupport_introspection_c__Odometer_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_argj801_sensors_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, argj801_sensors_msgs, msg, Odometer)() {
  argj801_sensors_msgs__msg__Odometer__rosidl_typesupport_introspection_c__Odometer_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!argj801_sensors_msgs__msg__Odometer__rosidl_typesupport_introspection_c__Odometer_message_type_support_handle.typesupport_identifier) {
    argj801_sensors_msgs__msg__Odometer__rosidl_typesupport_introspection_c__Odometer_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &argj801_sensors_msgs__msg__Odometer__rosidl_typesupport_introspection_c__Odometer_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
