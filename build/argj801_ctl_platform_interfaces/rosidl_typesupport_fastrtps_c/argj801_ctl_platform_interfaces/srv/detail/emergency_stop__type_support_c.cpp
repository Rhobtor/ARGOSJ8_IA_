// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from argj801_ctl_platform_interfaces:srv/EmergencyStop.idl
// generated code does not contain a copyright notice
#include "argj801_ctl_platform_interfaces/srv/detail/emergency_stop__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "argj801_ctl_platform_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "argj801_ctl_platform_interfaces/srv/detail/emergency_stop__struct.h"
#include "argj801_ctl_platform_interfaces/srv/detail/emergency_stop__functions.h"
#include "fastcdr/Cdr.h"

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

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _EmergencyStop_Request__ros_msg_type = argj801_ctl_platform_interfaces__srv__EmergencyStop_Request;

static bool _EmergencyStop_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _EmergencyStop_Request__ros_msg_type * ros_message = static_cast<const _EmergencyStop_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: stop
  {
    cdr << (ros_message->stop ? true : false);
  }

  return true;
}

static bool _EmergencyStop_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _EmergencyStop_Request__ros_msg_type * ros_message = static_cast<_EmergencyStop_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: stop
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->stop = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_argj801_ctl_platform_interfaces
size_t get_serialized_size_argj801_ctl_platform_interfaces__srv__EmergencyStop_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _EmergencyStop_Request__ros_msg_type * ros_message = static_cast<const _EmergencyStop_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name stop
  {
    size_t item_size = sizeof(ros_message->stop);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _EmergencyStop_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_argj801_ctl_platform_interfaces__srv__EmergencyStop_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_argj801_ctl_platform_interfaces
size_t max_serialized_size_argj801_ctl_platform_interfaces__srv__EmergencyStop_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: stop
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = argj801_ctl_platform_interfaces__srv__EmergencyStop_Request;
    is_plain =
      (
      offsetof(DataType, stop) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _EmergencyStop_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_argj801_ctl_platform_interfaces__srv__EmergencyStop_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_EmergencyStop_Request = {
  "argj801_ctl_platform_interfaces::srv",
  "EmergencyStop_Request",
  _EmergencyStop_Request__cdr_serialize,
  _EmergencyStop_Request__cdr_deserialize,
  _EmergencyStop_Request__get_serialized_size,
  _EmergencyStop_Request__max_serialized_size
};

static rosidl_message_type_support_t _EmergencyStop_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_EmergencyStop_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, argj801_ctl_platform_interfaces, srv, EmergencyStop_Request)() {
  return &_EmergencyStop_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "argj801_ctl_platform_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "argj801_ctl_platform_interfaces/srv/detail/emergency_stop__struct.h"
// already included above
// #include "argj801_ctl_platform_interfaces/srv/detail/emergency_stop__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

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

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _EmergencyStop_Response__ros_msg_type = argj801_ctl_platform_interfaces__srv__EmergencyStop_Response;

static bool _EmergencyStop_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _EmergencyStop_Response__ros_msg_type * ros_message = static_cast<const _EmergencyStop_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: structure_needs_at_least_one_member
  {
    cdr << ros_message->structure_needs_at_least_one_member;
  }

  return true;
}

static bool _EmergencyStop_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _EmergencyStop_Response__ros_msg_type * ros_message = static_cast<_EmergencyStop_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: structure_needs_at_least_one_member
  {
    cdr >> ros_message->structure_needs_at_least_one_member;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_argj801_ctl_platform_interfaces
size_t get_serialized_size_argj801_ctl_platform_interfaces__srv__EmergencyStop_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _EmergencyStop_Response__ros_msg_type * ros_message = static_cast<const _EmergencyStop_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name structure_needs_at_least_one_member
  {
    size_t item_size = sizeof(ros_message->structure_needs_at_least_one_member);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _EmergencyStop_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_argj801_ctl_platform_interfaces__srv__EmergencyStop_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_argj801_ctl_platform_interfaces
size_t max_serialized_size_argj801_ctl_platform_interfaces__srv__EmergencyStop_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: structure_needs_at_least_one_member
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = argj801_ctl_platform_interfaces__srv__EmergencyStop_Response;
    is_plain =
      (
      offsetof(DataType, structure_needs_at_least_one_member) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _EmergencyStop_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_argj801_ctl_platform_interfaces__srv__EmergencyStop_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_EmergencyStop_Response = {
  "argj801_ctl_platform_interfaces::srv",
  "EmergencyStop_Response",
  _EmergencyStop_Response__cdr_serialize,
  _EmergencyStop_Response__cdr_deserialize,
  _EmergencyStop_Response__get_serialized_size,
  _EmergencyStop_Response__max_serialized_size
};

static rosidl_message_type_support_t _EmergencyStop_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_EmergencyStop_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, argj801_ctl_platform_interfaces, srv, EmergencyStop_Response)() {
  return &_EmergencyStop_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "argj801_ctl_platform_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "argj801_ctl_platform_interfaces/srv/emergency_stop.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t EmergencyStop__callbacks = {
  "argj801_ctl_platform_interfaces::srv",
  "EmergencyStop",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, argj801_ctl_platform_interfaces, srv, EmergencyStop_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, argj801_ctl_platform_interfaces, srv, EmergencyStop_Response)(),
};

static rosidl_service_type_support_t EmergencyStop__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &EmergencyStop__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, argj801_ctl_platform_interfaces, srv, EmergencyStop)() {
  return &EmergencyStop__handle;
}

#if defined(__cplusplus)
}
#endif
