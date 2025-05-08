// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from ctl_mission_interfaces:srv/ConfigDynamicLAPureCtrl.idl
// generated code does not contain a copyright notice
#include "ctl_mission_interfaces/srv/detail/config_dynamic_la_pure_ctrl__rosidl_typesupport_fastrtps_cpp.hpp"
#include "ctl_mission_interfaces/srv/detail/config_dynamic_la_pure_ctrl__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace ctl_mission_interfaces
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ctl_mission_interfaces
cdr_serialize(
  const ctl_mission_interfaces::srv::ConfigDynamicLAPureCtrl_Request & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: look_ahead_v_gain
  cdr << ros_message.look_ahead_v_gain;
  // Member: max_v_forward
  cdr << ros_message.max_v_forward;
  // Member: max_ang_acc
  cdr << ros_message.max_ang_acc;
  // Member: max_ang_dec
  cdr << ros_message.max_ang_dec;
  // Member: max_lin_acc
  cdr << ros_message.max_lin_acc;
  // Member: max_lin_dec
  cdr << ros_message.max_lin_dec;
  // Member: speed_pow
  cdr << ros_message.speed_pow;
  // Member: min_look_ahead_d
  cdr << ros_message.min_look_ahead_d;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ctl_mission_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ctl_mission_interfaces::srv::ConfigDynamicLAPureCtrl_Request & ros_message)
{
  // Member: look_ahead_v_gain
  cdr >> ros_message.look_ahead_v_gain;

  // Member: max_v_forward
  cdr >> ros_message.max_v_forward;

  // Member: max_ang_acc
  cdr >> ros_message.max_ang_acc;

  // Member: max_ang_dec
  cdr >> ros_message.max_ang_dec;

  // Member: max_lin_acc
  cdr >> ros_message.max_lin_acc;

  // Member: max_lin_dec
  cdr >> ros_message.max_lin_dec;

  // Member: speed_pow
  cdr >> ros_message.speed_pow;

  // Member: min_look_ahead_d
  cdr >> ros_message.min_look_ahead_d;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ctl_mission_interfaces
get_serialized_size(
  const ctl_mission_interfaces::srv::ConfigDynamicLAPureCtrl_Request & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: look_ahead_v_gain
  {
    size_t item_size = sizeof(ros_message.look_ahead_v_gain);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: max_v_forward
  {
    size_t item_size = sizeof(ros_message.max_v_forward);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: max_ang_acc
  {
    size_t item_size = sizeof(ros_message.max_ang_acc);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: max_ang_dec
  {
    size_t item_size = sizeof(ros_message.max_ang_dec);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: max_lin_acc
  {
    size_t item_size = sizeof(ros_message.max_lin_acc);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: max_lin_dec
  {
    size_t item_size = sizeof(ros_message.max_lin_dec);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: speed_pow
  {
    size_t item_size = sizeof(ros_message.speed_pow);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: min_look_ahead_d
  {
    size_t item_size = sizeof(ros_message.min_look_ahead_d);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ctl_mission_interfaces
max_serialized_size_ConfigDynamicLAPureCtrl_Request(
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


  // Member: look_ahead_v_gain
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: max_v_forward
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: max_ang_acc
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: max_ang_dec
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: max_lin_acc
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: max_lin_dec
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: speed_pow
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: min_look_ahead_d
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = ctl_mission_interfaces::srv::ConfigDynamicLAPureCtrl_Request;
    is_plain =
      (
      offsetof(DataType, min_look_ahead_d) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _ConfigDynamicLAPureCtrl_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const ctl_mission_interfaces::srv::ConfigDynamicLAPureCtrl_Request *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _ConfigDynamicLAPureCtrl_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<ctl_mission_interfaces::srv::ConfigDynamicLAPureCtrl_Request *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _ConfigDynamicLAPureCtrl_Request__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const ctl_mission_interfaces::srv::ConfigDynamicLAPureCtrl_Request *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _ConfigDynamicLAPureCtrl_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ConfigDynamicLAPureCtrl_Request(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _ConfigDynamicLAPureCtrl_Request__callbacks = {
  "ctl_mission_interfaces::srv",
  "ConfigDynamicLAPureCtrl_Request",
  _ConfigDynamicLAPureCtrl_Request__cdr_serialize,
  _ConfigDynamicLAPureCtrl_Request__cdr_deserialize,
  _ConfigDynamicLAPureCtrl_Request__get_serialized_size,
  _ConfigDynamicLAPureCtrl_Request__max_serialized_size
};

static rosidl_message_type_support_t _ConfigDynamicLAPureCtrl_Request__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_ConfigDynamicLAPureCtrl_Request__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace ctl_mission_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_ctl_mission_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<ctl_mission_interfaces::srv::ConfigDynamicLAPureCtrl_Request>()
{
  return &ctl_mission_interfaces::srv::typesupport_fastrtps_cpp::_ConfigDynamicLAPureCtrl_Request__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ctl_mission_interfaces, srv, ConfigDynamicLAPureCtrl_Request)() {
  return &ctl_mission_interfaces::srv::typesupport_fastrtps_cpp::_ConfigDynamicLAPureCtrl_Request__handle;
}

#ifdef __cplusplus
}
#endif

// already included above
// #include <limits>
// already included above
// #include <stdexcept>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
// already included above
// #include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace ctl_mission_interfaces
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ctl_mission_interfaces
cdr_serialize(
  const ctl_mission_interfaces::srv::ConfigDynamicLAPureCtrl_Response & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: ack
  cdr << (ros_message.ack ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ctl_mission_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ctl_mission_interfaces::srv::ConfigDynamicLAPureCtrl_Response & ros_message)
{
  // Member: ack
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.ack = tmp ? true : false;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ctl_mission_interfaces
get_serialized_size(
  const ctl_mission_interfaces::srv::ConfigDynamicLAPureCtrl_Response & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: ack
  {
    size_t item_size = sizeof(ros_message.ack);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ctl_mission_interfaces
max_serialized_size_ConfigDynamicLAPureCtrl_Response(
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


  // Member: ack
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
    using DataType = ctl_mission_interfaces::srv::ConfigDynamicLAPureCtrl_Response;
    is_plain =
      (
      offsetof(DataType, ack) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _ConfigDynamicLAPureCtrl_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const ctl_mission_interfaces::srv::ConfigDynamicLAPureCtrl_Response *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _ConfigDynamicLAPureCtrl_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<ctl_mission_interfaces::srv::ConfigDynamicLAPureCtrl_Response *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _ConfigDynamicLAPureCtrl_Response__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const ctl_mission_interfaces::srv::ConfigDynamicLAPureCtrl_Response *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _ConfigDynamicLAPureCtrl_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ConfigDynamicLAPureCtrl_Response(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _ConfigDynamicLAPureCtrl_Response__callbacks = {
  "ctl_mission_interfaces::srv",
  "ConfigDynamicLAPureCtrl_Response",
  _ConfigDynamicLAPureCtrl_Response__cdr_serialize,
  _ConfigDynamicLAPureCtrl_Response__cdr_deserialize,
  _ConfigDynamicLAPureCtrl_Response__get_serialized_size,
  _ConfigDynamicLAPureCtrl_Response__max_serialized_size
};

static rosidl_message_type_support_t _ConfigDynamicLAPureCtrl_Response__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_ConfigDynamicLAPureCtrl_Response__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace ctl_mission_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_ctl_mission_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<ctl_mission_interfaces::srv::ConfigDynamicLAPureCtrl_Response>()
{
  return &ctl_mission_interfaces::srv::typesupport_fastrtps_cpp::_ConfigDynamicLAPureCtrl_Response__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ctl_mission_interfaces, srv, ConfigDynamicLAPureCtrl_Response)() {
  return &ctl_mission_interfaces::srv::typesupport_fastrtps_cpp::_ConfigDynamicLAPureCtrl_Response__handle;
}

#ifdef __cplusplus
}
#endif

#include "rmw/error_handling.h"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support_decl.hpp"

namespace ctl_mission_interfaces
{

namespace srv
{

namespace typesupport_fastrtps_cpp
{

static service_type_support_callbacks_t _ConfigDynamicLAPureCtrl__callbacks = {
  "ctl_mission_interfaces::srv",
  "ConfigDynamicLAPureCtrl",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ctl_mission_interfaces, srv, ConfigDynamicLAPureCtrl_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ctl_mission_interfaces, srv, ConfigDynamicLAPureCtrl_Response)(),
};

static rosidl_service_type_support_t _ConfigDynamicLAPureCtrl__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_ConfigDynamicLAPureCtrl__callbacks,
  get_service_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace srv

}  // namespace ctl_mission_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_ctl_mission_interfaces
const rosidl_service_type_support_t *
get_service_type_support_handle<ctl_mission_interfaces::srv::ConfigDynamicLAPureCtrl>()
{
  return &ctl_mission_interfaces::srv::typesupport_fastrtps_cpp::_ConfigDynamicLAPureCtrl__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ctl_mission_interfaces, srv, ConfigDynamicLAPureCtrl)() {
  return &ctl_mission_interfaces::srv::typesupport_fastrtps_cpp::_ConfigDynamicLAPureCtrl__handle;
}

#ifdef __cplusplus
}
#endif
