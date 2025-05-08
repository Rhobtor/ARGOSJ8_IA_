// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from ctl_mission_interfaces:srv/ConfigRegulatedPureCtrl.idl
// generated code does not contain a copyright notice
#include "ctl_mission_interfaces/srv/detail/config_regulated_pure_ctrl__rosidl_typesupport_fastrtps_cpp.hpp"
#include "ctl_mission_interfaces/srv/detail/config_regulated_pure_ctrl__struct.hpp"

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
  const ctl_mission_interfaces::srv::ConfigRegulatedPureCtrl_Request & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: look_ahead_dis
  cdr << ros_message.look_ahead_dis;
  // Member: v_forward
  cdr << ros_message.v_forward;
  // Member: r_min
  cdr << ros_message.r_min;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ctl_mission_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ctl_mission_interfaces::srv::ConfigRegulatedPureCtrl_Request & ros_message)
{
  // Member: look_ahead_dis
  cdr >> ros_message.look_ahead_dis;

  // Member: v_forward
  cdr >> ros_message.v_forward;

  // Member: r_min
  cdr >> ros_message.r_min;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ctl_mission_interfaces
get_serialized_size(
  const ctl_mission_interfaces::srv::ConfigRegulatedPureCtrl_Request & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: look_ahead_dis
  {
    size_t item_size = sizeof(ros_message.look_ahead_dis);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: v_forward
  {
    size_t item_size = sizeof(ros_message.v_forward);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: r_min
  {
    size_t item_size = sizeof(ros_message.r_min);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ctl_mission_interfaces
max_serialized_size_ConfigRegulatedPureCtrl_Request(
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


  // Member: look_ahead_dis
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: v_forward
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: r_min
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
    using DataType = ctl_mission_interfaces::srv::ConfigRegulatedPureCtrl_Request;
    is_plain =
      (
      offsetof(DataType, r_min) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _ConfigRegulatedPureCtrl_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const ctl_mission_interfaces::srv::ConfigRegulatedPureCtrl_Request *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _ConfigRegulatedPureCtrl_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<ctl_mission_interfaces::srv::ConfigRegulatedPureCtrl_Request *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _ConfigRegulatedPureCtrl_Request__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const ctl_mission_interfaces::srv::ConfigRegulatedPureCtrl_Request *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _ConfigRegulatedPureCtrl_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ConfigRegulatedPureCtrl_Request(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _ConfigRegulatedPureCtrl_Request__callbacks = {
  "ctl_mission_interfaces::srv",
  "ConfigRegulatedPureCtrl_Request",
  _ConfigRegulatedPureCtrl_Request__cdr_serialize,
  _ConfigRegulatedPureCtrl_Request__cdr_deserialize,
  _ConfigRegulatedPureCtrl_Request__get_serialized_size,
  _ConfigRegulatedPureCtrl_Request__max_serialized_size
};

static rosidl_message_type_support_t _ConfigRegulatedPureCtrl_Request__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_ConfigRegulatedPureCtrl_Request__callbacks,
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
get_message_type_support_handle<ctl_mission_interfaces::srv::ConfigRegulatedPureCtrl_Request>()
{
  return &ctl_mission_interfaces::srv::typesupport_fastrtps_cpp::_ConfigRegulatedPureCtrl_Request__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ctl_mission_interfaces, srv, ConfigRegulatedPureCtrl_Request)() {
  return &ctl_mission_interfaces::srv::typesupport_fastrtps_cpp::_ConfigRegulatedPureCtrl_Request__handle;
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
  const ctl_mission_interfaces::srv::ConfigRegulatedPureCtrl_Response & ros_message,
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
  ctl_mission_interfaces::srv::ConfigRegulatedPureCtrl_Response & ros_message)
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
  const ctl_mission_interfaces::srv::ConfigRegulatedPureCtrl_Response & ros_message,
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
max_serialized_size_ConfigRegulatedPureCtrl_Response(
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
    using DataType = ctl_mission_interfaces::srv::ConfigRegulatedPureCtrl_Response;
    is_plain =
      (
      offsetof(DataType, ack) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _ConfigRegulatedPureCtrl_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const ctl_mission_interfaces::srv::ConfigRegulatedPureCtrl_Response *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _ConfigRegulatedPureCtrl_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<ctl_mission_interfaces::srv::ConfigRegulatedPureCtrl_Response *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _ConfigRegulatedPureCtrl_Response__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const ctl_mission_interfaces::srv::ConfigRegulatedPureCtrl_Response *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _ConfigRegulatedPureCtrl_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ConfigRegulatedPureCtrl_Response(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _ConfigRegulatedPureCtrl_Response__callbacks = {
  "ctl_mission_interfaces::srv",
  "ConfigRegulatedPureCtrl_Response",
  _ConfigRegulatedPureCtrl_Response__cdr_serialize,
  _ConfigRegulatedPureCtrl_Response__cdr_deserialize,
  _ConfigRegulatedPureCtrl_Response__get_serialized_size,
  _ConfigRegulatedPureCtrl_Response__max_serialized_size
};

static rosidl_message_type_support_t _ConfigRegulatedPureCtrl_Response__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_ConfigRegulatedPureCtrl_Response__callbacks,
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
get_message_type_support_handle<ctl_mission_interfaces::srv::ConfigRegulatedPureCtrl_Response>()
{
  return &ctl_mission_interfaces::srv::typesupport_fastrtps_cpp::_ConfigRegulatedPureCtrl_Response__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ctl_mission_interfaces, srv, ConfigRegulatedPureCtrl_Response)() {
  return &ctl_mission_interfaces::srv::typesupport_fastrtps_cpp::_ConfigRegulatedPureCtrl_Response__handle;
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

static service_type_support_callbacks_t _ConfigRegulatedPureCtrl__callbacks = {
  "ctl_mission_interfaces::srv",
  "ConfigRegulatedPureCtrl",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ctl_mission_interfaces, srv, ConfigRegulatedPureCtrl_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ctl_mission_interfaces, srv, ConfigRegulatedPureCtrl_Response)(),
};

static rosidl_service_type_support_t _ConfigRegulatedPureCtrl__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_ConfigRegulatedPureCtrl__callbacks,
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
get_service_type_support_handle<ctl_mission_interfaces::srv::ConfigRegulatedPureCtrl>()
{
  return &ctl_mission_interfaces::srv::typesupport_fastrtps_cpp::_ConfigRegulatedPureCtrl__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ctl_mission_interfaces, srv, ConfigRegulatedPureCtrl)() {
  return &ctl_mission_interfaces::srv::typesupport_fastrtps_cpp::_ConfigRegulatedPureCtrl__handle;
}

#ifdef __cplusplus
}
#endif
