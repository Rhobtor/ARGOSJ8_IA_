// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from argj801_ctl_platform_interfaces:srv/GetVelocity.idl
// generated code does not contain a copyright notice

#ifndef ARGJ801_CTL_PLATFORM_INTERFACES__SRV__DETAIL__GET_VELOCITY__STRUCT_H_
#define ARGJ801_CTL_PLATFORM_INTERFACES__SRV__DETAIL__GET_VELOCITY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetVelocity in the package argj801_ctl_platform_interfaces.
typedef struct argj801_ctl_platform_interfaces__srv__GetVelocity_Request
{
  uint8_t structure_needs_at_least_one_member;
} argj801_ctl_platform_interfaces__srv__GetVelocity_Request;

// Struct for a sequence of argj801_ctl_platform_interfaces__srv__GetVelocity_Request.
typedef struct argj801_ctl_platform_interfaces__srv__GetVelocity_Request__Sequence
{
  argj801_ctl_platform_interfaces__srv__GetVelocity_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} argj801_ctl_platform_interfaces__srv__GetVelocity_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/GetVelocity in the package argj801_ctl_platform_interfaces.
typedef struct argj801_ctl_platform_interfaces__srv__GetVelocity_Response
{
  int8_t actual_velocity;
} argj801_ctl_platform_interfaces__srv__GetVelocity_Response;

// Struct for a sequence of argj801_ctl_platform_interfaces__srv__GetVelocity_Response.
typedef struct argj801_ctl_platform_interfaces__srv__GetVelocity_Response__Sequence
{
  argj801_ctl_platform_interfaces__srv__GetVelocity_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} argj801_ctl_platform_interfaces__srv__GetVelocity_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARGJ801_CTL_PLATFORM_INTERFACES__SRV__DETAIL__GET_VELOCITY__STRUCT_H_
