// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from argj801_sensors_msgs:msg/Odometer.idl
// generated code does not contain a copyright notice

#ifndef ARGJ801_SENSORS_MSGS__MSG__DETAIL__ODOMETER__STRUCT_H_
#define ARGJ801_SENSORS_MSGS__MSG__DETAIL__ODOMETER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/Odometer in the package argj801_sensors_msgs.
typedef struct argj801_sensors_msgs__msg__Odometer
{
  std_msgs__msg__Header header;
  int64_t odometer;
} argj801_sensors_msgs__msg__Odometer;

// Struct for a sequence of argj801_sensors_msgs__msg__Odometer.
typedef struct argj801_sensors_msgs__msg__Odometer__Sequence
{
  argj801_sensors_msgs__msg__Odometer * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} argj801_sensors_msgs__msg__Odometer__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARGJ801_SENSORS_MSGS__MSG__DETAIL__ODOMETER__STRUCT_H_
