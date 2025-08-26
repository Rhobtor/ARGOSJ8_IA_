// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from argj801_sensors_msgs:msg/MotorTelemetry.idl
// generated code does not contain a copyright notice

#ifndef ARGJ801_SENSORS_MSGS__MSG__DETAIL__MOTOR_TELEMETRY__STRUCT_H_
#define ARGJ801_SENSORS_MSGS__MSG__DETAIL__MOTOR_TELEMETRY__STRUCT_H_

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

/// Struct defined in msg/MotorTelemetry in the package argj801_sensors_msgs.
typedef struct argj801_sensors_msgs__msg__MotorTelemetry
{
  std_msgs__msg__Header header;
  int64_t endocer;
  float rpm;
} argj801_sensors_msgs__msg__MotorTelemetry;

// Struct for a sequence of argj801_sensors_msgs__msg__MotorTelemetry.
typedef struct argj801_sensors_msgs__msg__MotorTelemetry__Sequence
{
  argj801_sensors_msgs__msg__MotorTelemetry * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} argj801_sensors_msgs__msg__MotorTelemetry__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARGJ801_SENSORS_MSGS__MSG__DETAIL__MOTOR_TELEMETRY__STRUCT_H_
