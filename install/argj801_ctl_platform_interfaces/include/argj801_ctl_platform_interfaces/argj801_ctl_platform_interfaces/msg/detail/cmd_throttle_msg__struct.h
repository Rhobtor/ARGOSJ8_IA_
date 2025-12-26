// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from argj801_ctl_platform_interfaces:msg/CmdThrottleMsg.idl
// generated code does not contain a copyright notice

#ifndef ARGJ801_CTL_PLATFORM_INTERFACES__MSG__DETAIL__CMD_THROTTLE_MSG__STRUCT_H_
#define ARGJ801_CTL_PLATFORM_INTERFACES__MSG__DETAIL__CMD_THROTTLE_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/CmdThrottleMsg in the package argj801_ctl_platform_interfaces.
/**
  * CmdThrottleMsg.msg
  *
  * Mensaje de mando a nivel bajo para la plataforma (modo Gazebo / LCM).
  *
  * Semántica:
  * - throttle: mando de acelerador/velocidad longitudinal (habitualmente en "porcentaje"
  *   o unidades normalizadas tras aplicar el modelo cinemático del vehículo).
  * - steering: mando de giro/dirección (también suele ser un porcentaje o valor
  *   normalizado).
  *
  * Nota: las unidades exactas dependen de los parámetros `throttle_to_percent` y
  * `steer_to_percent` y de la implementación del kinematic model.
 */
typedef struct argj801_ctl_platform_interfaces__msg__CmdThrottleMsg
{
  float throttle;
  float steering;
} argj801_ctl_platform_interfaces__msg__CmdThrottleMsg;

// Struct for a sequence of argj801_ctl_platform_interfaces__msg__CmdThrottleMsg.
typedef struct argj801_ctl_platform_interfaces__msg__CmdThrottleMsg__Sequence
{
  argj801_ctl_platform_interfaces__msg__CmdThrottleMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} argj801_ctl_platform_interfaces__msg__CmdThrottleMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ARGJ801_CTL_PLATFORM_INTERFACES__MSG__DETAIL__CMD_THROTTLE_MSG__STRUCT_H_
