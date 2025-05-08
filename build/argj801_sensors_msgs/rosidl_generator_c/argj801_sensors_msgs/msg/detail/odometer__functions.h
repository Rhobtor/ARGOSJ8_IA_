// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from argj801_sensors_msgs:msg/Odometer.idl
// generated code does not contain a copyright notice

#ifndef ARGJ801_SENSORS_MSGS__MSG__DETAIL__ODOMETER__FUNCTIONS_H_
#define ARGJ801_SENSORS_MSGS__MSG__DETAIL__ODOMETER__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "argj801_sensors_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "argj801_sensors_msgs/msg/detail/odometer__struct.h"

/// Initialize msg/Odometer message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * argj801_sensors_msgs__msg__Odometer
 * )) before or use
 * argj801_sensors_msgs__msg__Odometer__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_argj801_sensors_msgs
bool
argj801_sensors_msgs__msg__Odometer__init(argj801_sensors_msgs__msg__Odometer * msg);

/// Finalize msg/Odometer message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_argj801_sensors_msgs
void
argj801_sensors_msgs__msg__Odometer__fini(argj801_sensors_msgs__msg__Odometer * msg);

/// Create msg/Odometer message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * argj801_sensors_msgs__msg__Odometer__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_argj801_sensors_msgs
argj801_sensors_msgs__msg__Odometer *
argj801_sensors_msgs__msg__Odometer__create();

/// Destroy msg/Odometer message.
/**
 * It calls
 * argj801_sensors_msgs__msg__Odometer__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_argj801_sensors_msgs
void
argj801_sensors_msgs__msg__Odometer__destroy(argj801_sensors_msgs__msg__Odometer * msg);

/// Check for msg/Odometer message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_argj801_sensors_msgs
bool
argj801_sensors_msgs__msg__Odometer__are_equal(const argj801_sensors_msgs__msg__Odometer * lhs, const argj801_sensors_msgs__msg__Odometer * rhs);

/// Copy a msg/Odometer message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_argj801_sensors_msgs
bool
argj801_sensors_msgs__msg__Odometer__copy(
  const argj801_sensors_msgs__msg__Odometer * input,
  argj801_sensors_msgs__msg__Odometer * output);

/// Initialize array of msg/Odometer messages.
/**
 * It allocates the memory for the number of elements and calls
 * argj801_sensors_msgs__msg__Odometer__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_argj801_sensors_msgs
bool
argj801_sensors_msgs__msg__Odometer__Sequence__init(argj801_sensors_msgs__msg__Odometer__Sequence * array, size_t size);

/// Finalize array of msg/Odometer messages.
/**
 * It calls
 * argj801_sensors_msgs__msg__Odometer__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_argj801_sensors_msgs
void
argj801_sensors_msgs__msg__Odometer__Sequence__fini(argj801_sensors_msgs__msg__Odometer__Sequence * array);

/// Create array of msg/Odometer messages.
/**
 * It allocates the memory for the array and calls
 * argj801_sensors_msgs__msg__Odometer__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_argj801_sensors_msgs
argj801_sensors_msgs__msg__Odometer__Sequence *
argj801_sensors_msgs__msg__Odometer__Sequence__create(size_t size);

/// Destroy array of msg/Odometer messages.
/**
 * It calls
 * argj801_sensors_msgs__msg__Odometer__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_argj801_sensors_msgs
void
argj801_sensors_msgs__msg__Odometer__Sequence__destroy(argj801_sensors_msgs__msg__Odometer__Sequence * array);

/// Check for msg/Odometer message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_argj801_sensors_msgs
bool
argj801_sensors_msgs__msg__Odometer__Sequence__are_equal(const argj801_sensors_msgs__msg__Odometer__Sequence * lhs, const argj801_sensors_msgs__msg__Odometer__Sequence * rhs);

/// Copy an array of msg/Odometer messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_argj801_sensors_msgs
bool
argj801_sensors_msgs__msg__Odometer__Sequence__copy(
  const argj801_sensors_msgs__msg__Odometer__Sequence * input,
  argj801_sensors_msgs__msg__Odometer__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ARGJ801_SENSORS_MSGS__MSG__DETAIL__ODOMETER__FUNCTIONS_H_
