// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from argj801_ctl_platform_interfaces:srv/EmergencyStop.idl
// generated code does not contain a copyright notice

#ifndef ARGJ801_CTL_PLATFORM_INTERFACES__SRV__DETAIL__EMERGENCY_STOP__FUNCTIONS_H_
#define ARGJ801_CTL_PLATFORM_INTERFACES__SRV__DETAIL__EMERGENCY_STOP__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "argj801_ctl_platform_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "argj801_ctl_platform_interfaces/srv/detail/emergency_stop__struct.h"

/// Initialize srv/EmergencyStop message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * argj801_ctl_platform_interfaces__srv__EmergencyStop_Request
 * )) before or use
 * argj801_ctl_platform_interfaces__srv__EmergencyStop_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_argj801_ctl_platform_interfaces
bool
argj801_ctl_platform_interfaces__srv__EmergencyStop_Request__init(argj801_ctl_platform_interfaces__srv__EmergencyStop_Request * msg);

/// Finalize srv/EmergencyStop message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_argj801_ctl_platform_interfaces
void
argj801_ctl_platform_interfaces__srv__EmergencyStop_Request__fini(argj801_ctl_platform_interfaces__srv__EmergencyStop_Request * msg);

/// Create srv/EmergencyStop message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * argj801_ctl_platform_interfaces__srv__EmergencyStop_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_argj801_ctl_platform_interfaces
argj801_ctl_platform_interfaces__srv__EmergencyStop_Request *
argj801_ctl_platform_interfaces__srv__EmergencyStop_Request__create();

/// Destroy srv/EmergencyStop message.
/**
 * It calls
 * argj801_ctl_platform_interfaces__srv__EmergencyStop_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_argj801_ctl_platform_interfaces
void
argj801_ctl_platform_interfaces__srv__EmergencyStop_Request__destroy(argj801_ctl_platform_interfaces__srv__EmergencyStop_Request * msg);

/// Check for srv/EmergencyStop message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_argj801_ctl_platform_interfaces
bool
argj801_ctl_platform_interfaces__srv__EmergencyStop_Request__are_equal(const argj801_ctl_platform_interfaces__srv__EmergencyStop_Request * lhs, const argj801_ctl_platform_interfaces__srv__EmergencyStop_Request * rhs);

/// Copy a srv/EmergencyStop message.
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
ROSIDL_GENERATOR_C_PUBLIC_argj801_ctl_platform_interfaces
bool
argj801_ctl_platform_interfaces__srv__EmergencyStop_Request__copy(
  const argj801_ctl_platform_interfaces__srv__EmergencyStop_Request * input,
  argj801_ctl_platform_interfaces__srv__EmergencyStop_Request * output);

/// Initialize array of srv/EmergencyStop messages.
/**
 * It allocates the memory for the number of elements and calls
 * argj801_ctl_platform_interfaces__srv__EmergencyStop_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_argj801_ctl_platform_interfaces
bool
argj801_ctl_platform_interfaces__srv__EmergencyStop_Request__Sequence__init(argj801_ctl_platform_interfaces__srv__EmergencyStop_Request__Sequence * array, size_t size);

/// Finalize array of srv/EmergencyStop messages.
/**
 * It calls
 * argj801_ctl_platform_interfaces__srv__EmergencyStop_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_argj801_ctl_platform_interfaces
void
argj801_ctl_platform_interfaces__srv__EmergencyStop_Request__Sequence__fini(argj801_ctl_platform_interfaces__srv__EmergencyStop_Request__Sequence * array);

/// Create array of srv/EmergencyStop messages.
/**
 * It allocates the memory for the array and calls
 * argj801_ctl_platform_interfaces__srv__EmergencyStop_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_argj801_ctl_platform_interfaces
argj801_ctl_platform_interfaces__srv__EmergencyStop_Request__Sequence *
argj801_ctl_platform_interfaces__srv__EmergencyStop_Request__Sequence__create(size_t size);

/// Destroy array of srv/EmergencyStop messages.
/**
 * It calls
 * argj801_ctl_platform_interfaces__srv__EmergencyStop_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_argj801_ctl_platform_interfaces
void
argj801_ctl_platform_interfaces__srv__EmergencyStop_Request__Sequence__destroy(argj801_ctl_platform_interfaces__srv__EmergencyStop_Request__Sequence * array);

/// Check for srv/EmergencyStop message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_argj801_ctl_platform_interfaces
bool
argj801_ctl_platform_interfaces__srv__EmergencyStop_Request__Sequence__are_equal(const argj801_ctl_platform_interfaces__srv__EmergencyStop_Request__Sequence * lhs, const argj801_ctl_platform_interfaces__srv__EmergencyStop_Request__Sequence * rhs);

/// Copy an array of srv/EmergencyStop messages.
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
ROSIDL_GENERATOR_C_PUBLIC_argj801_ctl_platform_interfaces
bool
argj801_ctl_platform_interfaces__srv__EmergencyStop_Request__Sequence__copy(
  const argj801_ctl_platform_interfaces__srv__EmergencyStop_Request__Sequence * input,
  argj801_ctl_platform_interfaces__srv__EmergencyStop_Request__Sequence * output);

/// Initialize srv/EmergencyStop message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * argj801_ctl_platform_interfaces__srv__EmergencyStop_Response
 * )) before or use
 * argj801_ctl_platform_interfaces__srv__EmergencyStop_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_argj801_ctl_platform_interfaces
bool
argj801_ctl_platform_interfaces__srv__EmergencyStop_Response__init(argj801_ctl_platform_interfaces__srv__EmergencyStop_Response * msg);

/// Finalize srv/EmergencyStop message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_argj801_ctl_platform_interfaces
void
argj801_ctl_platform_interfaces__srv__EmergencyStop_Response__fini(argj801_ctl_platform_interfaces__srv__EmergencyStop_Response * msg);

/// Create srv/EmergencyStop message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * argj801_ctl_platform_interfaces__srv__EmergencyStop_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_argj801_ctl_platform_interfaces
argj801_ctl_platform_interfaces__srv__EmergencyStop_Response *
argj801_ctl_platform_interfaces__srv__EmergencyStop_Response__create();

/// Destroy srv/EmergencyStop message.
/**
 * It calls
 * argj801_ctl_platform_interfaces__srv__EmergencyStop_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_argj801_ctl_platform_interfaces
void
argj801_ctl_platform_interfaces__srv__EmergencyStop_Response__destroy(argj801_ctl_platform_interfaces__srv__EmergencyStop_Response * msg);

/// Check for srv/EmergencyStop message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_argj801_ctl_platform_interfaces
bool
argj801_ctl_platform_interfaces__srv__EmergencyStop_Response__are_equal(const argj801_ctl_platform_interfaces__srv__EmergencyStop_Response * lhs, const argj801_ctl_platform_interfaces__srv__EmergencyStop_Response * rhs);

/// Copy a srv/EmergencyStop message.
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
ROSIDL_GENERATOR_C_PUBLIC_argj801_ctl_platform_interfaces
bool
argj801_ctl_platform_interfaces__srv__EmergencyStop_Response__copy(
  const argj801_ctl_platform_interfaces__srv__EmergencyStop_Response * input,
  argj801_ctl_platform_interfaces__srv__EmergencyStop_Response * output);

/// Initialize array of srv/EmergencyStop messages.
/**
 * It allocates the memory for the number of elements and calls
 * argj801_ctl_platform_interfaces__srv__EmergencyStop_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_argj801_ctl_platform_interfaces
bool
argj801_ctl_platform_interfaces__srv__EmergencyStop_Response__Sequence__init(argj801_ctl_platform_interfaces__srv__EmergencyStop_Response__Sequence * array, size_t size);

/// Finalize array of srv/EmergencyStop messages.
/**
 * It calls
 * argj801_ctl_platform_interfaces__srv__EmergencyStop_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_argj801_ctl_platform_interfaces
void
argj801_ctl_platform_interfaces__srv__EmergencyStop_Response__Sequence__fini(argj801_ctl_platform_interfaces__srv__EmergencyStop_Response__Sequence * array);

/// Create array of srv/EmergencyStop messages.
/**
 * It allocates the memory for the array and calls
 * argj801_ctl_platform_interfaces__srv__EmergencyStop_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_argj801_ctl_platform_interfaces
argj801_ctl_platform_interfaces__srv__EmergencyStop_Response__Sequence *
argj801_ctl_platform_interfaces__srv__EmergencyStop_Response__Sequence__create(size_t size);

/// Destroy array of srv/EmergencyStop messages.
/**
 * It calls
 * argj801_ctl_platform_interfaces__srv__EmergencyStop_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_argj801_ctl_platform_interfaces
void
argj801_ctl_platform_interfaces__srv__EmergencyStop_Response__Sequence__destroy(argj801_ctl_platform_interfaces__srv__EmergencyStop_Response__Sequence * array);

/// Check for srv/EmergencyStop message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_argj801_ctl_platform_interfaces
bool
argj801_ctl_platform_interfaces__srv__EmergencyStop_Response__Sequence__are_equal(const argj801_ctl_platform_interfaces__srv__EmergencyStop_Response__Sequence * lhs, const argj801_ctl_platform_interfaces__srv__EmergencyStop_Response__Sequence * rhs);

/// Copy an array of srv/EmergencyStop messages.
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
ROSIDL_GENERATOR_C_PUBLIC_argj801_ctl_platform_interfaces
bool
argj801_ctl_platform_interfaces__srv__EmergencyStop_Response__Sequence__copy(
  const argj801_ctl_platform_interfaces__srv__EmergencyStop_Response__Sequence * input,
  argj801_ctl_platform_interfaces__srv__EmergencyStop_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ARGJ801_CTL_PLATFORM_INTERFACES__SRV__DETAIL__EMERGENCY_STOP__FUNCTIONS_H_
