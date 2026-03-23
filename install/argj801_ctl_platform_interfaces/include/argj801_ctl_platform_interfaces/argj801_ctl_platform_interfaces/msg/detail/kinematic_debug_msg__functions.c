// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from argj801_ctl_platform_interfaces:msg/KinematicDebugMsg.idl
// generated code does not contain a copyright notice
#include "argj801_ctl_platform_interfaces/msg/detail/kinematic_debug_msg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
argj801_ctl_platform_interfaces__msg__KinematicDebugMsg__init(argj801_ctl_platform_interfaces__msg__KinematicDebugMsg * msg)
{
  if (!msg) {
    return false;
  }
  // throttle
  // steering
  // requested_throttle_acc
  // requested_steering_acc
  // throttle_limited
  // steering_limited
  return true;
}

void
argj801_ctl_platform_interfaces__msg__KinematicDebugMsg__fini(argj801_ctl_platform_interfaces__msg__KinematicDebugMsg * msg)
{
  if (!msg) {
    return;
  }
  // throttle
  // steering
  // requested_throttle_acc
  // requested_steering_acc
  // throttle_limited
  // steering_limited
}

bool
argj801_ctl_platform_interfaces__msg__KinematicDebugMsg__are_equal(const argj801_ctl_platform_interfaces__msg__KinematicDebugMsg * lhs, const argj801_ctl_platform_interfaces__msg__KinematicDebugMsg * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // throttle
  if (lhs->throttle != rhs->throttle) {
    return false;
  }
  // steering
  if (lhs->steering != rhs->steering) {
    return false;
  }
  // requested_throttle_acc
  if (lhs->requested_throttle_acc != rhs->requested_throttle_acc) {
    return false;
  }
  // requested_steering_acc
  if (lhs->requested_steering_acc != rhs->requested_steering_acc) {
    return false;
  }
  // throttle_limited
  if (lhs->throttle_limited != rhs->throttle_limited) {
    return false;
  }
  // steering_limited
  if (lhs->steering_limited != rhs->steering_limited) {
    return false;
  }
  return true;
}

bool
argj801_ctl_platform_interfaces__msg__KinematicDebugMsg__copy(
  const argj801_ctl_platform_interfaces__msg__KinematicDebugMsg * input,
  argj801_ctl_platform_interfaces__msg__KinematicDebugMsg * output)
{
  if (!input || !output) {
    return false;
  }
  // throttle
  output->throttle = input->throttle;
  // steering
  output->steering = input->steering;
  // requested_throttle_acc
  output->requested_throttle_acc = input->requested_throttle_acc;
  // requested_steering_acc
  output->requested_steering_acc = input->requested_steering_acc;
  // throttle_limited
  output->throttle_limited = input->throttle_limited;
  // steering_limited
  output->steering_limited = input->steering_limited;
  return true;
}

argj801_ctl_platform_interfaces__msg__KinematicDebugMsg *
argj801_ctl_platform_interfaces__msg__KinematicDebugMsg__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  argj801_ctl_platform_interfaces__msg__KinematicDebugMsg * msg = (argj801_ctl_platform_interfaces__msg__KinematicDebugMsg *)allocator.allocate(sizeof(argj801_ctl_platform_interfaces__msg__KinematicDebugMsg), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(argj801_ctl_platform_interfaces__msg__KinematicDebugMsg));
  bool success = argj801_ctl_platform_interfaces__msg__KinematicDebugMsg__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
argj801_ctl_platform_interfaces__msg__KinematicDebugMsg__destroy(argj801_ctl_platform_interfaces__msg__KinematicDebugMsg * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    argj801_ctl_platform_interfaces__msg__KinematicDebugMsg__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
argj801_ctl_platform_interfaces__msg__KinematicDebugMsg__Sequence__init(argj801_ctl_platform_interfaces__msg__KinematicDebugMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  argj801_ctl_platform_interfaces__msg__KinematicDebugMsg * data = NULL;

  if (size) {
    data = (argj801_ctl_platform_interfaces__msg__KinematicDebugMsg *)allocator.zero_allocate(size, sizeof(argj801_ctl_platform_interfaces__msg__KinematicDebugMsg), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = argj801_ctl_platform_interfaces__msg__KinematicDebugMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        argj801_ctl_platform_interfaces__msg__KinematicDebugMsg__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
argj801_ctl_platform_interfaces__msg__KinematicDebugMsg__Sequence__fini(argj801_ctl_platform_interfaces__msg__KinematicDebugMsg__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      argj801_ctl_platform_interfaces__msg__KinematicDebugMsg__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

argj801_ctl_platform_interfaces__msg__KinematicDebugMsg__Sequence *
argj801_ctl_platform_interfaces__msg__KinematicDebugMsg__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  argj801_ctl_platform_interfaces__msg__KinematicDebugMsg__Sequence * array = (argj801_ctl_platform_interfaces__msg__KinematicDebugMsg__Sequence *)allocator.allocate(sizeof(argj801_ctl_platform_interfaces__msg__KinematicDebugMsg__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = argj801_ctl_platform_interfaces__msg__KinematicDebugMsg__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
argj801_ctl_platform_interfaces__msg__KinematicDebugMsg__Sequence__destroy(argj801_ctl_platform_interfaces__msg__KinematicDebugMsg__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    argj801_ctl_platform_interfaces__msg__KinematicDebugMsg__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
argj801_ctl_platform_interfaces__msg__KinematicDebugMsg__Sequence__are_equal(const argj801_ctl_platform_interfaces__msg__KinematicDebugMsg__Sequence * lhs, const argj801_ctl_platform_interfaces__msg__KinematicDebugMsg__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!argj801_ctl_platform_interfaces__msg__KinematicDebugMsg__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
argj801_ctl_platform_interfaces__msg__KinematicDebugMsg__Sequence__copy(
  const argj801_ctl_platform_interfaces__msg__KinematicDebugMsg__Sequence * input,
  argj801_ctl_platform_interfaces__msg__KinematicDebugMsg__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(argj801_ctl_platform_interfaces__msg__KinematicDebugMsg);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    argj801_ctl_platform_interfaces__msg__KinematicDebugMsg * data =
      (argj801_ctl_platform_interfaces__msg__KinematicDebugMsg *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!argj801_ctl_platform_interfaces__msg__KinematicDebugMsg__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          argj801_ctl_platform_interfaces__msg__KinematicDebugMsg__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!argj801_ctl_platform_interfaces__msg__KinematicDebugMsg__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
