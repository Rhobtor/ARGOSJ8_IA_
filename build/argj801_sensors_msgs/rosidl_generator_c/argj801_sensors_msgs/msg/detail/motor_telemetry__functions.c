// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from argj801_sensors_msgs:msg/MotorTelemetry.idl
// generated code does not contain a copyright notice
#include "argj801_sensors_msgs/msg/detail/motor_telemetry__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
argj801_sensors_msgs__msg__MotorTelemetry__init(argj801_sensors_msgs__msg__MotorTelemetry * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    argj801_sensors_msgs__msg__MotorTelemetry__fini(msg);
    return false;
  }
  // endocer
  // rpm
  return true;
}

void
argj801_sensors_msgs__msg__MotorTelemetry__fini(argj801_sensors_msgs__msg__MotorTelemetry * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // endocer
  // rpm
}

bool
argj801_sensors_msgs__msg__MotorTelemetry__are_equal(const argj801_sensors_msgs__msg__MotorTelemetry * lhs, const argj801_sensors_msgs__msg__MotorTelemetry * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // endocer
  if (lhs->endocer != rhs->endocer) {
    return false;
  }
  // rpm
  if (lhs->rpm != rhs->rpm) {
    return false;
  }
  return true;
}

bool
argj801_sensors_msgs__msg__MotorTelemetry__copy(
  const argj801_sensors_msgs__msg__MotorTelemetry * input,
  argj801_sensors_msgs__msg__MotorTelemetry * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // endocer
  output->endocer = input->endocer;
  // rpm
  output->rpm = input->rpm;
  return true;
}

argj801_sensors_msgs__msg__MotorTelemetry *
argj801_sensors_msgs__msg__MotorTelemetry__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  argj801_sensors_msgs__msg__MotorTelemetry * msg = (argj801_sensors_msgs__msg__MotorTelemetry *)allocator.allocate(sizeof(argj801_sensors_msgs__msg__MotorTelemetry), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(argj801_sensors_msgs__msg__MotorTelemetry));
  bool success = argj801_sensors_msgs__msg__MotorTelemetry__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
argj801_sensors_msgs__msg__MotorTelemetry__destroy(argj801_sensors_msgs__msg__MotorTelemetry * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    argj801_sensors_msgs__msg__MotorTelemetry__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
argj801_sensors_msgs__msg__MotorTelemetry__Sequence__init(argj801_sensors_msgs__msg__MotorTelemetry__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  argj801_sensors_msgs__msg__MotorTelemetry * data = NULL;

  if (size) {
    data = (argj801_sensors_msgs__msg__MotorTelemetry *)allocator.zero_allocate(size, sizeof(argj801_sensors_msgs__msg__MotorTelemetry), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = argj801_sensors_msgs__msg__MotorTelemetry__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        argj801_sensors_msgs__msg__MotorTelemetry__fini(&data[i - 1]);
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
argj801_sensors_msgs__msg__MotorTelemetry__Sequence__fini(argj801_sensors_msgs__msg__MotorTelemetry__Sequence * array)
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
      argj801_sensors_msgs__msg__MotorTelemetry__fini(&array->data[i]);
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

argj801_sensors_msgs__msg__MotorTelemetry__Sequence *
argj801_sensors_msgs__msg__MotorTelemetry__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  argj801_sensors_msgs__msg__MotorTelemetry__Sequence * array = (argj801_sensors_msgs__msg__MotorTelemetry__Sequence *)allocator.allocate(sizeof(argj801_sensors_msgs__msg__MotorTelemetry__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = argj801_sensors_msgs__msg__MotorTelemetry__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
argj801_sensors_msgs__msg__MotorTelemetry__Sequence__destroy(argj801_sensors_msgs__msg__MotorTelemetry__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    argj801_sensors_msgs__msg__MotorTelemetry__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
argj801_sensors_msgs__msg__MotorTelemetry__Sequence__are_equal(const argj801_sensors_msgs__msg__MotorTelemetry__Sequence * lhs, const argj801_sensors_msgs__msg__MotorTelemetry__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!argj801_sensors_msgs__msg__MotorTelemetry__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
argj801_sensors_msgs__msg__MotorTelemetry__Sequence__copy(
  const argj801_sensors_msgs__msg__MotorTelemetry__Sequence * input,
  argj801_sensors_msgs__msg__MotorTelemetry__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(argj801_sensors_msgs__msg__MotorTelemetry);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    argj801_sensors_msgs__msg__MotorTelemetry * data =
      (argj801_sensors_msgs__msg__MotorTelemetry *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!argj801_sensors_msgs__msg__MotorTelemetry__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          argj801_sensors_msgs__msg__MotorTelemetry__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!argj801_sensors_msgs__msg__MotorTelemetry__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
