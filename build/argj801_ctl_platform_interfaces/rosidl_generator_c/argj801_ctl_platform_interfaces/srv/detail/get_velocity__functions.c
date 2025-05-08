// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from argj801_ctl_platform_interfaces:srv/GetVelocity.idl
// generated code does not contain a copyright notice
#include "argj801_ctl_platform_interfaces/srv/detail/get_velocity__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
argj801_ctl_platform_interfaces__srv__GetVelocity_Request__init(argj801_ctl_platform_interfaces__srv__GetVelocity_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
argj801_ctl_platform_interfaces__srv__GetVelocity_Request__fini(argj801_ctl_platform_interfaces__srv__GetVelocity_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
argj801_ctl_platform_interfaces__srv__GetVelocity_Request__are_equal(const argj801_ctl_platform_interfaces__srv__GetVelocity_Request * lhs, const argj801_ctl_platform_interfaces__srv__GetVelocity_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
argj801_ctl_platform_interfaces__srv__GetVelocity_Request__copy(
  const argj801_ctl_platform_interfaces__srv__GetVelocity_Request * input,
  argj801_ctl_platform_interfaces__srv__GetVelocity_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

argj801_ctl_platform_interfaces__srv__GetVelocity_Request *
argj801_ctl_platform_interfaces__srv__GetVelocity_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  argj801_ctl_platform_interfaces__srv__GetVelocity_Request * msg = (argj801_ctl_platform_interfaces__srv__GetVelocity_Request *)allocator.allocate(sizeof(argj801_ctl_platform_interfaces__srv__GetVelocity_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(argj801_ctl_platform_interfaces__srv__GetVelocity_Request));
  bool success = argj801_ctl_platform_interfaces__srv__GetVelocity_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
argj801_ctl_platform_interfaces__srv__GetVelocity_Request__destroy(argj801_ctl_platform_interfaces__srv__GetVelocity_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    argj801_ctl_platform_interfaces__srv__GetVelocity_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
argj801_ctl_platform_interfaces__srv__GetVelocity_Request__Sequence__init(argj801_ctl_platform_interfaces__srv__GetVelocity_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  argj801_ctl_platform_interfaces__srv__GetVelocity_Request * data = NULL;

  if (size) {
    data = (argj801_ctl_platform_interfaces__srv__GetVelocity_Request *)allocator.zero_allocate(size, sizeof(argj801_ctl_platform_interfaces__srv__GetVelocity_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = argj801_ctl_platform_interfaces__srv__GetVelocity_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        argj801_ctl_platform_interfaces__srv__GetVelocity_Request__fini(&data[i - 1]);
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
argj801_ctl_platform_interfaces__srv__GetVelocity_Request__Sequence__fini(argj801_ctl_platform_interfaces__srv__GetVelocity_Request__Sequence * array)
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
      argj801_ctl_platform_interfaces__srv__GetVelocity_Request__fini(&array->data[i]);
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

argj801_ctl_platform_interfaces__srv__GetVelocity_Request__Sequence *
argj801_ctl_platform_interfaces__srv__GetVelocity_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  argj801_ctl_platform_interfaces__srv__GetVelocity_Request__Sequence * array = (argj801_ctl_platform_interfaces__srv__GetVelocity_Request__Sequence *)allocator.allocate(sizeof(argj801_ctl_platform_interfaces__srv__GetVelocity_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = argj801_ctl_platform_interfaces__srv__GetVelocity_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
argj801_ctl_platform_interfaces__srv__GetVelocity_Request__Sequence__destroy(argj801_ctl_platform_interfaces__srv__GetVelocity_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    argj801_ctl_platform_interfaces__srv__GetVelocity_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
argj801_ctl_platform_interfaces__srv__GetVelocity_Request__Sequence__are_equal(const argj801_ctl_platform_interfaces__srv__GetVelocity_Request__Sequence * lhs, const argj801_ctl_platform_interfaces__srv__GetVelocity_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!argj801_ctl_platform_interfaces__srv__GetVelocity_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
argj801_ctl_platform_interfaces__srv__GetVelocity_Request__Sequence__copy(
  const argj801_ctl_platform_interfaces__srv__GetVelocity_Request__Sequence * input,
  argj801_ctl_platform_interfaces__srv__GetVelocity_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(argj801_ctl_platform_interfaces__srv__GetVelocity_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    argj801_ctl_platform_interfaces__srv__GetVelocity_Request * data =
      (argj801_ctl_platform_interfaces__srv__GetVelocity_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!argj801_ctl_platform_interfaces__srv__GetVelocity_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          argj801_ctl_platform_interfaces__srv__GetVelocity_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!argj801_ctl_platform_interfaces__srv__GetVelocity_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
argj801_ctl_platform_interfaces__srv__GetVelocity_Response__init(argj801_ctl_platform_interfaces__srv__GetVelocity_Response * msg)
{
  if (!msg) {
    return false;
  }
  // actual_velocity
  return true;
}

void
argj801_ctl_platform_interfaces__srv__GetVelocity_Response__fini(argj801_ctl_platform_interfaces__srv__GetVelocity_Response * msg)
{
  if (!msg) {
    return;
  }
  // actual_velocity
}

bool
argj801_ctl_platform_interfaces__srv__GetVelocity_Response__are_equal(const argj801_ctl_platform_interfaces__srv__GetVelocity_Response * lhs, const argj801_ctl_platform_interfaces__srv__GetVelocity_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // actual_velocity
  if (lhs->actual_velocity != rhs->actual_velocity) {
    return false;
  }
  return true;
}

bool
argj801_ctl_platform_interfaces__srv__GetVelocity_Response__copy(
  const argj801_ctl_platform_interfaces__srv__GetVelocity_Response * input,
  argj801_ctl_platform_interfaces__srv__GetVelocity_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // actual_velocity
  output->actual_velocity = input->actual_velocity;
  return true;
}

argj801_ctl_platform_interfaces__srv__GetVelocity_Response *
argj801_ctl_platform_interfaces__srv__GetVelocity_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  argj801_ctl_platform_interfaces__srv__GetVelocity_Response * msg = (argj801_ctl_platform_interfaces__srv__GetVelocity_Response *)allocator.allocate(sizeof(argj801_ctl_platform_interfaces__srv__GetVelocity_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(argj801_ctl_platform_interfaces__srv__GetVelocity_Response));
  bool success = argj801_ctl_platform_interfaces__srv__GetVelocity_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
argj801_ctl_platform_interfaces__srv__GetVelocity_Response__destroy(argj801_ctl_platform_interfaces__srv__GetVelocity_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    argj801_ctl_platform_interfaces__srv__GetVelocity_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
argj801_ctl_platform_interfaces__srv__GetVelocity_Response__Sequence__init(argj801_ctl_platform_interfaces__srv__GetVelocity_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  argj801_ctl_platform_interfaces__srv__GetVelocity_Response * data = NULL;

  if (size) {
    data = (argj801_ctl_platform_interfaces__srv__GetVelocity_Response *)allocator.zero_allocate(size, sizeof(argj801_ctl_platform_interfaces__srv__GetVelocity_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = argj801_ctl_platform_interfaces__srv__GetVelocity_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        argj801_ctl_platform_interfaces__srv__GetVelocity_Response__fini(&data[i - 1]);
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
argj801_ctl_platform_interfaces__srv__GetVelocity_Response__Sequence__fini(argj801_ctl_platform_interfaces__srv__GetVelocity_Response__Sequence * array)
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
      argj801_ctl_platform_interfaces__srv__GetVelocity_Response__fini(&array->data[i]);
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

argj801_ctl_platform_interfaces__srv__GetVelocity_Response__Sequence *
argj801_ctl_platform_interfaces__srv__GetVelocity_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  argj801_ctl_platform_interfaces__srv__GetVelocity_Response__Sequence * array = (argj801_ctl_platform_interfaces__srv__GetVelocity_Response__Sequence *)allocator.allocate(sizeof(argj801_ctl_platform_interfaces__srv__GetVelocity_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = argj801_ctl_platform_interfaces__srv__GetVelocity_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
argj801_ctl_platform_interfaces__srv__GetVelocity_Response__Sequence__destroy(argj801_ctl_platform_interfaces__srv__GetVelocity_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    argj801_ctl_platform_interfaces__srv__GetVelocity_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
argj801_ctl_platform_interfaces__srv__GetVelocity_Response__Sequence__are_equal(const argj801_ctl_platform_interfaces__srv__GetVelocity_Response__Sequence * lhs, const argj801_ctl_platform_interfaces__srv__GetVelocity_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!argj801_ctl_platform_interfaces__srv__GetVelocity_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
argj801_ctl_platform_interfaces__srv__GetVelocity_Response__Sequence__copy(
  const argj801_ctl_platform_interfaces__srv__GetVelocity_Response__Sequence * input,
  argj801_ctl_platform_interfaces__srv__GetVelocity_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(argj801_ctl_platform_interfaces__srv__GetVelocity_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    argj801_ctl_platform_interfaces__srv__GetVelocity_Response * data =
      (argj801_ctl_platform_interfaces__srv__GetVelocity_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!argj801_ctl_platform_interfaces__srv__GetVelocity_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          argj801_ctl_platform_interfaces__srv__GetVelocity_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!argj801_ctl_platform_interfaces__srv__GetVelocity_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
