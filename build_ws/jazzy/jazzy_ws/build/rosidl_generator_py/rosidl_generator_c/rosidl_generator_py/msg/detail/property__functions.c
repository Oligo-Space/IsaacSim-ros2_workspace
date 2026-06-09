// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rosidl_generator_py:msg/Property.idl
// generated code does not contain a copyright notice
#include "rosidl_generator_py/msg/detail/property__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `property`
// Member `anything`
#include "rosidl_runtime_c/string_functions.h"

bool
rosidl_generator_py__msg__Property__init(rosidl_generator_py__msg__Property * msg)
{
  if (!msg) {
    return false;
  }
  // property
  if (!rosidl_runtime_c__String__init(&msg->property)) {
    rosidl_generator_py__msg__Property__fini(msg);
    return false;
  }
  // anything
  if (!rosidl_runtime_c__String__init(&msg->anything)) {
    rosidl_generator_py__msg__Property__fini(msg);
    return false;
  }
  return true;
}

void
rosidl_generator_py__msg__Property__fini(rosidl_generator_py__msg__Property * msg)
{
  if (!msg) {
    return;
  }
  // property
  rosidl_runtime_c__String__fini(&msg->property);
  // anything
  rosidl_runtime_c__String__fini(&msg->anything);
}

bool
rosidl_generator_py__msg__Property__are_equal(const rosidl_generator_py__msg__Property * lhs, const rosidl_generator_py__msg__Property * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // property
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->property), &(rhs->property)))
  {
    return false;
  }
  // anything
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->anything), &(rhs->anything)))
  {
    return false;
  }
  return true;
}

bool
rosidl_generator_py__msg__Property__copy(
  const rosidl_generator_py__msg__Property * input,
  rosidl_generator_py__msg__Property * output)
{
  if (!input || !output) {
    return false;
  }
  // property
  if (!rosidl_runtime_c__String__copy(
      &(input->property), &(output->property)))
  {
    return false;
  }
  // anything
  if (!rosidl_runtime_c__String__copy(
      &(input->anything), &(output->anything)))
  {
    return false;
  }
  return true;
}

rosidl_generator_py__msg__Property *
rosidl_generator_py__msg__Property__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosidl_generator_py__msg__Property * msg = (rosidl_generator_py__msg__Property *)allocator.allocate(sizeof(rosidl_generator_py__msg__Property), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rosidl_generator_py__msg__Property));
  bool success = rosidl_generator_py__msg__Property__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rosidl_generator_py__msg__Property__destroy(rosidl_generator_py__msg__Property * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rosidl_generator_py__msg__Property__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rosidl_generator_py__msg__Property__Sequence__init(rosidl_generator_py__msg__Property__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosidl_generator_py__msg__Property * data = NULL;

  if (size) {
    data = (rosidl_generator_py__msg__Property *)allocator.zero_allocate(size, sizeof(rosidl_generator_py__msg__Property), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rosidl_generator_py__msg__Property__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rosidl_generator_py__msg__Property__fini(&data[i - 1]);
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
rosidl_generator_py__msg__Property__Sequence__fini(rosidl_generator_py__msg__Property__Sequence * array)
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
      rosidl_generator_py__msg__Property__fini(&array->data[i]);
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

rosidl_generator_py__msg__Property__Sequence *
rosidl_generator_py__msg__Property__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosidl_generator_py__msg__Property__Sequence * array = (rosidl_generator_py__msg__Property__Sequence *)allocator.allocate(sizeof(rosidl_generator_py__msg__Property__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rosidl_generator_py__msg__Property__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rosidl_generator_py__msg__Property__Sequence__destroy(rosidl_generator_py__msg__Property__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rosidl_generator_py__msg__Property__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rosidl_generator_py__msg__Property__Sequence__are_equal(const rosidl_generator_py__msg__Property__Sequence * lhs, const rosidl_generator_py__msg__Property__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rosidl_generator_py__msg__Property__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rosidl_generator_py__msg__Property__Sequence__copy(
  const rosidl_generator_py__msg__Property__Sequence * input,
  rosidl_generator_py__msg__Property__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rosidl_generator_py__msg__Property);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rosidl_generator_py__msg__Property * data =
      (rosidl_generator_py__msg__Property *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rosidl_generator_py__msg__Property__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rosidl_generator_py__msg__Property__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rosidl_generator_py__msg__Property__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
