// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rosidl_generator_py:msg/BuiltinTypeSequencesIdl.idl
// generated code does not contain a copyright notice
#include "rosidl_generator_py/msg/detail/builtin_type_sequences_idl__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `char_sequence_unbounded`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
rosidl_generator_py__msg__BuiltinTypeSequencesIdl__init(rosidl_generator_py__msg__BuiltinTypeSequencesIdl * msg)
{
  if (!msg) {
    return false;
  }
  // char_sequence_unbounded
  if (!rosidl_runtime_c__char__Sequence__init(&msg->char_sequence_unbounded, 0)) {
    rosidl_generator_py__msg__BuiltinTypeSequencesIdl__fini(msg);
    return false;
  }
  return true;
}

void
rosidl_generator_py__msg__BuiltinTypeSequencesIdl__fini(rosidl_generator_py__msg__BuiltinTypeSequencesIdl * msg)
{
  if (!msg) {
    return;
  }
  // char_sequence_unbounded
  rosidl_runtime_c__char__Sequence__fini(&msg->char_sequence_unbounded);
}

bool
rosidl_generator_py__msg__BuiltinTypeSequencesIdl__are_equal(const rosidl_generator_py__msg__BuiltinTypeSequencesIdl * lhs, const rosidl_generator_py__msg__BuiltinTypeSequencesIdl * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // char_sequence_unbounded
  if (!rosidl_runtime_c__char__Sequence__are_equal(
      &(lhs->char_sequence_unbounded), &(rhs->char_sequence_unbounded)))
  {
    return false;
  }
  return true;
}

bool
rosidl_generator_py__msg__BuiltinTypeSequencesIdl__copy(
  const rosidl_generator_py__msg__BuiltinTypeSequencesIdl * input,
  rosidl_generator_py__msg__BuiltinTypeSequencesIdl * output)
{
  if (!input || !output) {
    return false;
  }
  // char_sequence_unbounded
  if (!rosidl_runtime_c__char__Sequence__copy(
      &(input->char_sequence_unbounded), &(output->char_sequence_unbounded)))
  {
    return false;
  }
  return true;
}

rosidl_generator_py__msg__BuiltinTypeSequencesIdl *
rosidl_generator_py__msg__BuiltinTypeSequencesIdl__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosidl_generator_py__msg__BuiltinTypeSequencesIdl * msg = (rosidl_generator_py__msg__BuiltinTypeSequencesIdl *)allocator.allocate(sizeof(rosidl_generator_py__msg__BuiltinTypeSequencesIdl), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rosidl_generator_py__msg__BuiltinTypeSequencesIdl));
  bool success = rosidl_generator_py__msg__BuiltinTypeSequencesIdl__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rosidl_generator_py__msg__BuiltinTypeSequencesIdl__destroy(rosidl_generator_py__msg__BuiltinTypeSequencesIdl * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rosidl_generator_py__msg__BuiltinTypeSequencesIdl__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rosidl_generator_py__msg__BuiltinTypeSequencesIdl__Sequence__init(rosidl_generator_py__msg__BuiltinTypeSequencesIdl__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosidl_generator_py__msg__BuiltinTypeSequencesIdl * data = NULL;

  if (size) {
    data = (rosidl_generator_py__msg__BuiltinTypeSequencesIdl *)allocator.zero_allocate(size, sizeof(rosidl_generator_py__msg__BuiltinTypeSequencesIdl), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rosidl_generator_py__msg__BuiltinTypeSequencesIdl__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rosidl_generator_py__msg__BuiltinTypeSequencesIdl__fini(&data[i - 1]);
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
rosidl_generator_py__msg__BuiltinTypeSequencesIdl__Sequence__fini(rosidl_generator_py__msg__BuiltinTypeSequencesIdl__Sequence * array)
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
      rosidl_generator_py__msg__BuiltinTypeSequencesIdl__fini(&array->data[i]);
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

rosidl_generator_py__msg__BuiltinTypeSequencesIdl__Sequence *
rosidl_generator_py__msg__BuiltinTypeSequencesIdl__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosidl_generator_py__msg__BuiltinTypeSequencesIdl__Sequence * array = (rosidl_generator_py__msg__BuiltinTypeSequencesIdl__Sequence *)allocator.allocate(sizeof(rosidl_generator_py__msg__BuiltinTypeSequencesIdl__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rosidl_generator_py__msg__BuiltinTypeSequencesIdl__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rosidl_generator_py__msg__BuiltinTypeSequencesIdl__Sequence__destroy(rosidl_generator_py__msg__BuiltinTypeSequencesIdl__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rosidl_generator_py__msg__BuiltinTypeSequencesIdl__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rosidl_generator_py__msg__BuiltinTypeSequencesIdl__Sequence__are_equal(const rosidl_generator_py__msg__BuiltinTypeSequencesIdl__Sequence * lhs, const rosidl_generator_py__msg__BuiltinTypeSequencesIdl__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rosidl_generator_py__msg__BuiltinTypeSequencesIdl__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rosidl_generator_py__msg__BuiltinTypeSequencesIdl__Sequence__copy(
  const rosidl_generator_py__msg__BuiltinTypeSequencesIdl__Sequence * input,
  rosidl_generator_py__msg__BuiltinTypeSequencesIdl__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rosidl_generator_py__msg__BuiltinTypeSequencesIdl);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rosidl_generator_py__msg__BuiltinTypeSequencesIdl * data =
      (rosidl_generator_py__msg__BuiltinTypeSequencesIdl *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rosidl_generator_py__msg__BuiltinTypeSequencesIdl__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rosidl_generator_py__msg__BuiltinTypeSequencesIdl__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rosidl_generator_py__msg__BuiltinTypeSequencesIdl__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
