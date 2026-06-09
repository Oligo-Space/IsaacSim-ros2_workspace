// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rosidl_generator_py:msg/StringArrays.idl
// generated code does not contain a copyright notice
#include "rosidl_generator_py/msg/detail/string_arrays__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `ub_string_static_array_value`
// Member `ub_string_ub_array_value`
// Member `ub_string_dynamic_array_value`
// Member `string_dynamic_array_value`
// Member `string_static_array_value`
// Member `string_bounded_array_value`
// Member `def_string_dynamic_array_value`
// Member `def_string_static_array_value`
// Member `def_string_bounded_array_value`
// Member `def_various_quotes`
// Member `def_various_commas`
#include "rosidl_runtime_c/string_functions.h"

bool
rosidl_generator_py__msg__StringArrays__init(rosidl_generator_py__msg__StringArrays * msg)
{
  if (!msg) {
    return false;
  }
  // ub_string_static_array_value
  for (size_t i = 0; i < 3; ++i) {
    if (!rosidl_runtime_c__String__init(&msg->ub_string_static_array_value[i])) {
      rosidl_generator_py__msg__StringArrays__fini(msg);
      return false;
    }
  }
  // ub_string_ub_array_value
  if (!rosidl_runtime_c__String__Sequence__init(&msg->ub_string_ub_array_value, 0)) {
    rosidl_generator_py__msg__StringArrays__fini(msg);
    return false;
  }
  // ub_string_dynamic_array_value
  if (!rosidl_runtime_c__String__Sequence__init(&msg->ub_string_dynamic_array_value, 0)) {
    rosidl_generator_py__msg__StringArrays__fini(msg);
    return false;
  }
  // string_dynamic_array_value
  if (!rosidl_runtime_c__String__Sequence__init(&msg->string_dynamic_array_value, 0)) {
    rosidl_generator_py__msg__StringArrays__fini(msg);
    return false;
  }
  // string_static_array_value
  for (size_t i = 0; i < 3; ++i) {
    if (!rosidl_runtime_c__String__init(&msg->string_static_array_value[i])) {
      rosidl_generator_py__msg__StringArrays__fini(msg);
      return false;
    }
  }
  // string_bounded_array_value
  if (!rosidl_runtime_c__String__Sequence__init(&msg->string_bounded_array_value, 0)) {
    rosidl_generator_py__msg__StringArrays__fini(msg);
    return false;
  }
  // def_string_dynamic_array_value
  {
    bool success = rosidl_runtime_c__String__Sequence__init(&msg->def_string_dynamic_array_value, 5);
    if (!success) {
      goto abort_init_0;
    }
  }
  {
    bool success = rosidl_runtime_c__String__assign(&msg->def_string_dynamic_array_value.data[0], "What");
    if (!success) {
      goto abort_init_1;
    }
  }
  {
    bool success = rosidl_runtime_c__String__assign(&msg->def_string_dynamic_array_value.data[1], "a");
    if (!success) {
      goto abort_init_2;
    }
  }
  {
    bool success = rosidl_runtime_c__String__assign(&msg->def_string_dynamic_array_value.data[2], "wonderful");
    if (!success) {
      goto abort_init_3;
    }
  }
  {
    bool success = rosidl_runtime_c__String__assign(&msg->def_string_dynamic_array_value.data[3], "world");
    if (!success) {
      goto abort_init_4;
    }
  }
  {
    bool success = rosidl_runtime_c__String__assign(&msg->def_string_dynamic_array_value.data[4], "!");
    if (!success) {
      goto abort_init_5;
    }
  }
  // def_string_static_array_value
  for (size_t i = 0; i < 3; ++i) {
    if (!rosidl_runtime_c__String__init(&msg->def_string_static_array_value[i])) {
      rosidl_generator_py__msg__StringArrays__fini(msg);
      return false;
    }
  }
  {
    bool success = rosidl_runtime_c__String__assign(&msg->def_string_static_array_value[0], "Hello");
    if (!success) {
      goto abort_init_6;
    }
  }
  {
    bool success = rosidl_runtime_c__String__assign(&msg->def_string_static_array_value[1], "World");
    if (!success) {
      goto abort_init_7;
    }
  }
  {
    bool success = rosidl_runtime_c__String__assign(&msg->def_string_static_array_value[2], "!");
    if (!success) {
      goto abort_init_8;
    }
  }
  // def_string_bounded_array_value
  {
    bool success = rosidl_runtime_c__String__Sequence__init(&msg->def_string_bounded_array_value, 3);
    if (!success) {
      goto abort_init_9;
    }
  }
  {
    bool success = rosidl_runtime_c__String__assign(&msg->def_string_bounded_array_value.data[0], "Hello");
    if (!success) {
      goto abort_init_10;
    }
  }
  {
    bool success = rosidl_runtime_c__String__assign(&msg->def_string_bounded_array_value.data[1], "World");
    if (!success) {
      goto abort_init_11;
    }
  }
  {
    bool success = rosidl_runtime_c__String__assign(&msg->def_string_bounded_array_value.data[2], "!");
    if (!success) {
      goto abort_init_12;
    }
  }
  // def_various_quotes
  {
    bool success = rosidl_runtime_c__String__Sequence__init(&msg->def_various_quotes, 2);
    if (!success) {
      goto abort_init_13;
    }
  }
  {
    bool success = rosidl_runtime_c__String__assign(&msg->def_various_quotes.data[0], "H\"el'lo");
    if (!success) {
      goto abort_init_14;
    }
  }
  {
    bool success = rosidl_runtime_c__String__assign(&msg->def_various_quotes.data[1], "Wo'r\"ld");
    if (!success) {
      goto abort_init_15;
    }
  }
  // def_various_commas
  {
    bool success = rosidl_runtime_c__String__Sequence__init(&msg->def_various_commas, 4);
    if (!success) {
      goto abort_init_16;
    }
  }
  {
    bool success = rosidl_runtime_c__String__assign(&msg->def_various_commas.data[0], "Hel,lo");
    if (!success) {
      goto abort_init_17;
    }
  }
  {
    bool success = rosidl_runtime_c__String__assign(&msg->def_various_commas.data[1], ",World");
    if (!success) {
      goto abort_init_18;
    }
  }
  {
    bool success = rosidl_runtime_c__String__assign(&msg->def_various_commas.data[2], "abcd");
    if (!success) {
      goto abort_init_19;
    }
  }
  {
    bool success = rosidl_runtime_c__String__assign(&msg->def_various_commas.data[3], "!,");
    if (!success) {
      goto abort_init_20;
    }
  }
  return true;
abort_init_20:
  rosidl_runtime_c__String__fini(&msg->def_various_commas.data[2]);
abort_init_19:
  rosidl_runtime_c__String__fini(&msg->def_various_commas.data[1]);
abort_init_18:
  rosidl_runtime_c__String__fini(&msg->def_various_commas.data[0]);
abort_init_17:
  rosidl_runtime_c__String__Sequence__fini(&msg->def_various_commas);
abort_init_16:
  rosidl_runtime_c__String__fini(&msg->def_various_quotes.data[1]);
abort_init_15:
  rosidl_runtime_c__String__fini(&msg->def_various_quotes.data[0]);
abort_init_14:
  rosidl_runtime_c__String__Sequence__fini(&msg->def_various_quotes);
abort_init_13:
  rosidl_runtime_c__String__fini(&msg->def_string_bounded_array_value.data[2]);
abort_init_12:
  rosidl_runtime_c__String__fini(&msg->def_string_bounded_array_value.data[1]);
abort_init_11:
  rosidl_runtime_c__String__fini(&msg->def_string_bounded_array_value.data[0]);
abort_init_10:
  rosidl_runtime_c__String__Sequence__fini(&msg->def_string_bounded_array_value);
abort_init_9:
  rosidl_runtime_c__String__fini(&msg->def_string_static_array_value[2]);
abort_init_8:
  rosidl_runtime_c__String__fini(&msg->def_string_static_array_value[1]);
abort_init_7:
  rosidl_runtime_c__String__fini(&msg->def_string_static_array_value[0]);
abort_init_6:
  rosidl_runtime_c__String__fini(&msg->def_string_dynamic_array_value.data[4]);
abort_init_5:
  rosidl_runtime_c__String__fini(&msg->def_string_dynamic_array_value.data[3]);
abort_init_4:
  rosidl_runtime_c__String__fini(&msg->def_string_dynamic_array_value.data[2]);
abort_init_3:
  rosidl_runtime_c__String__fini(&msg->def_string_dynamic_array_value.data[1]);
abort_init_2:
  rosidl_runtime_c__String__fini(&msg->def_string_dynamic_array_value.data[0]);
abort_init_1:
  rosidl_runtime_c__String__Sequence__fini(&msg->def_string_dynamic_array_value);
abort_init_0:
  return false;
}

void
rosidl_generator_py__msg__StringArrays__fini(rosidl_generator_py__msg__StringArrays * msg)
{
  if (!msg) {
    return;
  }
  // ub_string_static_array_value
  for (size_t i = 0; i < 3; ++i) {
    rosidl_runtime_c__String__fini(&msg->ub_string_static_array_value[i]);
  }
  // ub_string_ub_array_value
  rosidl_runtime_c__String__Sequence__fini(&msg->ub_string_ub_array_value);
  // ub_string_dynamic_array_value
  rosidl_runtime_c__String__Sequence__fini(&msg->ub_string_dynamic_array_value);
  // string_dynamic_array_value
  rosidl_runtime_c__String__Sequence__fini(&msg->string_dynamic_array_value);
  // string_static_array_value
  for (size_t i = 0; i < 3; ++i) {
    rosidl_runtime_c__String__fini(&msg->string_static_array_value[i]);
  }
  // string_bounded_array_value
  rosidl_runtime_c__String__Sequence__fini(&msg->string_bounded_array_value);
  // def_string_dynamic_array_value
  rosidl_runtime_c__String__Sequence__fini(&msg->def_string_dynamic_array_value);
  // def_string_static_array_value
  for (size_t i = 0; i < 3; ++i) {
    rosidl_runtime_c__String__fini(&msg->def_string_static_array_value[i]);
  }
  // def_string_bounded_array_value
  rosidl_runtime_c__String__Sequence__fini(&msg->def_string_bounded_array_value);
  // def_various_quotes
  rosidl_runtime_c__String__Sequence__fini(&msg->def_various_quotes);
  // def_various_commas
  rosidl_runtime_c__String__Sequence__fini(&msg->def_various_commas);
}

bool
rosidl_generator_py__msg__StringArrays__are_equal(const rosidl_generator_py__msg__StringArrays * lhs, const rosidl_generator_py__msg__StringArrays * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // ub_string_static_array_value
  for (size_t i = 0; i < 3; ++i) {
    if (!rosidl_runtime_c__String__are_equal(
        &(lhs->ub_string_static_array_value[i]), &(rhs->ub_string_static_array_value[i])))
    {
      return false;
    }
  }
  // ub_string_ub_array_value
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->ub_string_ub_array_value), &(rhs->ub_string_ub_array_value)))
  {
    return false;
  }
  // ub_string_dynamic_array_value
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->ub_string_dynamic_array_value), &(rhs->ub_string_dynamic_array_value)))
  {
    return false;
  }
  // string_dynamic_array_value
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->string_dynamic_array_value), &(rhs->string_dynamic_array_value)))
  {
    return false;
  }
  // string_static_array_value
  for (size_t i = 0; i < 3; ++i) {
    if (!rosidl_runtime_c__String__are_equal(
        &(lhs->string_static_array_value[i]), &(rhs->string_static_array_value[i])))
    {
      return false;
    }
  }
  // string_bounded_array_value
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->string_bounded_array_value), &(rhs->string_bounded_array_value)))
  {
    return false;
  }
  // def_string_dynamic_array_value
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->def_string_dynamic_array_value), &(rhs->def_string_dynamic_array_value)))
  {
    return false;
  }
  // def_string_static_array_value
  for (size_t i = 0; i < 3; ++i) {
    if (!rosidl_runtime_c__String__are_equal(
        &(lhs->def_string_static_array_value[i]), &(rhs->def_string_static_array_value[i])))
    {
      return false;
    }
  }
  // def_string_bounded_array_value
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->def_string_bounded_array_value), &(rhs->def_string_bounded_array_value)))
  {
    return false;
  }
  // def_various_quotes
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->def_various_quotes), &(rhs->def_various_quotes)))
  {
    return false;
  }
  // def_various_commas
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->def_various_commas), &(rhs->def_various_commas)))
  {
    return false;
  }
  return true;
}

bool
rosidl_generator_py__msg__StringArrays__copy(
  const rosidl_generator_py__msg__StringArrays * input,
  rosidl_generator_py__msg__StringArrays * output)
{
  if (!input || !output) {
    return false;
  }
  // ub_string_static_array_value
  for (size_t i = 0; i < 3; ++i) {
    if (!rosidl_runtime_c__String__copy(
        &(input->ub_string_static_array_value[i]), &(output->ub_string_static_array_value[i])))
    {
      return false;
    }
  }
  // ub_string_ub_array_value
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->ub_string_ub_array_value), &(output->ub_string_ub_array_value)))
  {
    return false;
  }
  // ub_string_dynamic_array_value
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->ub_string_dynamic_array_value), &(output->ub_string_dynamic_array_value)))
  {
    return false;
  }
  // string_dynamic_array_value
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->string_dynamic_array_value), &(output->string_dynamic_array_value)))
  {
    return false;
  }
  // string_static_array_value
  for (size_t i = 0; i < 3; ++i) {
    if (!rosidl_runtime_c__String__copy(
        &(input->string_static_array_value[i]), &(output->string_static_array_value[i])))
    {
      return false;
    }
  }
  // string_bounded_array_value
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->string_bounded_array_value), &(output->string_bounded_array_value)))
  {
    return false;
  }
  // def_string_dynamic_array_value
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->def_string_dynamic_array_value), &(output->def_string_dynamic_array_value)))
  {
    return false;
  }
  // def_string_static_array_value
  for (size_t i = 0; i < 3; ++i) {
    if (!rosidl_runtime_c__String__copy(
        &(input->def_string_static_array_value[i]), &(output->def_string_static_array_value[i])))
    {
      return false;
    }
  }
  // def_string_bounded_array_value
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->def_string_bounded_array_value), &(output->def_string_bounded_array_value)))
  {
    return false;
  }
  // def_various_quotes
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->def_various_quotes), &(output->def_various_quotes)))
  {
    return false;
  }
  // def_various_commas
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->def_various_commas), &(output->def_various_commas)))
  {
    return false;
  }
  return true;
}

rosidl_generator_py__msg__StringArrays *
rosidl_generator_py__msg__StringArrays__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosidl_generator_py__msg__StringArrays * msg = (rosidl_generator_py__msg__StringArrays *)allocator.allocate(sizeof(rosidl_generator_py__msg__StringArrays), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rosidl_generator_py__msg__StringArrays));
  bool success = rosidl_generator_py__msg__StringArrays__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rosidl_generator_py__msg__StringArrays__destroy(rosidl_generator_py__msg__StringArrays * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rosidl_generator_py__msg__StringArrays__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rosidl_generator_py__msg__StringArrays__Sequence__init(rosidl_generator_py__msg__StringArrays__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosidl_generator_py__msg__StringArrays * data = NULL;

  if (size) {
    data = (rosidl_generator_py__msg__StringArrays *)allocator.zero_allocate(size, sizeof(rosidl_generator_py__msg__StringArrays), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rosidl_generator_py__msg__StringArrays__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rosidl_generator_py__msg__StringArrays__fini(&data[i - 1]);
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
rosidl_generator_py__msg__StringArrays__Sequence__fini(rosidl_generator_py__msg__StringArrays__Sequence * array)
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
      rosidl_generator_py__msg__StringArrays__fini(&array->data[i]);
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

rosidl_generator_py__msg__StringArrays__Sequence *
rosidl_generator_py__msg__StringArrays__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rosidl_generator_py__msg__StringArrays__Sequence * array = (rosidl_generator_py__msg__StringArrays__Sequence *)allocator.allocate(sizeof(rosidl_generator_py__msg__StringArrays__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rosidl_generator_py__msg__StringArrays__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rosidl_generator_py__msg__StringArrays__Sequence__destroy(rosidl_generator_py__msg__StringArrays__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rosidl_generator_py__msg__StringArrays__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rosidl_generator_py__msg__StringArrays__Sequence__are_equal(const rosidl_generator_py__msg__StringArrays__Sequence * lhs, const rosidl_generator_py__msg__StringArrays__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rosidl_generator_py__msg__StringArrays__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rosidl_generator_py__msg__StringArrays__Sequence__copy(
  const rosidl_generator_py__msg__StringArrays__Sequence * input,
  rosidl_generator_py__msg__StringArrays__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rosidl_generator_py__msg__StringArrays);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rosidl_generator_py__msg__StringArrays * data =
      (rosidl_generator_py__msg__StringArrays *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rosidl_generator_py__msg__StringArrays__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rosidl_generator_py__msg__StringArrays__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rosidl_generator_py__msg__StringArrays__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
