// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rosidl_generator_py:msg/MultiNested.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rosidl_generator_py/msg/detail/multi_nested__rosidl_typesupport_introspection_c.h"
#include "rosidl_generator_py/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rosidl_generator_py/msg/detail/multi_nested__functions.h"
#include "rosidl_generator_py/msg/detail/multi_nested__struct.h"


// Include directives for member types
// Member `array_of_arrays`
// Member `bounded_sequence_of_arrays`
// Member `unbounded_sequence_of_arrays`
#include "rosidl_generator_py/msg/arrays.h"
// Member `array_of_arrays`
// Member `bounded_sequence_of_arrays`
// Member `unbounded_sequence_of_arrays`
#include "rosidl_generator_py/msg/detail/arrays__rosidl_typesupport_introspection_c.h"
// Member `array_of_bounded_sequences`
// Member `bounded_sequence_of_bounded_sequences`
// Member `unbounded_sequence_of_bounded_sequences`
#include "rosidl_generator_py/msg/bounded_sequences.h"
// Member `array_of_bounded_sequences`
// Member `bounded_sequence_of_bounded_sequences`
// Member `unbounded_sequence_of_bounded_sequences`
#include "rosidl_generator_py/msg/detail/bounded_sequences__rosidl_typesupport_introspection_c.h"
// Member `array_of_unbounded_sequences`
// Member `bounded_sequence_of_unbounded_sequences`
// Member `unbounded_sequence_of_unbounded_sequences`
#include "rosidl_generator_py/msg/unbounded_sequences.h"
// Member `array_of_unbounded_sequences`
// Member `bounded_sequence_of_unbounded_sequences`
// Member `unbounded_sequence_of_unbounded_sequences`
#include "rosidl_generator_py/msg/detail/unbounded_sequences__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__MultiNested_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rosidl_generator_py__msg__MultiNested__init(message_memory);
}

void rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__MultiNested_fini_function(void * message_memory)
{
  rosidl_generator_py__msg__MultiNested__fini(message_memory);
}

size_t rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__size_function__MultiNested__array_of_arrays(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_const_function__MultiNested__array_of_arrays(
  const void * untyped_member, size_t index)
{
  const rosidl_generator_py__msg__Arrays * member =
    (const rosidl_generator_py__msg__Arrays *)(untyped_member);
  return &member[index];
}

void * rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_function__MultiNested__array_of_arrays(
  void * untyped_member, size_t index)
{
  rosidl_generator_py__msg__Arrays * member =
    (rosidl_generator_py__msg__Arrays *)(untyped_member);
  return &member[index];
}

void rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__fetch_function__MultiNested__array_of_arrays(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_generator_py__msg__Arrays * item =
    ((const rosidl_generator_py__msg__Arrays *)
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_const_function__MultiNested__array_of_arrays(untyped_member, index));
  rosidl_generator_py__msg__Arrays * value =
    (rosidl_generator_py__msg__Arrays *)(untyped_value);
  *value = *item;
}

void rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__assign_function__MultiNested__array_of_arrays(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_generator_py__msg__Arrays * item =
    ((rosidl_generator_py__msg__Arrays *)
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_function__MultiNested__array_of_arrays(untyped_member, index));
  const rosidl_generator_py__msg__Arrays * value =
    (const rosidl_generator_py__msg__Arrays *)(untyped_value);
  *item = *value;
}

size_t rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__size_function__MultiNested__array_of_bounded_sequences(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_const_function__MultiNested__array_of_bounded_sequences(
  const void * untyped_member, size_t index)
{
  const rosidl_generator_py__msg__BoundedSequences * member =
    (const rosidl_generator_py__msg__BoundedSequences *)(untyped_member);
  return &member[index];
}

void * rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_function__MultiNested__array_of_bounded_sequences(
  void * untyped_member, size_t index)
{
  rosidl_generator_py__msg__BoundedSequences * member =
    (rosidl_generator_py__msg__BoundedSequences *)(untyped_member);
  return &member[index];
}

void rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__fetch_function__MultiNested__array_of_bounded_sequences(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_generator_py__msg__BoundedSequences * item =
    ((const rosidl_generator_py__msg__BoundedSequences *)
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_const_function__MultiNested__array_of_bounded_sequences(untyped_member, index));
  rosidl_generator_py__msg__BoundedSequences * value =
    (rosidl_generator_py__msg__BoundedSequences *)(untyped_value);
  *value = *item;
}

void rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__assign_function__MultiNested__array_of_bounded_sequences(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_generator_py__msg__BoundedSequences * item =
    ((rosidl_generator_py__msg__BoundedSequences *)
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_function__MultiNested__array_of_bounded_sequences(untyped_member, index));
  const rosidl_generator_py__msg__BoundedSequences * value =
    (const rosidl_generator_py__msg__BoundedSequences *)(untyped_value);
  *item = *value;
}

size_t rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__size_function__MultiNested__array_of_unbounded_sequences(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_const_function__MultiNested__array_of_unbounded_sequences(
  const void * untyped_member, size_t index)
{
  const rosidl_generator_py__msg__UnboundedSequences * member =
    (const rosidl_generator_py__msg__UnboundedSequences *)(untyped_member);
  return &member[index];
}

void * rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_function__MultiNested__array_of_unbounded_sequences(
  void * untyped_member, size_t index)
{
  rosidl_generator_py__msg__UnboundedSequences * member =
    (rosidl_generator_py__msg__UnboundedSequences *)(untyped_member);
  return &member[index];
}

void rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__fetch_function__MultiNested__array_of_unbounded_sequences(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_generator_py__msg__UnboundedSequences * item =
    ((const rosidl_generator_py__msg__UnboundedSequences *)
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_const_function__MultiNested__array_of_unbounded_sequences(untyped_member, index));
  rosidl_generator_py__msg__UnboundedSequences * value =
    (rosidl_generator_py__msg__UnboundedSequences *)(untyped_value);
  *value = *item;
}

void rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__assign_function__MultiNested__array_of_unbounded_sequences(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_generator_py__msg__UnboundedSequences * item =
    ((rosidl_generator_py__msg__UnboundedSequences *)
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_function__MultiNested__array_of_unbounded_sequences(untyped_member, index));
  const rosidl_generator_py__msg__UnboundedSequences * value =
    (const rosidl_generator_py__msg__UnboundedSequences *)(untyped_value);
  *item = *value;
}

size_t rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__size_function__MultiNested__bounded_sequence_of_arrays(
  const void * untyped_member)
{
  const rosidl_generator_py__msg__Arrays__Sequence * member =
    (const rosidl_generator_py__msg__Arrays__Sequence *)(untyped_member);
  return member->size;
}

const void * rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_const_function__MultiNested__bounded_sequence_of_arrays(
  const void * untyped_member, size_t index)
{
  const rosidl_generator_py__msg__Arrays__Sequence * member =
    (const rosidl_generator_py__msg__Arrays__Sequence *)(untyped_member);
  return &member->data[index];
}

void * rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_function__MultiNested__bounded_sequence_of_arrays(
  void * untyped_member, size_t index)
{
  rosidl_generator_py__msg__Arrays__Sequence * member =
    (rosidl_generator_py__msg__Arrays__Sequence *)(untyped_member);
  return &member->data[index];
}

void rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__fetch_function__MultiNested__bounded_sequence_of_arrays(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_generator_py__msg__Arrays * item =
    ((const rosidl_generator_py__msg__Arrays *)
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_const_function__MultiNested__bounded_sequence_of_arrays(untyped_member, index));
  rosidl_generator_py__msg__Arrays * value =
    (rosidl_generator_py__msg__Arrays *)(untyped_value);
  *value = *item;
}

void rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__assign_function__MultiNested__bounded_sequence_of_arrays(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_generator_py__msg__Arrays * item =
    ((rosidl_generator_py__msg__Arrays *)
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_function__MultiNested__bounded_sequence_of_arrays(untyped_member, index));
  const rosidl_generator_py__msg__Arrays * value =
    (const rosidl_generator_py__msg__Arrays *)(untyped_value);
  *item = *value;
}

bool rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__resize_function__MultiNested__bounded_sequence_of_arrays(
  void * untyped_member, size_t size)
{
  rosidl_generator_py__msg__Arrays__Sequence * member =
    (rosidl_generator_py__msg__Arrays__Sequence *)(untyped_member);
  rosidl_generator_py__msg__Arrays__Sequence__fini(member);
  return rosidl_generator_py__msg__Arrays__Sequence__init(member, size);
}

size_t rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__size_function__MultiNested__bounded_sequence_of_bounded_sequences(
  const void * untyped_member)
{
  const rosidl_generator_py__msg__BoundedSequences__Sequence * member =
    (const rosidl_generator_py__msg__BoundedSequences__Sequence *)(untyped_member);
  return member->size;
}

const void * rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_const_function__MultiNested__bounded_sequence_of_bounded_sequences(
  const void * untyped_member, size_t index)
{
  const rosidl_generator_py__msg__BoundedSequences__Sequence * member =
    (const rosidl_generator_py__msg__BoundedSequences__Sequence *)(untyped_member);
  return &member->data[index];
}

void * rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_function__MultiNested__bounded_sequence_of_bounded_sequences(
  void * untyped_member, size_t index)
{
  rosidl_generator_py__msg__BoundedSequences__Sequence * member =
    (rosidl_generator_py__msg__BoundedSequences__Sequence *)(untyped_member);
  return &member->data[index];
}

void rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__fetch_function__MultiNested__bounded_sequence_of_bounded_sequences(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_generator_py__msg__BoundedSequences * item =
    ((const rosidl_generator_py__msg__BoundedSequences *)
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_const_function__MultiNested__bounded_sequence_of_bounded_sequences(untyped_member, index));
  rosidl_generator_py__msg__BoundedSequences * value =
    (rosidl_generator_py__msg__BoundedSequences *)(untyped_value);
  *value = *item;
}

void rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__assign_function__MultiNested__bounded_sequence_of_bounded_sequences(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_generator_py__msg__BoundedSequences * item =
    ((rosidl_generator_py__msg__BoundedSequences *)
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_function__MultiNested__bounded_sequence_of_bounded_sequences(untyped_member, index));
  const rosidl_generator_py__msg__BoundedSequences * value =
    (const rosidl_generator_py__msg__BoundedSequences *)(untyped_value);
  *item = *value;
}

bool rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__resize_function__MultiNested__bounded_sequence_of_bounded_sequences(
  void * untyped_member, size_t size)
{
  rosidl_generator_py__msg__BoundedSequences__Sequence * member =
    (rosidl_generator_py__msg__BoundedSequences__Sequence *)(untyped_member);
  rosidl_generator_py__msg__BoundedSequences__Sequence__fini(member);
  return rosidl_generator_py__msg__BoundedSequences__Sequence__init(member, size);
}

size_t rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__size_function__MultiNested__bounded_sequence_of_unbounded_sequences(
  const void * untyped_member)
{
  const rosidl_generator_py__msg__UnboundedSequences__Sequence * member =
    (const rosidl_generator_py__msg__UnboundedSequences__Sequence *)(untyped_member);
  return member->size;
}

const void * rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_const_function__MultiNested__bounded_sequence_of_unbounded_sequences(
  const void * untyped_member, size_t index)
{
  const rosidl_generator_py__msg__UnboundedSequences__Sequence * member =
    (const rosidl_generator_py__msg__UnboundedSequences__Sequence *)(untyped_member);
  return &member->data[index];
}

void * rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_function__MultiNested__bounded_sequence_of_unbounded_sequences(
  void * untyped_member, size_t index)
{
  rosidl_generator_py__msg__UnboundedSequences__Sequence * member =
    (rosidl_generator_py__msg__UnboundedSequences__Sequence *)(untyped_member);
  return &member->data[index];
}

void rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__fetch_function__MultiNested__bounded_sequence_of_unbounded_sequences(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_generator_py__msg__UnboundedSequences * item =
    ((const rosidl_generator_py__msg__UnboundedSequences *)
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_const_function__MultiNested__bounded_sequence_of_unbounded_sequences(untyped_member, index));
  rosidl_generator_py__msg__UnboundedSequences * value =
    (rosidl_generator_py__msg__UnboundedSequences *)(untyped_value);
  *value = *item;
}

void rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__assign_function__MultiNested__bounded_sequence_of_unbounded_sequences(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_generator_py__msg__UnboundedSequences * item =
    ((rosidl_generator_py__msg__UnboundedSequences *)
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_function__MultiNested__bounded_sequence_of_unbounded_sequences(untyped_member, index));
  const rosidl_generator_py__msg__UnboundedSequences * value =
    (const rosidl_generator_py__msg__UnboundedSequences *)(untyped_value);
  *item = *value;
}

bool rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__resize_function__MultiNested__bounded_sequence_of_unbounded_sequences(
  void * untyped_member, size_t size)
{
  rosidl_generator_py__msg__UnboundedSequences__Sequence * member =
    (rosidl_generator_py__msg__UnboundedSequences__Sequence *)(untyped_member);
  rosidl_generator_py__msg__UnboundedSequences__Sequence__fini(member);
  return rosidl_generator_py__msg__UnboundedSequences__Sequence__init(member, size);
}

size_t rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__size_function__MultiNested__unbounded_sequence_of_arrays(
  const void * untyped_member)
{
  const rosidl_generator_py__msg__Arrays__Sequence * member =
    (const rosidl_generator_py__msg__Arrays__Sequence *)(untyped_member);
  return member->size;
}

const void * rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_const_function__MultiNested__unbounded_sequence_of_arrays(
  const void * untyped_member, size_t index)
{
  const rosidl_generator_py__msg__Arrays__Sequence * member =
    (const rosidl_generator_py__msg__Arrays__Sequence *)(untyped_member);
  return &member->data[index];
}

void * rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_function__MultiNested__unbounded_sequence_of_arrays(
  void * untyped_member, size_t index)
{
  rosidl_generator_py__msg__Arrays__Sequence * member =
    (rosidl_generator_py__msg__Arrays__Sequence *)(untyped_member);
  return &member->data[index];
}

void rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__fetch_function__MultiNested__unbounded_sequence_of_arrays(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_generator_py__msg__Arrays * item =
    ((const rosidl_generator_py__msg__Arrays *)
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_const_function__MultiNested__unbounded_sequence_of_arrays(untyped_member, index));
  rosidl_generator_py__msg__Arrays * value =
    (rosidl_generator_py__msg__Arrays *)(untyped_value);
  *value = *item;
}

void rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__assign_function__MultiNested__unbounded_sequence_of_arrays(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_generator_py__msg__Arrays * item =
    ((rosidl_generator_py__msg__Arrays *)
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_function__MultiNested__unbounded_sequence_of_arrays(untyped_member, index));
  const rosidl_generator_py__msg__Arrays * value =
    (const rosidl_generator_py__msg__Arrays *)(untyped_value);
  *item = *value;
}

bool rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__resize_function__MultiNested__unbounded_sequence_of_arrays(
  void * untyped_member, size_t size)
{
  rosidl_generator_py__msg__Arrays__Sequence * member =
    (rosidl_generator_py__msg__Arrays__Sequence *)(untyped_member);
  rosidl_generator_py__msg__Arrays__Sequence__fini(member);
  return rosidl_generator_py__msg__Arrays__Sequence__init(member, size);
}

size_t rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__size_function__MultiNested__unbounded_sequence_of_bounded_sequences(
  const void * untyped_member)
{
  const rosidl_generator_py__msg__BoundedSequences__Sequence * member =
    (const rosidl_generator_py__msg__BoundedSequences__Sequence *)(untyped_member);
  return member->size;
}

const void * rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_const_function__MultiNested__unbounded_sequence_of_bounded_sequences(
  const void * untyped_member, size_t index)
{
  const rosidl_generator_py__msg__BoundedSequences__Sequence * member =
    (const rosidl_generator_py__msg__BoundedSequences__Sequence *)(untyped_member);
  return &member->data[index];
}

void * rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_function__MultiNested__unbounded_sequence_of_bounded_sequences(
  void * untyped_member, size_t index)
{
  rosidl_generator_py__msg__BoundedSequences__Sequence * member =
    (rosidl_generator_py__msg__BoundedSequences__Sequence *)(untyped_member);
  return &member->data[index];
}

void rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__fetch_function__MultiNested__unbounded_sequence_of_bounded_sequences(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_generator_py__msg__BoundedSequences * item =
    ((const rosidl_generator_py__msg__BoundedSequences *)
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_const_function__MultiNested__unbounded_sequence_of_bounded_sequences(untyped_member, index));
  rosidl_generator_py__msg__BoundedSequences * value =
    (rosidl_generator_py__msg__BoundedSequences *)(untyped_value);
  *value = *item;
}

void rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__assign_function__MultiNested__unbounded_sequence_of_bounded_sequences(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_generator_py__msg__BoundedSequences * item =
    ((rosidl_generator_py__msg__BoundedSequences *)
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_function__MultiNested__unbounded_sequence_of_bounded_sequences(untyped_member, index));
  const rosidl_generator_py__msg__BoundedSequences * value =
    (const rosidl_generator_py__msg__BoundedSequences *)(untyped_value);
  *item = *value;
}

bool rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__resize_function__MultiNested__unbounded_sequence_of_bounded_sequences(
  void * untyped_member, size_t size)
{
  rosidl_generator_py__msg__BoundedSequences__Sequence * member =
    (rosidl_generator_py__msg__BoundedSequences__Sequence *)(untyped_member);
  rosidl_generator_py__msg__BoundedSequences__Sequence__fini(member);
  return rosidl_generator_py__msg__BoundedSequences__Sequence__init(member, size);
}

size_t rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__size_function__MultiNested__unbounded_sequence_of_unbounded_sequences(
  const void * untyped_member)
{
  const rosidl_generator_py__msg__UnboundedSequences__Sequence * member =
    (const rosidl_generator_py__msg__UnboundedSequences__Sequence *)(untyped_member);
  return member->size;
}

const void * rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_const_function__MultiNested__unbounded_sequence_of_unbounded_sequences(
  const void * untyped_member, size_t index)
{
  const rosidl_generator_py__msg__UnboundedSequences__Sequence * member =
    (const rosidl_generator_py__msg__UnboundedSequences__Sequence *)(untyped_member);
  return &member->data[index];
}

void * rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_function__MultiNested__unbounded_sequence_of_unbounded_sequences(
  void * untyped_member, size_t index)
{
  rosidl_generator_py__msg__UnboundedSequences__Sequence * member =
    (rosidl_generator_py__msg__UnboundedSequences__Sequence *)(untyped_member);
  return &member->data[index];
}

void rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__fetch_function__MultiNested__unbounded_sequence_of_unbounded_sequences(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_generator_py__msg__UnboundedSequences * item =
    ((const rosidl_generator_py__msg__UnboundedSequences *)
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_const_function__MultiNested__unbounded_sequence_of_unbounded_sequences(untyped_member, index));
  rosidl_generator_py__msg__UnboundedSequences * value =
    (rosidl_generator_py__msg__UnboundedSequences *)(untyped_value);
  *value = *item;
}

void rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__assign_function__MultiNested__unbounded_sequence_of_unbounded_sequences(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_generator_py__msg__UnboundedSequences * item =
    ((rosidl_generator_py__msg__UnboundedSequences *)
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_function__MultiNested__unbounded_sequence_of_unbounded_sequences(untyped_member, index));
  const rosidl_generator_py__msg__UnboundedSequences * value =
    (const rosidl_generator_py__msg__UnboundedSequences *)(untyped_value);
  *item = *value;
}

bool rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__resize_function__MultiNested__unbounded_sequence_of_unbounded_sequences(
  void * untyped_member, size_t size)
{
  rosidl_generator_py__msg__UnboundedSequences__Sequence * member =
    (rosidl_generator_py__msg__UnboundedSequences__Sequence *)(untyped_member);
  rosidl_generator_py__msg__UnboundedSequences__Sequence__fini(member);
  return rosidl_generator_py__msg__UnboundedSequences__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__MultiNested_message_member_array[9] = {
  {
    "array_of_arrays",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(rosidl_generator_py__msg__MultiNested, array_of_arrays),  // bytes offset in struct
    NULL,  // default value
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__size_function__MultiNested__array_of_arrays,  // size() function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_const_function__MultiNested__array_of_arrays,  // get_const(index) function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_function__MultiNested__array_of_arrays,  // get(index) function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__fetch_function__MultiNested__array_of_arrays,  // fetch(index, &value) function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__assign_function__MultiNested__array_of_arrays,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "array_of_bounded_sequences",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(rosidl_generator_py__msg__MultiNested, array_of_bounded_sequences),  // bytes offset in struct
    NULL,  // default value
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__size_function__MultiNested__array_of_bounded_sequences,  // size() function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_const_function__MultiNested__array_of_bounded_sequences,  // get_const(index) function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_function__MultiNested__array_of_bounded_sequences,  // get(index) function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__fetch_function__MultiNested__array_of_bounded_sequences,  // fetch(index, &value) function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__assign_function__MultiNested__array_of_bounded_sequences,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "array_of_unbounded_sequences",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(rosidl_generator_py__msg__MultiNested, array_of_unbounded_sequences),  // bytes offset in struct
    NULL,  // default value
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__size_function__MultiNested__array_of_unbounded_sequences,  // size() function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_const_function__MultiNested__array_of_unbounded_sequences,  // get_const(index) function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_function__MultiNested__array_of_unbounded_sequences,  // get(index) function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__fetch_function__MultiNested__array_of_unbounded_sequences,  // fetch(index, &value) function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__assign_function__MultiNested__array_of_unbounded_sequences,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bounded_sequence_of_arrays",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    3,  // array size
    true,  // is upper bound
    offsetof(rosidl_generator_py__msg__MultiNested, bounded_sequence_of_arrays),  // bytes offset in struct
    NULL,  // default value
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__size_function__MultiNested__bounded_sequence_of_arrays,  // size() function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_const_function__MultiNested__bounded_sequence_of_arrays,  // get_const(index) function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_function__MultiNested__bounded_sequence_of_arrays,  // get(index) function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__fetch_function__MultiNested__bounded_sequence_of_arrays,  // fetch(index, &value) function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__assign_function__MultiNested__bounded_sequence_of_arrays,  // assign(index, value) function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__resize_function__MultiNested__bounded_sequence_of_arrays  // resize(index) function pointer
  },
  {
    "bounded_sequence_of_bounded_sequences",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    3,  // array size
    true,  // is upper bound
    offsetof(rosidl_generator_py__msg__MultiNested, bounded_sequence_of_bounded_sequences),  // bytes offset in struct
    NULL,  // default value
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__size_function__MultiNested__bounded_sequence_of_bounded_sequences,  // size() function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_const_function__MultiNested__bounded_sequence_of_bounded_sequences,  // get_const(index) function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_function__MultiNested__bounded_sequence_of_bounded_sequences,  // get(index) function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__fetch_function__MultiNested__bounded_sequence_of_bounded_sequences,  // fetch(index, &value) function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__assign_function__MultiNested__bounded_sequence_of_bounded_sequences,  // assign(index, value) function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__resize_function__MultiNested__bounded_sequence_of_bounded_sequences  // resize(index) function pointer
  },
  {
    "bounded_sequence_of_unbounded_sequences",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    3,  // array size
    true,  // is upper bound
    offsetof(rosidl_generator_py__msg__MultiNested, bounded_sequence_of_unbounded_sequences),  // bytes offset in struct
    NULL,  // default value
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__size_function__MultiNested__bounded_sequence_of_unbounded_sequences,  // size() function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_const_function__MultiNested__bounded_sequence_of_unbounded_sequences,  // get_const(index) function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_function__MultiNested__bounded_sequence_of_unbounded_sequences,  // get(index) function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__fetch_function__MultiNested__bounded_sequence_of_unbounded_sequences,  // fetch(index, &value) function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__assign_function__MultiNested__bounded_sequence_of_unbounded_sequences,  // assign(index, value) function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__resize_function__MultiNested__bounded_sequence_of_unbounded_sequences  // resize(index) function pointer
  },
  {
    "unbounded_sequence_of_arrays",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_generator_py__msg__MultiNested, unbounded_sequence_of_arrays),  // bytes offset in struct
    NULL,  // default value
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__size_function__MultiNested__unbounded_sequence_of_arrays,  // size() function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_const_function__MultiNested__unbounded_sequence_of_arrays,  // get_const(index) function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_function__MultiNested__unbounded_sequence_of_arrays,  // get(index) function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__fetch_function__MultiNested__unbounded_sequence_of_arrays,  // fetch(index, &value) function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__assign_function__MultiNested__unbounded_sequence_of_arrays,  // assign(index, value) function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__resize_function__MultiNested__unbounded_sequence_of_arrays  // resize(index) function pointer
  },
  {
    "unbounded_sequence_of_bounded_sequences",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_generator_py__msg__MultiNested, unbounded_sequence_of_bounded_sequences),  // bytes offset in struct
    NULL,  // default value
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__size_function__MultiNested__unbounded_sequence_of_bounded_sequences,  // size() function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_const_function__MultiNested__unbounded_sequence_of_bounded_sequences,  // get_const(index) function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_function__MultiNested__unbounded_sequence_of_bounded_sequences,  // get(index) function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__fetch_function__MultiNested__unbounded_sequence_of_bounded_sequences,  // fetch(index, &value) function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__assign_function__MultiNested__unbounded_sequence_of_bounded_sequences,  // assign(index, value) function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__resize_function__MultiNested__unbounded_sequence_of_bounded_sequences  // resize(index) function pointer
  },
  {
    "unbounded_sequence_of_unbounded_sequences",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_generator_py__msg__MultiNested, unbounded_sequence_of_unbounded_sequences),  // bytes offset in struct
    NULL,  // default value
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__size_function__MultiNested__unbounded_sequence_of_unbounded_sequences,  // size() function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_const_function__MultiNested__unbounded_sequence_of_unbounded_sequences,  // get_const(index) function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__get_function__MultiNested__unbounded_sequence_of_unbounded_sequences,  // get(index) function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__fetch_function__MultiNested__unbounded_sequence_of_unbounded_sequences,  // fetch(index, &value) function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__assign_function__MultiNested__unbounded_sequence_of_unbounded_sequences,  // assign(index, value) function pointer
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__resize_function__MultiNested__unbounded_sequence_of_unbounded_sequences  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__MultiNested_message_members = {
  "rosidl_generator_py__msg",  // message namespace
  "MultiNested",  // message name
  9,  // number of fields
  sizeof(rosidl_generator_py__msg__MultiNested),
  false,  // has_any_key_member_
  rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__MultiNested_message_member_array,  // message members
  rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__MultiNested_init_function,  // function to initialize message memory (memory has to be allocated)
  rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__MultiNested_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__MultiNested_message_type_support_handle = {
  0,
  &rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__MultiNested_message_members,
  get_message_typesupport_handle_function,
  &rosidl_generator_py__msg__MultiNested__get_type_hash,
  &rosidl_generator_py__msg__MultiNested__get_type_description,
  &rosidl_generator_py__msg__MultiNested__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rosidl_generator_py
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosidl_generator_py, msg, MultiNested)() {
  rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__MultiNested_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosidl_generator_py, msg, Arrays)();
  rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__MultiNested_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosidl_generator_py, msg, BoundedSequences)();
  rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__MultiNested_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosidl_generator_py, msg, UnboundedSequences)();
  rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__MultiNested_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosidl_generator_py, msg, Arrays)();
  rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__MultiNested_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosidl_generator_py, msg, BoundedSequences)();
  rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__MultiNested_message_member_array[5].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosidl_generator_py, msg, UnboundedSequences)();
  rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__MultiNested_message_member_array[6].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosidl_generator_py, msg, Arrays)();
  rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__MultiNested_message_member_array[7].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosidl_generator_py, msg, BoundedSequences)();
  rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__MultiNested_message_member_array[8].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosidl_generator_py, msg, UnboundedSequences)();
  if (!rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__MultiNested_message_type_support_handle.typesupport_identifier) {
    rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__MultiNested_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rosidl_generator_py__msg__MultiNested__rosidl_typesupport_introspection_c__MultiNested_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
