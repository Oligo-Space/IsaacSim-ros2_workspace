// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rosidl_generator_py:msg/StringArrays.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rosidl_generator_py/msg/detail/string_arrays__rosidl_typesupport_introspection_c.h"
#include "rosidl_generator_py/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rosidl_generator_py/msg/detail/string_arrays__functions.h"
#include "rosidl_generator_py/msg/detail/string_arrays__struct.h"


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

#ifdef __cplusplus
extern "C"
{
#endif

void rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__StringArrays_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rosidl_generator_py__msg__StringArrays__init(message_memory);
}

void rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__StringArrays_fini_function(void * message_memory)
{
  rosidl_generator_py__msg__StringArrays__fini(message_memory);
}

size_t rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__size_function__StringArrays__ub_string_static_array_value(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_const_function__StringArrays__ub_string_static_array_value(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String * member =
    (const rosidl_runtime_c__String *)(untyped_member);
  return &member[index];
}

void * rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_function__StringArrays__ub_string_static_array_value(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String * member =
    (rosidl_runtime_c__String *)(untyped_member);
  return &member[index];
}

void rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__fetch_function__StringArrays__ub_string_static_array_value(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_const_function__StringArrays__ub_string_static_array_value(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__assign_function__StringArrays__ub_string_static_array_value(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_function__StringArrays__ub_string_static_array_value(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

size_t rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__size_function__StringArrays__ub_string_ub_array_value(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_const_function__StringArrays__ub_string_ub_array_value(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_function__StringArrays__ub_string_ub_array_value(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__fetch_function__StringArrays__ub_string_ub_array_value(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_const_function__StringArrays__ub_string_ub_array_value(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__assign_function__StringArrays__ub_string_ub_array_value(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_function__StringArrays__ub_string_ub_array_value(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__resize_function__StringArrays__ub_string_ub_array_value(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__size_function__StringArrays__ub_string_dynamic_array_value(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_const_function__StringArrays__ub_string_dynamic_array_value(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_function__StringArrays__ub_string_dynamic_array_value(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__fetch_function__StringArrays__ub_string_dynamic_array_value(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_const_function__StringArrays__ub_string_dynamic_array_value(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__assign_function__StringArrays__ub_string_dynamic_array_value(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_function__StringArrays__ub_string_dynamic_array_value(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__resize_function__StringArrays__ub_string_dynamic_array_value(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__size_function__StringArrays__string_dynamic_array_value(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_const_function__StringArrays__string_dynamic_array_value(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_function__StringArrays__string_dynamic_array_value(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__fetch_function__StringArrays__string_dynamic_array_value(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_const_function__StringArrays__string_dynamic_array_value(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__assign_function__StringArrays__string_dynamic_array_value(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_function__StringArrays__string_dynamic_array_value(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__resize_function__StringArrays__string_dynamic_array_value(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__size_function__StringArrays__string_static_array_value(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_const_function__StringArrays__string_static_array_value(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String * member =
    (const rosidl_runtime_c__String *)(untyped_member);
  return &member[index];
}

void * rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_function__StringArrays__string_static_array_value(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String * member =
    (rosidl_runtime_c__String *)(untyped_member);
  return &member[index];
}

void rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__fetch_function__StringArrays__string_static_array_value(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_const_function__StringArrays__string_static_array_value(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__assign_function__StringArrays__string_static_array_value(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_function__StringArrays__string_static_array_value(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

size_t rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__size_function__StringArrays__string_bounded_array_value(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_const_function__StringArrays__string_bounded_array_value(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_function__StringArrays__string_bounded_array_value(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__fetch_function__StringArrays__string_bounded_array_value(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_const_function__StringArrays__string_bounded_array_value(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__assign_function__StringArrays__string_bounded_array_value(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_function__StringArrays__string_bounded_array_value(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__resize_function__StringArrays__string_bounded_array_value(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__size_function__StringArrays__def_string_dynamic_array_value(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_const_function__StringArrays__def_string_dynamic_array_value(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_function__StringArrays__def_string_dynamic_array_value(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__fetch_function__StringArrays__def_string_dynamic_array_value(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_const_function__StringArrays__def_string_dynamic_array_value(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__assign_function__StringArrays__def_string_dynamic_array_value(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_function__StringArrays__def_string_dynamic_array_value(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__resize_function__StringArrays__def_string_dynamic_array_value(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__size_function__StringArrays__def_string_static_array_value(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_const_function__StringArrays__def_string_static_array_value(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String * member =
    (const rosidl_runtime_c__String *)(untyped_member);
  return &member[index];
}

void * rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_function__StringArrays__def_string_static_array_value(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String * member =
    (rosidl_runtime_c__String *)(untyped_member);
  return &member[index];
}

void rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__fetch_function__StringArrays__def_string_static_array_value(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_const_function__StringArrays__def_string_static_array_value(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__assign_function__StringArrays__def_string_static_array_value(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_function__StringArrays__def_string_static_array_value(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

size_t rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__size_function__StringArrays__def_string_bounded_array_value(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_const_function__StringArrays__def_string_bounded_array_value(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_function__StringArrays__def_string_bounded_array_value(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__fetch_function__StringArrays__def_string_bounded_array_value(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_const_function__StringArrays__def_string_bounded_array_value(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__assign_function__StringArrays__def_string_bounded_array_value(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_function__StringArrays__def_string_bounded_array_value(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__resize_function__StringArrays__def_string_bounded_array_value(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__size_function__StringArrays__def_various_quotes(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_const_function__StringArrays__def_various_quotes(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_function__StringArrays__def_various_quotes(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__fetch_function__StringArrays__def_various_quotes(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_const_function__StringArrays__def_various_quotes(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__assign_function__StringArrays__def_various_quotes(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_function__StringArrays__def_various_quotes(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__resize_function__StringArrays__def_various_quotes(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__size_function__StringArrays__def_various_commas(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_const_function__StringArrays__def_various_commas(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_function__StringArrays__def_various_commas(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__fetch_function__StringArrays__def_various_commas(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_const_function__StringArrays__def_various_commas(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__assign_function__StringArrays__def_various_commas(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_function__StringArrays__def_various_commas(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__resize_function__StringArrays__def_various_commas(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__StringArrays_message_member_array[11] = {
  {
    "ub_string_static_array_value",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    5,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(rosidl_generator_py__msg__StringArrays, ub_string_static_array_value),  // bytes offset in struct
    NULL,  // default value
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__size_function__StringArrays__ub_string_static_array_value,  // size() function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_const_function__StringArrays__ub_string_static_array_value,  // get_const(index) function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_function__StringArrays__ub_string_static_array_value,  // get(index) function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__fetch_function__StringArrays__ub_string_static_array_value,  // fetch(index, &value) function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__assign_function__StringArrays__ub_string_static_array_value,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ub_string_ub_array_value",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    5,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    10,  // array size
    true,  // is upper bound
    offsetof(rosidl_generator_py__msg__StringArrays, ub_string_ub_array_value),  // bytes offset in struct
    NULL,  // default value
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__size_function__StringArrays__ub_string_ub_array_value,  // size() function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_const_function__StringArrays__ub_string_ub_array_value,  // get_const(index) function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_function__StringArrays__ub_string_ub_array_value,  // get(index) function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__fetch_function__StringArrays__ub_string_ub_array_value,  // fetch(index, &value) function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__assign_function__StringArrays__ub_string_ub_array_value,  // assign(index, value) function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__resize_function__StringArrays__ub_string_ub_array_value  // resize(index) function pointer
  },
  {
    "ub_string_dynamic_array_value",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    5,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_generator_py__msg__StringArrays, ub_string_dynamic_array_value),  // bytes offset in struct
    NULL,  // default value
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__size_function__StringArrays__ub_string_dynamic_array_value,  // size() function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_const_function__StringArrays__ub_string_dynamic_array_value,  // get_const(index) function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_function__StringArrays__ub_string_dynamic_array_value,  // get(index) function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__fetch_function__StringArrays__ub_string_dynamic_array_value,  // fetch(index, &value) function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__assign_function__StringArrays__ub_string_dynamic_array_value,  // assign(index, value) function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__resize_function__StringArrays__ub_string_dynamic_array_value  // resize(index) function pointer
  },
  {
    "string_dynamic_array_value",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_generator_py__msg__StringArrays, string_dynamic_array_value),  // bytes offset in struct
    NULL,  // default value
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__size_function__StringArrays__string_dynamic_array_value,  // size() function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_const_function__StringArrays__string_dynamic_array_value,  // get_const(index) function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_function__StringArrays__string_dynamic_array_value,  // get(index) function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__fetch_function__StringArrays__string_dynamic_array_value,  // fetch(index, &value) function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__assign_function__StringArrays__string_dynamic_array_value,  // assign(index, value) function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__resize_function__StringArrays__string_dynamic_array_value  // resize(index) function pointer
  },
  {
    "string_static_array_value",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(rosidl_generator_py__msg__StringArrays, string_static_array_value),  // bytes offset in struct
    NULL,  // default value
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__size_function__StringArrays__string_static_array_value,  // size() function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_const_function__StringArrays__string_static_array_value,  // get_const(index) function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_function__StringArrays__string_static_array_value,  // get(index) function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__fetch_function__StringArrays__string_static_array_value,  // fetch(index, &value) function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__assign_function__StringArrays__string_static_array_value,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "string_bounded_array_value",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    10,  // array size
    true,  // is upper bound
    offsetof(rosidl_generator_py__msg__StringArrays, string_bounded_array_value),  // bytes offset in struct
    NULL,  // default value
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__size_function__StringArrays__string_bounded_array_value,  // size() function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_const_function__StringArrays__string_bounded_array_value,  // get_const(index) function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_function__StringArrays__string_bounded_array_value,  // get(index) function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__fetch_function__StringArrays__string_bounded_array_value,  // fetch(index, &value) function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__assign_function__StringArrays__string_bounded_array_value,  // assign(index, value) function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__resize_function__StringArrays__string_bounded_array_value  // resize(index) function pointer
  },
  {
    "def_string_dynamic_array_value",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_generator_py__msg__StringArrays, def_string_dynamic_array_value),  // bytes offset in struct
    NULL,  // default value
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__size_function__StringArrays__def_string_dynamic_array_value,  // size() function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_const_function__StringArrays__def_string_dynamic_array_value,  // get_const(index) function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_function__StringArrays__def_string_dynamic_array_value,  // get(index) function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__fetch_function__StringArrays__def_string_dynamic_array_value,  // fetch(index, &value) function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__assign_function__StringArrays__def_string_dynamic_array_value,  // assign(index, value) function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__resize_function__StringArrays__def_string_dynamic_array_value  // resize(index) function pointer
  },
  {
    "def_string_static_array_value",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(rosidl_generator_py__msg__StringArrays, def_string_static_array_value),  // bytes offset in struct
    NULL,  // default value
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__size_function__StringArrays__def_string_static_array_value,  // size() function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_const_function__StringArrays__def_string_static_array_value,  // get_const(index) function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_function__StringArrays__def_string_static_array_value,  // get(index) function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__fetch_function__StringArrays__def_string_static_array_value,  // fetch(index, &value) function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__assign_function__StringArrays__def_string_static_array_value,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "def_string_bounded_array_value",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    10,  // array size
    true,  // is upper bound
    offsetof(rosidl_generator_py__msg__StringArrays, def_string_bounded_array_value),  // bytes offset in struct
    NULL,  // default value
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__size_function__StringArrays__def_string_bounded_array_value,  // size() function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_const_function__StringArrays__def_string_bounded_array_value,  // get_const(index) function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_function__StringArrays__def_string_bounded_array_value,  // get(index) function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__fetch_function__StringArrays__def_string_bounded_array_value,  // fetch(index, &value) function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__assign_function__StringArrays__def_string_bounded_array_value,  // assign(index, value) function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__resize_function__StringArrays__def_string_bounded_array_value  // resize(index) function pointer
  },
  {
    "def_various_quotes",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_generator_py__msg__StringArrays, def_various_quotes),  // bytes offset in struct
    NULL,  // default value
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__size_function__StringArrays__def_various_quotes,  // size() function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_const_function__StringArrays__def_various_quotes,  // get_const(index) function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_function__StringArrays__def_various_quotes,  // get(index) function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__fetch_function__StringArrays__def_various_quotes,  // fetch(index, &value) function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__assign_function__StringArrays__def_various_quotes,  // assign(index, value) function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__resize_function__StringArrays__def_various_quotes  // resize(index) function pointer
  },
  {
    "def_various_commas",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_generator_py__msg__StringArrays, def_various_commas),  // bytes offset in struct
    NULL,  // default value
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__size_function__StringArrays__def_various_commas,  // size() function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_const_function__StringArrays__def_various_commas,  // get_const(index) function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__get_function__StringArrays__def_various_commas,  // get(index) function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__fetch_function__StringArrays__def_various_commas,  // fetch(index, &value) function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__assign_function__StringArrays__def_various_commas,  // assign(index, value) function pointer
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__resize_function__StringArrays__def_various_commas  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__StringArrays_message_members = {
  "rosidl_generator_py__msg",  // message namespace
  "StringArrays",  // message name
  11,  // number of fields
  sizeof(rosidl_generator_py__msg__StringArrays),
  false,  // has_any_key_member_
  rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__StringArrays_message_member_array,  // message members
  rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__StringArrays_init_function,  // function to initialize message memory (memory has to be allocated)
  rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__StringArrays_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__StringArrays_message_type_support_handle = {
  0,
  &rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__StringArrays_message_members,
  get_message_typesupport_handle_function,
  &rosidl_generator_py__msg__StringArrays__get_type_hash,
  &rosidl_generator_py__msg__StringArrays__get_type_description,
  &rosidl_generator_py__msg__StringArrays__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rosidl_generator_py
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosidl_generator_py, msg, StringArrays)() {
  if (!rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__StringArrays_message_type_support_handle.typesupport_identifier) {
    rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__StringArrays_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rosidl_generator_py__msg__StringArrays__rosidl_typesupport_introspection_c__StringArrays_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
