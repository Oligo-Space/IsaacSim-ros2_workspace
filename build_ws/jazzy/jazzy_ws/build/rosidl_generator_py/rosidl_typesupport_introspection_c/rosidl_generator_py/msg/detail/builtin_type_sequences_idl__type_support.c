// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rosidl_generator_py:msg/BuiltinTypeSequencesIdl.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rosidl_generator_py/msg/detail/builtin_type_sequences_idl__rosidl_typesupport_introspection_c.h"
#include "rosidl_generator_py/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rosidl_generator_py/msg/detail/builtin_type_sequences_idl__functions.h"
#include "rosidl_generator_py/msg/detail/builtin_type_sequences_idl__struct.h"


// Include directives for member types
// Member `char_sequence_unbounded`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rosidl_generator_py__msg__BuiltinTypeSequencesIdl__rosidl_typesupport_introspection_c__BuiltinTypeSequencesIdl_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rosidl_generator_py__msg__BuiltinTypeSequencesIdl__init(message_memory);
}

void rosidl_generator_py__msg__BuiltinTypeSequencesIdl__rosidl_typesupport_introspection_c__BuiltinTypeSequencesIdl_fini_function(void * message_memory)
{
  rosidl_generator_py__msg__BuiltinTypeSequencesIdl__fini(message_memory);
}

size_t rosidl_generator_py__msg__BuiltinTypeSequencesIdl__rosidl_typesupport_introspection_c__size_function__BuiltinTypeSequencesIdl__char_sequence_unbounded(
  const void * untyped_member)
{
  const rosidl_runtime_c__char__Sequence * member =
    (const rosidl_runtime_c__char__Sequence *)(untyped_member);
  return member->size;
}

const void * rosidl_generator_py__msg__BuiltinTypeSequencesIdl__rosidl_typesupport_introspection_c__get_const_function__BuiltinTypeSequencesIdl__char_sequence_unbounded(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__char__Sequence * member =
    (const rosidl_runtime_c__char__Sequence *)(untyped_member);
  return &member->data[index];
}

void * rosidl_generator_py__msg__BuiltinTypeSequencesIdl__rosidl_typesupport_introspection_c__get_function__BuiltinTypeSequencesIdl__char_sequence_unbounded(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__char__Sequence * member =
    (rosidl_runtime_c__char__Sequence *)(untyped_member);
  return &member->data[index];
}

void rosidl_generator_py__msg__BuiltinTypeSequencesIdl__rosidl_typesupport_introspection_c__fetch_function__BuiltinTypeSequencesIdl__char_sequence_unbounded(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const signed char * item =
    ((const signed char *)
    rosidl_generator_py__msg__BuiltinTypeSequencesIdl__rosidl_typesupport_introspection_c__get_const_function__BuiltinTypeSequencesIdl__char_sequence_unbounded(untyped_member, index));
  signed char * value =
    (signed char *)(untyped_value);
  *value = *item;
}

void rosidl_generator_py__msg__BuiltinTypeSequencesIdl__rosidl_typesupport_introspection_c__assign_function__BuiltinTypeSequencesIdl__char_sequence_unbounded(
  void * untyped_member, size_t index, const void * untyped_value)
{
  signed char * item =
    ((signed char *)
    rosidl_generator_py__msg__BuiltinTypeSequencesIdl__rosidl_typesupport_introspection_c__get_function__BuiltinTypeSequencesIdl__char_sequence_unbounded(untyped_member, index));
  const signed char * value =
    (const signed char *)(untyped_value);
  *item = *value;
}

bool rosidl_generator_py__msg__BuiltinTypeSequencesIdl__rosidl_typesupport_introspection_c__resize_function__BuiltinTypeSequencesIdl__char_sequence_unbounded(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__char__Sequence * member =
    (rosidl_runtime_c__char__Sequence *)(untyped_member);
  rosidl_runtime_c__char__Sequence__fini(member);
  return rosidl_runtime_c__char__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember rosidl_generator_py__msg__BuiltinTypeSequencesIdl__rosidl_typesupport_introspection_c__BuiltinTypeSequencesIdl_message_member_array[1] = {
  {
    "char_sequence_unbounded",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_CHAR,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_generator_py__msg__BuiltinTypeSequencesIdl, char_sequence_unbounded),  // bytes offset in struct
    NULL,  // default value
    rosidl_generator_py__msg__BuiltinTypeSequencesIdl__rosidl_typesupport_introspection_c__size_function__BuiltinTypeSequencesIdl__char_sequence_unbounded,  // size() function pointer
    rosidl_generator_py__msg__BuiltinTypeSequencesIdl__rosidl_typesupport_introspection_c__get_const_function__BuiltinTypeSequencesIdl__char_sequence_unbounded,  // get_const(index) function pointer
    rosidl_generator_py__msg__BuiltinTypeSequencesIdl__rosidl_typesupport_introspection_c__get_function__BuiltinTypeSequencesIdl__char_sequence_unbounded,  // get(index) function pointer
    rosidl_generator_py__msg__BuiltinTypeSequencesIdl__rosidl_typesupport_introspection_c__fetch_function__BuiltinTypeSequencesIdl__char_sequence_unbounded,  // fetch(index, &value) function pointer
    rosidl_generator_py__msg__BuiltinTypeSequencesIdl__rosidl_typesupport_introspection_c__assign_function__BuiltinTypeSequencesIdl__char_sequence_unbounded,  // assign(index, value) function pointer
    rosidl_generator_py__msg__BuiltinTypeSequencesIdl__rosidl_typesupport_introspection_c__resize_function__BuiltinTypeSequencesIdl__char_sequence_unbounded  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rosidl_generator_py__msg__BuiltinTypeSequencesIdl__rosidl_typesupport_introspection_c__BuiltinTypeSequencesIdl_message_members = {
  "rosidl_generator_py__msg",  // message namespace
  "BuiltinTypeSequencesIdl",  // message name
  1,  // number of fields
  sizeof(rosidl_generator_py__msg__BuiltinTypeSequencesIdl),
  false,  // has_any_key_member_
  rosidl_generator_py__msg__BuiltinTypeSequencesIdl__rosidl_typesupport_introspection_c__BuiltinTypeSequencesIdl_message_member_array,  // message members
  rosidl_generator_py__msg__BuiltinTypeSequencesIdl__rosidl_typesupport_introspection_c__BuiltinTypeSequencesIdl_init_function,  // function to initialize message memory (memory has to be allocated)
  rosidl_generator_py__msg__BuiltinTypeSequencesIdl__rosidl_typesupport_introspection_c__BuiltinTypeSequencesIdl_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rosidl_generator_py__msg__BuiltinTypeSequencesIdl__rosidl_typesupport_introspection_c__BuiltinTypeSequencesIdl_message_type_support_handle = {
  0,
  &rosidl_generator_py__msg__BuiltinTypeSequencesIdl__rosidl_typesupport_introspection_c__BuiltinTypeSequencesIdl_message_members,
  get_message_typesupport_handle_function,
  &rosidl_generator_py__msg__BuiltinTypeSequencesIdl__get_type_hash,
  &rosidl_generator_py__msg__BuiltinTypeSequencesIdl__get_type_description,
  &rosidl_generator_py__msg__BuiltinTypeSequencesIdl__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rosidl_generator_py
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosidl_generator_py, msg, BuiltinTypeSequencesIdl)() {
  if (!rosidl_generator_py__msg__BuiltinTypeSequencesIdl__rosidl_typesupport_introspection_c__BuiltinTypeSequencesIdl_message_type_support_handle.typesupport_identifier) {
    rosidl_generator_py__msg__BuiltinTypeSequencesIdl__rosidl_typesupport_introspection_c__BuiltinTypeSequencesIdl_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rosidl_generator_py__msg__BuiltinTypeSequencesIdl__rosidl_typesupport_introspection_c__BuiltinTypeSequencesIdl_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
