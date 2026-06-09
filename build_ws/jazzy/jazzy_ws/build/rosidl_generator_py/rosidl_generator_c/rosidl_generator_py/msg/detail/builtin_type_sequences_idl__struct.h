// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rosidl_generator_py:msg/BuiltinTypeSequencesIdl.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "rosidl_generator_py/msg/builtin_type_sequences_idl.h"


#ifndef ROSIDL_GENERATOR_PY__MSG__DETAIL__BUILTIN_TYPE_SEQUENCES_IDL__STRUCT_H_
#define ROSIDL_GENERATOR_PY__MSG__DETAIL__BUILTIN_TYPE_SEQUENCES_IDL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'char_sequence_unbounded'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/BuiltinTypeSequencesIdl in the package rosidl_generator_py.
typedef struct rosidl_generator_py__msg__BuiltinTypeSequencesIdl
{
  rosidl_runtime_c__char__Sequence char_sequence_unbounded;
} rosidl_generator_py__msg__BuiltinTypeSequencesIdl;

// Struct for a sequence of rosidl_generator_py__msg__BuiltinTypeSequencesIdl.
typedef struct rosidl_generator_py__msg__BuiltinTypeSequencesIdl__Sequence
{
  rosidl_generator_py__msg__BuiltinTypeSequencesIdl * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rosidl_generator_py__msg__BuiltinTypeSequencesIdl__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROSIDL_GENERATOR_PY__MSG__DETAIL__BUILTIN_TYPE_SEQUENCES_IDL__STRUCT_H_
