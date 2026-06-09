// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from rosidl_generator_py:msg/Property.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "rosidl_generator_py/msg/property.h"


#ifndef ROSIDL_GENERATOR_PY__MSG__DETAIL__PROPERTY__STRUCT_H_
#define ROSIDL_GENERATOR_PY__MSG__DETAIL__PROPERTY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'property'
// Member 'anything'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Property in the package rosidl_generator_py.
typedef struct rosidl_generator_py__msg__Property
{
  rosidl_runtime_c__String property;
  rosidl_runtime_c__String anything;
} rosidl_generator_py__msg__Property;

// Struct for a sequence of rosidl_generator_py__msg__Property.
typedef struct rosidl_generator_py__msg__Property__Sequence
{
  rosidl_generator_py__msg__Property * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} rosidl_generator_py__msg__Property__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROSIDL_GENERATOR_PY__MSG__DETAIL__PROPERTY__STRUCT_H_
