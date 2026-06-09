// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rosidl_generator_py:msg/Strings.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rosidl_generator_py/msg/detail/strings__rosidl_typesupport_introspection_c.h"
#include "rosidl_generator_py/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rosidl_generator_py/msg/detail/strings__functions.h"
#include "rosidl_generator_py/msg/detail/strings__struct.h"


// Include directives for member types
// Member `string_value`
// Member `string_value_default1`
// Member `string_value_default2`
// Member `string_value_default3`
// Member `string_value_default4`
// Member `string_value_default5`
// Member `bounded_string_value`
// Member `bounded_string_value_default1`
// Member `bounded_string_value_default2`
// Member `bounded_string_value_default3`
// Member `bounded_string_value_default4`
// Member `bounded_string_value_default5`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rosidl_generator_py__msg__Strings__rosidl_typesupport_introspection_c__Strings_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rosidl_generator_py__msg__Strings__init(message_memory);
}

void rosidl_generator_py__msg__Strings__rosidl_typesupport_introspection_c__Strings_fini_function(void * message_memory)
{
  rosidl_generator_py__msg__Strings__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember rosidl_generator_py__msg__Strings__rosidl_typesupport_introspection_c__Strings_message_member_array[12] = {
  {
    "string_value",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_generator_py__msg__Strings, string_value),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "string_value_default1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_generator_py__msg__Strings, string_value_default1),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "string_value_default2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_generator_py__msg__Strings, string_value_default2),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "string_value_default3",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_generator_py__msg__Strings, string_value_default3),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "string_value_default4",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_generator_py__msg__Strings, string_value_default4),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "string_value_default5",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_generator_py__msg__Strings, string_value_default5),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bounded_string_value",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    22,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_generator_py__msg__Strings, bounded_string_value),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bounded_string_value_default1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    22,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_generator_py__msg__Strings, bounded_string_value_default1),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bounded_string_value_default2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    22,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_generator_py__msg__Strings, bounded_string_value_default2),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bounded_string_value_default3",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    22,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_generator_py__msg__Strings, bounded_string_value_default3),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bounded_string_value_default4",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    22,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_generator_py__msg__Strings, bounded_string_value_default4),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bounded_string_value_default5",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    22,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_generator_py__msg__Strings, bounded_string_value_default5),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rosidl_generator_py__msg__Strings__rosidl_typesupport_introspection_c__Strings_message_members = {
  "rosidl_generator_py__msg",  // message namespace
  "Strings",  // message name
  12,  // number of fields
  sizeof(rosidl_generator_py__msg__Strings),
  false,  // has_any_key_member_
  rosidl_generator_py__msg__Strings__rosidl_typesupport_introspection_c__Strings_message_member_array,  // message members
  rosidl_generator_py__msg__Strings__rosidl_typesupport_introspection_c__Strings_init_function,  // function to initialize message memory (memory has to be allocated)
  rosidl_generator_py__msg__Strings__rosidl_typesupport_introspection_c__Strings_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rosidl_generator_py__msg__Strings__rosidl_typesupport_introspection_c__Strings_message_type_support_handle = {
  0,
  &rosidl_generator_py__msg__Strings__rosidl_typesupport_introspection_c__Strings_message_members,
  get_message_typesupport_handle_function,
  &rosidl_generator_py__msg__Strings__get_type_hash,
  &rosidl_generator_py__msg__Strings__get_type_description,
  &rosidl_generator_py__msg__Strings__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rosidl_generator_py
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosidl_generator_py, msg, Strings)() {
  if (!rosidl_generator_py__msg__Strings__rosidl_typesupport_introspection_c__Strings_message_type_support_handle.typesupport_identifier) {
    rosidl_generator_py__msg__Strings__rosidl_typesupport_introspection_c__Strings_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rosidl_generator_py__msg__Strings__rosidl_typesupport_introspection_c__Strings_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
