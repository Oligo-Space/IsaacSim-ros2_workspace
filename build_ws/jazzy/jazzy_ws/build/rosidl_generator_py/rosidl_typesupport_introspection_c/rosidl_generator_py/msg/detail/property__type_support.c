// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from rosidl_generator_py:msg/Property.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "rosidl_generator_py/msg/detail/property__rosidl_typesupport_introspection_c.h"
#include "rosidl_generator_py/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "rosidl_generator_py/msg/detail/property__functions.h"
#include "rosidl_generator_py/msg/detail/property__struct.h"


// Include directives for member types
// Member `property`
// Member `anything`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void rosidl_generator_py__msg__Property__rosidl_typesupport_introspection_c__Property_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  rosidl_generator_py__msg__Property__init(message_memory);
}

void rosidl_generator_py__msg__Property__rosidl_typesupport_introspection_c__Property_fini_function(void * message_memory)
{
  rosidl_generator_py__msg__Property__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember rosidl_generator_py__msg__Property__rosidl_typesupport_introspection_c__Property_message_member_array[2] = {
  {
    "property",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_generator_py__msg__Property, property),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "anything",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosidl_generator_py__msg__Property, anything),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers rosidl_generator_py__msg__Property__rosidl_typesupport_introspection_c__Property_message_members = {
  "rosidl_generator_py__msg",  // message namespace
  "Property",  // message name
  2,  // number of fields
  sizeof(rosidl_generator_py__msg__Property),
  false,  // has_any_key_member_
  rosidl_generator_py__msg__Property__rosidl_typesupport_introspection_c__Property_message_member_array,  // message members
  rosidl_generator_py__msg__Property__rosidl_typesupport_introspection_c__Property_init_function,  // function to initialize message memory (memory has to be allocated)
  rosidl_generator_py__msg__Property__rosidl_typesupport_introspection_c__Property_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t rosidl_generator_py__msg__Property__rosidl_typesupport_introspection_c__Property_message_type_support_handle = {
  0,
  &rosidl_generator_py__msg__Property__rosidl_typesupport_introspection_c__Property_message_members,
  get_message_typesupport_handle_function,
  &rosidl_generator_py__msg__Property__get_type_hash,
  &rosidl_generator_py__msg__Property__get_type_description,
  &rosidl_generator_py__msg__Property__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_rosidl_generator_py
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, rosidl_generator_py, msg, Property)() {
  if (!rosidl_generator_py__msg__Property__rosidl_typesupport_introspection_c__Property_message_type_support_handle.typesupport_identifier) {
    rosidl_generator_py__msg__Property__rosidl_typesupport_introspection_c__Property_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &rosidl_generator_py__msg__Property__rosidl_typesupport_introspection_c__Property_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
