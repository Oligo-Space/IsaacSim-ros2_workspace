// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from rosidl_generator_py:msg/Empty.idl
// generated code does not contain a copyright notice

#include "rosidl_generator_py/msg/detail/empty__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_rosidl_generator_py
const rosidl_type_hash_t *
rosidl_generator_py__msg__Empty__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x8f, 0xb7, 0xec, 0x6c, 0x81, 0x88, 0x76, 0x70,
      0x78, 0xe7, 0x3a, 0xac, 0x74, 0xad, 0x6f, 0x7d,
      0xbf, 0x58, 0xdd, 0x34, 0xb8, 0x31, 0x90, 0x59,
      0xba, 0xdb, 0xf6, 0x36, 0x40, 0xc4, 0x99, 0x75,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char rosidl_generator_py__msg__Empty__TYPE_NAME[] = "rosidl_generator_py/msg/Empty";

// Define type names, field names, and default values
static char rosidl_generator_py__msg__Empty__FIELD_NAME__structure_needs_at_least_one_member[] = "structure_needs_at_least_one_member";

static rosidl_runtime_c__type_description__Field rosidl_generator_py__msg__Empty__FIELDS[] = {
  {
    {rosidl_generator_py__msg__Empty__FIELD_NAME__structure_needs_at_least_one_member, 35, 35},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
rosidl_generator_py__msg__Empty__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {rosidl_generator_py__msg__Empty__TYPE_NAME, 29, 29},
      {rosidl_generator_py__msg__Empty__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}


static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
rosidl_generator_py__msg__Empty__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {rosidl_generator_py__msg__Empty__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
rosidl_generator_py__msg__Empty__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *rosidl_generator_py__msg__Empty__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
