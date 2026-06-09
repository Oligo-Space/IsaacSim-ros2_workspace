// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from rosidl_generator_py:msg/Nested.idl
// generated code does not contain a copyright notice

#include "rosidl_generator_py/msg/detail/nested__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_rosidl_generator_py
const rosidl_type_hash_t *
rosidl_generator_py__msg__Nested__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x14, 0x29, 0x94, 0x92, 0x2a, 0xc9, 0x97, 0xf8,
      0x52, 0x7f, 0xa6, 0xaa, 0x56, 0xc8, 0x78, 0xe8,
      0xc5, 0x64, 0x0b, 0xfc, 0x26, 0x0b, 0x07, 0xfd,
      0x0d, 0x43, 0x60, 0xb3, 0x66, 0x31, 0x10, 0x5d,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "rosidl_generator_py/msg/detail/basic_types__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t rosidl_generator_py__msg__BasicTypes__EXPECTED_HASH = {1, {
    0x4b, 0xc8, 0x63, 0x64, 0xfa, 0x62, 0x1e, 0xb4,
    0x18, 0x68, 0xe8, 0x97, 0xe0, 0x42, 0xbb, 0x03,
    0xed, 0x6c, 0xfc, 0x64, 0xc9, 0x5d, 0x10, 0xe2,
    0xe3, 0x85, 0x41, 0x7d, 0xb9, 0x5f, 0x46, 0x76,
  }};
#endif

static char rosidl_generator_py__msg__Nested__TYPE_NAME[] = "rosidl_generator_py/msg/Nested";
static char rosidl_generator_py__msg__BasicTypes__TYPE_NAME[] = "rosidl_generator_py/msg/BasicTypes";

// Define type names, field names, and default values
static char rosidl_generator_py__msg__Nested__FIELD_NAME__basic_types_value[] = "basic_types_value";

static rosidl_runtime_c__type_description__Field rosidl_generator_py__msg__Nested__FIELDS[] = {
  {
    {rosidl_generator_py__msg__Nested__FIELD_NAME__basic_types_value, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {rosidl_generator_py__msg__BasicTypes__TYPE_NAME, 34, 34},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription rosidl_generator_py__msg__Nested__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {rosidl_generator_py__msg__BasicTypes__TYPE_NAME, 34, 34},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
rosidl_generator_py__msg__Nested__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {rosidl_generator_py__msg__Nested__TYPE_NAME, 30, 30},
      {rosidl_generator_py__msg__Nested__FIELDS, 1, 1},
    },
    {rosidl_generator_py__msg__Nested__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&rosidl_generator_py__msg__BasicTypes__EXPECTED_HASH, rosidl_generator_py__msg__BasicTypes__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = rosidl_generator_py__msg__BasicTypes__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "BasicTypes basic_types_value";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
rosidl_generator_py__msg__Nested__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {rosidl_generator_py__msg__Nested__TYPE_NAME, 30, 30},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 29, 29},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
rosidl_generator_py__msg__Nested__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *rosidl_generator_py__msg__Nested__get_individual_type_description_source(NULL),
    sources[1] = *rosidl_generator_py__msg__BasicTypes__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
