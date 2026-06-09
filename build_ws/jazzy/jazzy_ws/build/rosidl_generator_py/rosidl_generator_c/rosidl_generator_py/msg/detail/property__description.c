// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from rosidl_generator_py:msg/Property.idl
// generated code does not contain a copyright notice

#include "rosidl_generator_py/msg/detail/property__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_rosidl_generator_py
const rosidl_type_hash_t *
rosidl_generator_py__msg__Property__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x86, 0x6b, 0xf9, 0x26, 0x3c, 0x65, 0x4a, 0x91,
      0x74, 0xcc, 0xe6, 0xcd, 0xcc, 0x62, 0x72, 0x12,
      0x6f, 0xd5, 0x83, 0xb5, 0xe4, 0x77, 0xf9, 0xb0,
      0x88, 0x80, 0xe5, 0x96, 0x94, 0x1c, 0x4d, 0x34,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char rosidl_generator_py__msg__Property__TYPE_NAME[] = "rosidl_generator_py/msg/Property";

// Define type names, field names, and default values
static char rosidl_generator_py__msg__Property__FIELD_NAME__property[] = "property";
static char rosidl_generator_py__msg__Property__FIELD_NAME__anything[] = "anything";

static rosidl_runtime_c__type_description__Field rosidl_generator_py__msg__Property__FIELDS[] = {
  {
    {rosidl_generator_py__msg__Property__FIELD_NAME__property, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__Property__FIELD_NAME__anything, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
rosidl_generator_py__msg__Property__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {rosidl_generator_py__msg__Property__TYPE_NAME, 32, 32},
      {rosidl_generator_py__msg__Property__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string property\n"
  "string anything";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
rosidl_generator_py__msg__Property__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {rosidl_generator_py__msg__Property__TYPE_NAME, 32, 32},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 32, 32},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
rosidl_generator_py__msg__Property__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *rosidl_generator_py__msg__Property__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
