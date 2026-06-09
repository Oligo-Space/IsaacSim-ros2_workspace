// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from rosidl_generator_py:msg/BuiltinTypeSequencesIdl.idl
// generated code does not contain a copyright notice

#include "rosidl_generator_py/msg/detail/builtin_type_sequences_idl__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_rosidl_generator_py
const rosidl_type_hash_t *
rosidl_generator_py__msg__BuiltinTypeSequencesIdl__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xc5, 0x9f, 0x16, 0xb8, 0x75, 0x0b, 0x32, 0x93,
      0x77, 0x84, 0x81, 0x79, 0xfb, 0x60, 0xcc, 0x9f,
      0x53, 0x7d, 0x3c, 0x34, 0xbf, 0x2b, 0x55, 0x61,
      0xa4, 0xcd, 0x17, 0x35, 0xc4, 0x7b, 0x48, 0x27,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char rosidl_generator_py__msg__BuiltinTypeSequencesIdl__TYPE_NAME[] = "rosidl_generator_py/msg/BuiltinTypeSequencesIdl";

// Define type names, field names, and default values
static char rosidl_generator_py__msg__BuiltinTypeSequencesIdl__FIELD_NAME__char_sequence_unbounded[] = "char_sequence_unbounded";

static rosidl_runtime_c__type_description__Field rosidl_generator_py__msg__BuiltinTypeSequencesIdl__FIELDS[] = {
  {
    {rosidl_generator_py__msg__BuiltinTypeSequencesIdl__FIELD_NAME__char_sequence_unbounded, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_CHAR_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
rosidl_generator_py__msg__BuiltinTypeSequencesIdl__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {rosidl_generator_py__msg__BuiltinTypeSequencesIdl__TYPE_NAME, 47, 47},
      {rosidl_generator_py__msg__BuiltinTypeSequencesIdl__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "module rosidl_generator_py {\n"
  "    module msg {\n"
  "        struct BuiltinTypeSequencesIdl {\n"
  "            // Unbounded sequences\n"
  "            sequence<char> char_sequence_unbounded;\n"
  "        };\n"
  "    };\n"
  "};";

static char idl_encoding[] = "idl";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
rosidl_generator_py__msg__BuiltinTypeSequencesIdl__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {rosidl_generator_py__msg__BuiltinTypeSequencesIdl__TYPE_NAME, 47, 47},
    {idl_encoding, 3, 3},
    {toplevel_type_raw_source, 195, 195},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
rosidl_generator_py__msg__BuiltinTypeSequencesIdl__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *rosidl_generator_py__msg__BuiltinTypeSequencesIdl__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
