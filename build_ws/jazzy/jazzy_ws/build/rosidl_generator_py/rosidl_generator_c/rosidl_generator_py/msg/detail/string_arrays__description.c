// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from rosidl_generator_py:msg/StringArrays.idl
// generated code does not contain a copyright notice

#include "rosidl_generator_py/msg/detail/string_arrays__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_rosidl_generator_py
const rosidl_type_hash_t *
rosidl_generator_py__msg__StringArrays__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x69, 0x6f, 0xfe, 0xa1, 0x99, 0xbf, 0xa1, 0xd6,
      0xfb, 0x01, 0xdd, 0x1b, 0xae, 0x69, 0xd3, 0xd7,
      0x37, 0x36, 0x90, 0xb6, 0xe1, 0x18, 0xd9, 0x0a,
      0x25, 0xcb, 0x69, 0x33, 0x20, 0x86, 0xd6, 0x11,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char rosidl_generator_py__msg__StringArrays__TYPE_NAME[] = "rosidl_generator_py/msg/StringArrays";

// Define type names, field names, and default values
static char rosidl_generator_py__msg__StringArrays__FIELD_NAME__ub_string_static_array_value[] = "ub_string_static_array_value";
static char rosidl_generator_py__msg__StringArrays__FIELD_NAME__ub_string_ub_array_value[] = "ub_string_ub_array_value";
static char rosidl_generator_py__msg__StringArrays__FIELD_NAME__ub_string_dynamic_array_value[] = "ub_string_dynamic_array_value";
static char rosidl_generator_py__msg__StringArrays__FIELD_NAME__string_dynamic_array_value[] = "string_dynamic_array_value";
static char rosidl_generator_py__msg__StringArrays__FIELD_NAME__string_static_array_value[] = "string_static_array_value";
static char rosidl_generator_py__msg__StringArrays__FIELD_NAME__string_bounded_array_value[] = "string_bounded_array_value";
static char rosidl_generator_py__msg__StringArrays__FIELD_NAME__def_string_dynamic_array_value[] = "def_string_dynamic_array_value";
static char rosidl_generator_py__msg__StringArrays__DEFAULT_VALUE__def_string_dynamic_array_value[] = "('What', 'a', 'wonderful', 'world', '!')";
static char rosidl_generator_py__msg__StringArrays__FIELD_NAME__def_string_static_array_value[] = "def_string_static_array_value";
static char rosidl_generator_py__msg__StringArrays__DEFAULT_VALUE__def_string_static_array_value[] = "('Hello', 'World', '!')";
static char rosidl_generator_py__msg__StringArrays__FIELD_NAME__def_string_bounded_array_value[] = "def_string_bounded_array_value";
static char rosidl_generator_py__msg__StringArrays__DEFAULT_VALUE__def_string_bounded_array_value[] = "('Hello', 'World', '!')";
static char rosidl_generator_py__msg__StringArrays__FIELD_NAME__def_various_quotes[] = "def_various_quotes";
static char rosidl_generator_py__msg__StringArrays__DEFAULT_VALUE__def_various_quotes[] = "(\\'H\"el\\\\\\'lo\\', \\'Wo\\\\\\'r\"ld\\')";
static char rosidl_generator_py__msg__StringArrays__FIELD_NAME__def_various_commas[] = "def_various_commas";
static char rosidl_generator_py__msg__StringArrays__DEFAULT_VALUE__def_various_commas[] = "('Hel,lo', ',World', 'abcd', '!,')";

static rosidl_runtime_c__type_description__Field rosidl_generator_py__msg__StringArrays__FIELDS[] = {
  {
    {rosidl_generator_py__msg__StringArrays__FIELD_NAME__ub_string_static_array_value, 28, 28},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOUNDED_STRING_ARRAY,
      3,
      5,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__StringArrays__FIELD_NAME__ub_string_ub_array_value, 24, 24},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOUNDED_STRING_BOUNDED_SEQUENCE,
      10,
      5,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__StringArrays__FIELD_NAME__ub_string_dynamic_array_value, 29, 29},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOUNDED_STRING_UNBOUNDED_SEQUENCE,
      0,
      5,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__StringArrays__FIELD_NAME__string_dynamic_array_value, 26, 26},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__StringArrays__FIELD_NAME__string_static_array_value, 25, 25},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__StringArrays__FIELD_NAME__string_bounded_array_value, 26, 26},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_BOUNDED_SEQUENCE,
      10,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__StringArrays__FIELD_NAME__def_string_dynamic_array_value, 30, 30},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {rosidl_generator_py__msg__StringArrays__DEFAULT_VALUE__def_string_dynamic_array_value, 40, 40},
  },
  {
    {rosidl_generator_py__msg__StringArrays__FIELD_NAME__def_string_static_array_value, 29, 29},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {rosidl_generator_py__msg__StringArrays__DEFAULT_VALUE__def_string_static_array_value, 23, 23},
  },
  {
    {rosidl_generator_py__msg__StringArrays__FIELD_NAME__def_string_bounded_array_value, 30, 30},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_BOUNDED_SEQUENCE,
      10,
      0,
      {NULL, 0, 0},
    },
    {rosidl_generator_py__msg__StringArrays__DEFAULT_VALUE__def_string_bounded_array_value, 23, 23},
  },
  {
    {rosidl_generator_py__msg__StringArrays__FIELD_NAME__def_various_quotes, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {rosidl_generator_py__msg__StringArrays__DEFAULT_VALUE__def_various_quotes, 24, 24},
  },
  {
    {rosidl_generator_py__msg__StringArrays__FIELD_NAME__def_various_commas, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {rosidl_generator_py__msg__StringArrays__DEFAULT_VALUE__def_various_commas, 34, 34},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
rosidl_generator_py__msg__StringArrays__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {rosidl_generator_py__msg__StringArrays__TYPE_NAME, 36, 36},
      {rosidl_generator_py__msg__StringArrays__FIELDS, 11, 11},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "string<=5[3] ub_string_static_array_value\n"
  "string<=5[<=10] ub_string_ub_array_value\n"
  "string<=5[] ub_string_dynamic_array_value\n"
  "string[] string_dynamic_array_value\n"
  "string[3] string_static_array_value\n"
  "string[<=10] string_bounded_array_value\n"
  "string[] def_string_dynamic_array_value [\"What\", \"a\", \"wonderful\", \"world\", \"!\"]\n"
  "string[3] def_string_static_array_value [\"Hello\", \"World\", \"!\"]\n"
  "string[<=10] def_string_bounded_array_value [\\'Hello\\', \\'World\\', \"!\"]\n"
  "string[] def_various_quotes [\"H\\\\\"el\\'lo\", \\'Wo\\\\\\'r\"ld\\']\n"
  "string[] def_various_commas [\"Hel,lo\", \\',World\\', abcd , \"!,\",]";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
rosidl_generator_py__msg__StringArrays__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {rosidl_generator_py__msg__StringArrays__TYPE_NAME, 36, 36},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 566, 566},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
rosidl_generator_py__msg__StringArrays__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *rosidl_generator_py__msg__StringArrays__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
