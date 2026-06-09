// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from rosidl_generator_py:msg/WStrings.idl
// generated code does not contain a copyright notice

#include "rosidl_generator_py/msg/detail/w_strings__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_rosidl_generator_py
const rosidl_type_hash_t *
rosidl_generator_py__msg__WStrings__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe7, 0x5b, 0xe2, 0x79, 0xbc, 0x12, 0x72, 0xab,
      0xcf, 0x01, 0x2b, 0x95, 0x5d, 0x22, 0xea, 0x27,
      0xb0, 0x17, 0x6b, 0x68, 0xf8, 0x0c, 0x18, 0x3d,
      0xb7, 0x4a, 0x72, 0xae, 0x30, 0x16, 0xa7, 0x37,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char rosidl_generator_py__msg__WStrings__TYPE_NAME[] = "rosidl_generator_py/msg/WStrings";

// Define type names, field names, and default values
static char rosidl_generator_py__msg__WStrings__FIELD_NAME__wstring_value[] = "wstring_value";
static char rosidl_generator_py__msg__WStrings__FIELD_NAME__wstring_value_default1[] = "wstring_value_default1";
static char rosidl_generator_py__msg__WStrings__DEFAULT_VALUE__wstring_value_default1[] = "Hello world!";
static char rosidl_generator_py__msg__WStrings__FIELD_NAME__wstring_value_default2[] = "wstring_value_default2";
static char rosidl_generator_py__msg__WStrings__DEFAULT_VALUE__wstring_value_default2[] = "Hell\\xc3\\xb6 w\\xc3\\xb6rld!";
static char rosidl_generator_py__msg__WStrings__FIELD_NAME__wstring_value_default3[] = "wstring_value_default3";
static char rosidl_generator_py__msg__WStrings__DEFAULT_VALUE__wstring_value_default3[] = "\\xe3\\x83\\x8f\\xe3\\x83\\xad\\xe3\\x83\\xbc\\xe3\\x83\\xaf\\xe3\\x83\\xbc\\xe3\\x83\\xab\\xe3\\x83\\x89";
static char rosidl_generator_py__msg__WStrings__FIELD_NAME__array_of_wstrings[] = "array_of_wstrings";
static char rosidl_generator_py__msg__WStrings__FIELD_NAME__bounded_sequence_of_wstrings[] = "bounded_sequence_of_wstrings";
static char rosidl_generator_py__msg__WStrings__FIELD_NAME__unbounded_sequence_of_wstrings[] = "unbounded_sequence_of_wstrings";

static rosidl_runtime_c__type_description__Field rosidl_generator_py__msg__WStrings__FIELDS[] = {
  {
    {rosidl_generator_py__msg__WStrings__FIELD_NAME__wstring_value, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_WSTRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__WStrings__FIELD_NAME__wstring_value_default1, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_WSTRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {rosidl_generator_py__msg__WStrings__DEFAULT_VALUE__wstring_value_default1, 12, 12},
  },
  {
    {rosidl_generator_py__msg__WStrings__FIELD_NAME__wstring_value_default2, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_WSTRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {rosidl_generator_py__msg__WStrings__DEFAULT_VALUE__wstring_value_default2, 12, 12},
  },
  {
    {rosidl_generator_py__msg__WStrings__FIELD_NAME__wstring_value_default3, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_WSTRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {rosidl_generator_py__msg__WStrings__DEFAULT_VALUE__wstring_value_default3, 7, 7},
  },
  {
    {rosidl_generator_py__msg__WStrings__FIELD_NAME__array_of_wstrings, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_WSTRING_ARRAY,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__WStrings__FIELD_NAME__bounded_sequence_of_wstrings, 28, 28},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_WSTRING_BOUNDED_SEQUENCE,
      3,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__WStrings__FIELD_NAME__unbounded_sequence_of_wstrings, 30, 30},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_WSTRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
rosidl_generator_py__msg__WStrings__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {rosidl_generator_py__msg__WStrings__TYPE_NAME, 32, 32},
      {rosidl_generator_py__msg__WStrings__FIELDS, 7, 7},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "wstring wstring_value\n"
  "wstring wstring_value_default1 \"Hello world!\"\n"
  "wstring wstring_value_default2 \"Hell\\xc3\\xb6 w\\xc3\\xb6rld!\"\n"
  "wstring wstring_value_default3 \"\\xe3\\x83\\x8f\\xe3\\x83\\xad\\xe3\\x83\\xbc\\xe3\\x83\\xaf\\xe3\\x83\\xbc\\xe3\\x83\\xab\\xe3\\x83\\x89\"\n"
  "#wstring WSTRING_CONST=\"Hello world!\"\n"
  "#wstring<=22 bounded_wstring_value\n"
  "#wstring<=22 bounded_wstring_value_default1 \"Hello world!\"\n"
  "wstring[3] array_of_wstrings\n"
  "wstring[<=3] bounded_sequence_of_wstrings\n"
  "wstring[] unbounded_sequence_of_wstrings";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
rosidl_generator_py__msg__WStrings__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {rosidl_generator_py__msg__WStrings__TYPE_NAME, 32, 32},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 399, 399},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
rosidl_generator_py__msg__WStrings__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *rosidl_generator_py__msg__WStrings__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
