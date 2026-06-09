// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from rosidl_generator_py:msg/Defaults.idl
// generated code does not contain a copyright notice

#include "rosidl_generator_py/msg/detail/defaults__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_rosidl_generator_py
const rosidl_type_hash_t *
rosidl_generator_py__msg__Defaults__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x30, 0xb7, 0xc7, 0x32, 0xca, 0x3c, 0x39, 0xfc,
      0x6e, 0x24, 0x5d, 0x82, 0xea, 0x86, 0x8f, 0x83,
      0x6b, 0x66, 0x77, 0xc1, 0x3a, 0x50, 0x57, 0x17,
      0xfb, 0x02, 0xd7, 0xf0, 0xf5, 0xb1, 0xe7, 0x3d,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char rosidl_generator_py__msg__Defaults__TYPE_NAME[] = "rosidl_generator_py/msg/Defaults";

// Define type names, field names, and default values
static char rosidl_generator_py__msg__Defaults__FIELD_NAME__bool_value[] = "bool_value";
static char rosidl_generator_py__msg__Defaults__DEFAULT_VALUE__bool_value[] = "True";
static char rosidl_generator_py__msg__Defaults__FIELD_NAME__byte_value[] = "byte_value";
static char rosidl_generator_py__msg__Defaults__DEFAULT_VALUE__byte_value[] = "50";
static char rosidl_generator_py__msg__Defaults__FIELD_NAME__char_value[] = "char_value";
static char rosidl_generator_py__msg__Defaults__DEFAULT_VALUE__char_value[] = "100";
static char rosidl_generator_py__msg__Defaults__FIELD_NAME__float32_value[] = "float32_value";
static char rosidl_generator_py__msg__Defaults__DEFAULT_VALUE__float32_value[] = "1.125";
static char rosidl_generator_py__msg__Defaults__FIELD_NAME__float64_value[] = "float64_value";
static char rosidl_generator_py__msg__Defaults__DEFAULT_VALUE__float64_value[] = "1.125";
static char rosidl_generator_py__msg__Defaults__FIELD_NAME__int8_value[] = "int8_value";
static char rosidl_generator_py__msg__Defaults__DEFAULT_VALUE__int8_value[] = "-50";
static char rosidl_generator_py__msg__Defaults__FIELD_NAME__uint8_value[] = "uint8_value";
static char rosidl_generator_py__msg__Defaults__DEFAULT_VALUE__uint8_value[] = "200";
static char rosidl_generator_py__msg__Defaults__FIELD_NAME__int16_value[] = "int16_value";
static char rosidl_generator_py__msg__Defaults__DEFAULT_VALUE__int16_value[] = "-1000";
static char rosidl_generator_py__msg__Defaults__FIELD_NAME__uint16_value[] = "uint16_value";
static char rosidl_generator_py__msg__Defaults__DEFAULT_VALUE__uint16_value[] = "2000";
static char rosidl_generator_py__msg__Defaults__FIELD_NAME__int32_value[] = "int32_value";
static char rosidl_generator_py__msg__Defaults__DEFAULT_VALUE__int32_value[] = "-30000";
static char rosidl_generator_py__msg__Defaults__FIELD_NAME__uint32_value[] = "uint32_value";
static char rosidl_generator_py__msg__Defaults__DEFAULT_VALUE__uint32_value[] = "60000";
static char rosidl_generator_py__msg__Defaults__FIELD_NAME__int64_value[] = "int64_value";
static char rosidl_generator_py__msg__Defaults__DEFAULT_VALUE__int64_value[] = "-40000000";
static char rosidl_generator_py__msg__Defaults__FIELD_NAME__uint64_value[] = "uint64_value";
static char rosidl_generator_py__msg__Defaults__DEFAULT_VALUE__uint64_value[] = "50000000";

static rosidl_runtime_c__type_description__Field rosidl_generator_py__msg__Defaults__FIELDS[] = {
  {
    {rosidl_generator_py__msg__Defaults__FIELD_NAME__bool_value, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {rosidl_generator_py__msg__Defaults__DEFAULT_VALUE__bool_value, 4, 4},
  },
  {
    {rosidl_generator_py__msg__Defaults__FIELD_NAME__byte_value, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BYTE,
      0,
      0,
      {NULL, 0, 0},
    },
    {rosidl_generator_py__msg__Defaults__DEFAULT_VALUE__byte_value, 2, 2},
  },
  {
    {rosidl_generator_py__msg__Defaults__FIELD_NAME__char_value, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {rosidl_generator_py__msg__Defaults__DEFAULT_VALUE__char_value, 3, 3},
  },
  {
    {rosidl_generator_py__msg__Defaults__FIELD_NAME__float32_value, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {rosidl_generator_py__msg__Defaults__DEFAULT_VALUE__float32_value, 5, 5},
  },
  {
    {rosidl_generator_py__msg__Defaults__FIELD_NAME__float64_value, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {rosidl_generator_py__msg__Defaults__DEFAULT_VALUE__float64_value, 5, 5},
  },
  {
    {rosidl_generator_py__msg__Defaults__FIELD_NAME__int8_value, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {rosidl_generator_py__msg__Defaults__DEFAULT_VALUE__int8_value, 3, 3},
  },
  {
    {rosidl_generator_py__msg__Defaults__FIELD_NAME__uint8_value, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {rosidl_generator_py__msg__Defaults__DEFAULT_VALUE__uint8_value, 3, 3},
  },
  {
    {rosidl_generator_py__msg__Defaults__FIELD_NAME__int16_value, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {rosidl_generator_py__msg__Defaults__DEFAULT_VALUE__int16_value, 5, 5},
  },
  {
    {rosidl_generator_py__msg__Defaults__FIELD_NAME__uint16_value, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {rosidl_generator_py__msg__Defaults__DEFAULT_VALUE__uint16_value, 4, 4},
  },
  {
    {rosidl_generator_py__msg__Defaults__FIELD_NAME__int32_value, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {rosidl_generator_py__msg__Defaults__DEFAULT_VALUE__int32_value, 6, 6},
  },
  {
    {rosidl_generator_py__msg__Defaults__FIELD_NAME__uint32_value, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {rosidl_generator_py__msg__Defaults__DEFAULT_VALUE__uint32_value, 5, 5},
  },
  {
    {rosidl_generator_py__msg__Defaults__FIELD_NAME__int64_value, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {rosidl_generator_py__msg__Defaults__DEFAULT_VALUE__int64_value, 9, 9},
  },
  {
    {rosidl_generator_py__msg__Defaults__FIELD_NAME__uint64_value, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {rosidl_generator_py__msg__Defaults__DEFAULT_VALUE__uint64_value, 8, 8},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
rosidl_generator_py__msg__Defaults__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {rosidl_generator_py__msg__Defaults__TYPE_NAME, 32, 32},
      {rosidl_generator_py__msg__Defaults__FIELDS, 13, 13},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "bool bool_value true\n"
  "byte byte_value 50\n"
  "char char_value 100\n"
  "float32 float32_value 1.125\n"
  "float64 float64_value 1.125\n"
  "int8 int8_value -50\n"
  "uint8 uint8_value 200\n"
  "int16 int16_value -1000\n"
  "uint16 uint16_value 2000\n"
  "int32 int32_value -30000\n"
  "uint32 uint32_value 60000\n"
  "int64 int64_value -40000000\n"
  "uint64 uint64_value 50000000";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
rosidl_generator_py__msg__Defaults__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {rosidl_generator_py__msg__Defaults__TYPE_NAME, 32, 32},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 315, 315},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
rosidl_generator_py__msg__Defaults__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *rosidl_generator_py__msg__Defaults__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
