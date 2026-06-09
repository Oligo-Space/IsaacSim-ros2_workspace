// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from rosidl_generator_py:msg/BasicTypes.idl
// generated code does not contain a copyright notice

#include "rosidl_generator_py/msg/detail/basic_types__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_rosidl_generator_py
const rosidl_type_hash_t *
rosidl_generator_py__msg__BasicTypes__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x4b, 0xc8, 0x63, 0x64, 0xfa, 0x62, 0x1e, 0xb4,
      0x18, 0x68, 0xe8, 0x97, 0xe0, 0x42, 0xbb, 0x03,
      0xed, 0x6c, 0xfc, 0x64, 0xc9, 0x5d, 0x10, 0xe2,
      0xe3, 0x85, 0x41, 0x7d, 0xb9, 0x5f, 0x46, 0x76,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char rosidl_generator_py__msg__BasicTypes__TYPE_NAME[] = "rosidl_generator_py/msg/BasicTypes";

// Define type names, field names, and default values
static char rosidl_generator_py__msg__BasicTypes__FIELD_NAME__bool_value[] = "bool_value";
static char rosidl_generator_py__msg__BasicTypes__FIELD_NAME__byte_value[] = "byte_value";
static char rosidl_generator_py__msg__BasicTypes__FIELD_NAME__char_value[] = "char_value";
static char rosidl_generator_py__msg__BasicTypes__FIELD_NAME__float32_value[] = "float32_value";
static char rosidl_generator_py__msg__BasicTypes__FIELD_NAME__float64_value[] = "float64_value";
static char rosidl_generator_py__msg__BasicTypes__FIELD_NAME__int8_value[] = "int8_value";
static char rosidl_generator_py__msg__BasicTypes__FIELD_NAME__uint8_value[] = "uint8_value";
static char rosidl_generator_py__msg__BasicTypes__FIELD_NAME__int16_value[] = "int16_value";
static char rosidl_generator_py__msg__BasicTypes__FIELD_NAME__uint16_value[] = "uint16_value";
static char rosidl_generator_py__msg__BasicTypes__FIELD_NAME__int32_value[] = "int32_value";
static char rosidl_generator_py__msg__BasicTypes__FIELD_NAME__uint32_value[] = "uint32_value";
static char rosidl_generator_py__msg__BasicTypes__FIELD_NAME__int64_value[] = "int64_value";
static char rosidl_generator_py__msg__BasicTypes__FIELD_NAME__uint64_value[] = "uint64_value";

static rosidl_runtime_c__type_description__Field rosidl_generator_py__msg__BasicTypes__FIELDS[] = {
  {
    {rosidl_generator_py__msg__BasicTypes__FIELD_NAME__bool_value, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__BasicTypes__FIELD_NAME__byte_value, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BYTE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__BasicTypes__FIELD_NAME__char_value, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__BasicTypes__FIELD_NAME__float32_value, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__BasicTypes__FIELD_NAME__float64_value, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__BasicTypes__FIELD_NAME__int8_value, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__BasicTypes__FIELD_NAME__uint8_value, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__BasicTypes__FIELD_NAME__int16_value, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__BasicTypes__FIELD_NAME__uint16_value, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__BasicTypes__FIELD_NAME__int32_value, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__BasicTypes__FIELD_NAME__uint32_value, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__BasicTypes__FIELD_NAME__int64_value, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__BasicTypes__FIELD_NAME__uint64_value, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
rosidl_generator_py__msg__BasicTypes__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {rosidl_generator_py__msg__BasicTypes__TYPE_NAME, 34, 34},
      {rosidl_generator_py__msg__BasicTypes__FIELDS, 13, 13},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "bool bool_value\n"
  "byte byte_value\n"
  "char char_value\n"
  "float32 float32_value\n"
  "float64 float64_value\n"
  "int8 int8_value\n"
  "uint8 uint8_value\n"
  "int16 int16_value\n"
  "uint16 uint16_value\n"
  "int32 int32_value\n"
  "uint32 uint32_value\n"
  "int64 int64_value\n"
  "uint64 uint64_value";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
rosidl_generator_py__msg__BasicTypes__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {rosidl_generator_py__msg__BasicTypes__TYPE_NAME, 34, 34},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 240, 240},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
rosidl_generator_py__msg__BasicTypes__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *rosidl_generator_py__msg__BasicTypes__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
