// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from rosidl_generator_py:msg/UnboundedSequences.idl
// generated code does not contain a copyright notice

#include "rosidl_generator_py/msg/detail/unbounded_sequences__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_rosidl_generator_py
const rosidl_type_hash_t *
rosidl_generator_py__msg__UnboundedSequences__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x36, 0x58, 0x1e, 0x14, 0xfc, 0xc1, 0xac, 0x6f,
      0x4c, 0x7d, 0x52, 0x4a, 0xc5, 0xce, 0x01, 0xa1,
      0xd6, 0x6c, 0x84, 0x05, 0x1e, 0x1f, 0x93, 0x61,
      0xee, 0x0c, 0xc6, 0x6f, 0x9b, 0xf9, 0x56, 0xbf,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "rosidl_generator_py/msg/detail/basic_types__functions.h"
#include "rosidl_generator_py/msg/detail/constants__functions.h"
#include "rosidl_generator_py/msg/detail/defaults__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t rosidl_generator_py__msg__BasicTypes__EXPECTED_HASH = {1, {
    0x4b, 0xc8, 0x63, 0x64, 0xfa, 0x62, 0x1e, 0xb4,
    0x18, 0x68, 0xe8, 0x97, 0xe0, 0x42, 0xbb, 0x03,
    0xed, 0x6c, 0xfc, 0x64, 0xc9, 0x5d, 0x10, 0xe2,
    0xe3, 0x85, 0x41, 0x7d, 0xb9, 0x5f, 0x46, 0x76,
  }};
static const rosidl_type_hash_t rosidl_generator_py__msg__Constants__EXPECTED_HASH = {1, {
    0xf2, 0xbf, 0xd5, 0xaf, 0x9b, 0x3a, 0xdb, 0x20,
    0xa2, 0x13, 0x7b, 0xc1, 0x87, 0xec, 0x97, 0x2b,
    0xb3, 0x92, 0xb7, 0x3e, 0xff, 0x64, 0x58, 0x38,
    0x3b, 0x94, 0xd2, 0xa2, 0xa4, 0x0d, 0x60, 0xd0,
  }};
static const rosidl_type_hash_t rosidl_generator_py__msg__Defaults__EXPECTED_HASH = {1, {
    0x30, 0xb7, 0xc7, 0x32, 0xca, 0x3c, 0x39, 0xfc,
    0x6e, 0x24, 0x5d, 0x82, 0xea, 0x86, 0x8f, 0x83,
    0x6b, 0x66, 0x77, 0xc1, 0x3a, 0x50, 0x57, 0x17,
    0xfb, 0x02, 0xd7, 0xf0, 0xf5, 0xb1, 0xe7, 0x3d,
  }};
#endif

static char rosidl_generator_py__msg__UnboundedSequences__TYPE_NAME[] = "rosidl_generator_py/msg/UnboundedSequences";
static char rosidl_generator_py__msg__BasicTypes__TYPE_NAME[] = "rosidl_generator_py/msg/BasicTypes";
static char rosidl_generator_py__msg__Constants__TYPE_NAME[] = "rosidl_generator_py/msg/Constants";
static char rosidl_generator_py__msg__Defaults__TYPE_NAME[] = "rosidl_generator_py/msg/Defaults";

// Define type names, field names, and default values
static char rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__bool_values[] = "bool_values";
static char rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__byte_values[] = "byte_values";
static char rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__char_values[] = "char_values";
static char rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__float32_values[] = "float32_values";
static char rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__float64_values[] = "float64_values";
static char rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__int8_values[] = "int8_values";
static char rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__uint8_values[] = "uint8_values";
static char rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__int16_values[] = "int16_values";
static char rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__uint16_values[] = "uint16_values";
static char rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__int32_values[] = "int32_values";
static char rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__uint32_values[] = "uint32_values";
static char rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__int64_values[] = "int64_values";
static char rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__uint64_values[] = "uint64_values";
static char rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__string_values[] = "string_values";
static char rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__basic_types_values[] = "basic_types_values";
static char rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__constants_values[] = "constants_values";
static char rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__defaults_values[] = "defaults_values";
static char rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__bool_values_default[] = "bool_values_default";
static char rosidl_generator_py__msg__UnboundedSequences__DEFAULT_VALUE__bool_values_default[] = "(False, True, False)";
static char rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__byte_values_default[] = "byte_values_default";
static char rosidl_generator_py__msg__UnboundedSequences__DEFAULT_VALUE__byte_values_default[] = "(0, 1, 255)";
static char rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__char_values_default[] = "char_values_default";
static char rosidl_generator_py__msg__UnboundedSequences__DEFAULT_VALUE__char_values_default[] = "(0, 1, 127)";
static char rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__float32_values_default[] = "float32_values_default";
static char rosidl_generator_py__msg__UnboundedSequences__DEFAULT_VALUE__float32_values_default[] = "(1.125, 0.0, -1.125)";
static char rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__float64_values_default[] = "float64_values_default";
static char rosidl_generator_py__msg__UnboundedSequences__DEFAULT_VALUE__float64_values_default[] = "(3.1415, 0.0, -3.1415)";
static char rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__int8_values_default[] = "int8_values_default";
static char rosidl_generator_py__msg__UnboundedSequences__DEFAULT_VALUE__int8_values_default[] = "(0, 127, -128)";
static char rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__uint8_values_default[] = "uint8_values_default";
static char rosidl_generator_py__msg__UnboundedSequences__DEFAULT_VALUE__uint8_values_default[] = "(0, 1, 255)";
static char rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__int16_values_default[] = "int16_values_default";
static char rosidl_generator_py__msg__UnboundedSequences__DEFAULT_VALUE__int16_values_default[] = "(0, 32767, -32768)";
static char rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__uint16_values_default[] = "uint16_values_default";
static char rosidl_generator_py__msg__UnboundedSequences__DEFAULT_VALUE__uint16_values_default[] = "(0, 1, 65535)";
static char rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__int32_values_default[] = "int32_values_default";
static char rosidl_generator_py__msg__UnboundedSequences__DEFAULT_VALUE__int32_values_default[] = "(0, 2147483647, -2147483648)";
static char rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__uint32_values_default[] = "uint32_values_default";
static char rosidl_generator_py__msg__UnboundedSequences__DEFAULT_VALUE__uint32_values_default[] = "(0, 1, 4294967295)";
static char rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__int64_values_default[] = "int64_values_default";
static char rosidl_generator_py__msg__UnboundedSequences__DEFAULT_VALUE__int64_values_default[] = "(0, 9223372036854775807, -9223372036854775808)";
static char rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__uint64_values_default[] = "uint64_values_default";
static char rosidl_generator_py__msg__UnboundedSequences__DEFAULT_VALUE__uint64_values_default[] = "(0, 1, 18446744073709551615)";
static char rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__string_values_default[] = "string_values_default";
static char rosidl_generator_py__msg__UnboundedSequences__DEFAULT_VALUE__string_values_default[] = "('', 'max value', 'min value')";
static char rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__alignment_check[] = "alignment_check";

static rosidl_runtime_c__type_description__Field rosidl_generator_py__msg__UnboundedSequences__FIELDS[] = {
  {
    {rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__bool_values, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__byte_values, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BYTE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__char_values, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__float32_values, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__float64_values, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__int8_values, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__uint8_values, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__int16_values, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__uint16_values, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__int32_values, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__uint32_values, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__int64_values, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT64_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__uint64_values, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__string_values, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__basic_types_values, 18, 18},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {rosidl_generator_py__msg__BasicTypes__TYPE_NAME, 34, 34},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__constants_values, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {rosidl_generator_py__msg__Constants__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__defaults_values, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {rosidl_generator_py__msg__Defaults__TYPE_NAME, 32, 32},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__bool_values_default, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {rosidl_generator_py__msg__UnboundedSequences__DEFAULT_VALUE__bool_values_default, 20, 20},
  },
  {
    {rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__byte_values_default, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BYTE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {rosidl_generator_py__msg__UnboundedSequences__DEFAULT_VALUE__byte_values_default, 11, 11},
  },
  {
    {rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__char_values_default, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {rosidl_generator_py__msg__UnboundedSequences__DEFAULT_VALUE__char_values_default, 11, 11},
  },
  {
    {rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__float32_values_default, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {rosidl_generator_py__msg__UnboundedSequences__DEFAULT_VALUE__float32_values_default, 20, 20},
  },
  {
    {rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__float64_values_default, 22, 22},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {rosidl_generator_py__msg__UnboundedSequences__DEFAULT_VALUE__float64_values_default, 22, 22},
  },
  {
    {rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__int8_values_default, 19, 19},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT8_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {rosidl_generator_py__msg__UnboundedSequences__DEFAULT_VALUE__int8_values_default, 14, 14},
  },
  {
    {rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__uint8_values_default, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {rosidl_generator_py__msg__UnboundedSequences__DEFAULT_VALUE__uint8_values_default, 11, 11},
  },
  {
    {rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__int16_values_default, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT16_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {rosidl_generator_py__msg__UnboundedSequences__DEFAULT_VALUE__int16_values_default, 18, 18},
  },
  {
    {rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__uint16_values_default, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT16_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {rosidl_generator_py__msg__UnboundedSequences__DEFAULT_VALUE__uint16_values_default, 13, 13},
  },
  {
    {rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__int32_values_default, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {rosidl_generator_py__msg__UnboundedSequences__DEFAULT_VALUE__int32_values_default, 28, 28},
  },
  {
    {rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__uint32_values_default, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {rosidl_generator_py__msg__UnboundedSequences__DEFAULT_VALUE__uint32_values_default, 18, 18},
  },
  {
    {rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__int64_values_default, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT64_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {rosidl_generator_py__msg__UnboundedSequences__DEFAULT_VALUE__int64_values_default, 46, 46},
  },
  {
    {rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__uint64_values_default, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {rosidl_generator_py__msg__UnboundedSequences__DEFAULT_VALUE__uint64_values_default, 28, 28},
  },
  {
    {rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__string_values_default, 21, 21},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {rosidl_generator_py__msg__UnboundedSequences__DEFAULT_VALUE__string_values_default, 30, 30},
  },
  {
    {rosidl_generator_py__msg__UnboundedSequences__FIELD_NAME__alignment_check, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription rosidl_generator_py__msg__UnboundedSequences__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {rosidl_generator_py__msg__BasicTypes__TYPE_NAME, 34, 34},
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__Constants__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__Defaults__TYPE_NAME, 32, 32},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
rosidl_generator_py__msg__UnboundedSequences__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {rosidl_generator_py__msg__UnboundedSequences__TYPE_NAME, 42, 42},
      {rosidl_generator_py__msg__UnboundedSequences__FIELDS, 32, 32},
    },
    {rosidl_generator_py__msg__UnboundedSequences__REFERENCED_TYPE_DESCRIPTIONS, 3, 3},
  };
  if (!constructed) {
    assert(0 == memcmp(&rosidl_generator_py__msg__BasicTypes__EXPECTED_HASH, rosidl_generator_py__msg__BasicTypes__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = rosidl_generator_py__msg__BasicTypes__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&rosidl_generator_py__msg__Constants__EXPECTED_HASH, rosidl_generator_py__msg__Constants__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = rosidl_generator_py__msg__Constants__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&rosidl_generator_py__msg__Defaults__EXPECTED_HASH, rosidl_generator_py__msg__Defaults__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = rosidl_generator_py__msg__Defaults__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Unbounded sequences of different types\n"
  "bool[] bool_values\n"
  "byte[] byte_values\n"
  "char[] char_values\n"
  "float32[] float32_values\n"
  "float64[] float64_values\n"
  "int8[] int8_values\n"
  "uint8[] uint8_values\n"
  "int16[] int16_values\n"
  "uint16[] uint16_values\n"
  "int32[] int32_values\n"
  "uint32[] uint32_values\n"
  "int64[] int64_values\n"
  "uint64[] uint64_values\n"
  "string[] string_values\n"
  "BasicTypes[] basic_types_values\n"
  "Constants[] constants_values\n"
  "Defaults[] defaults_values\n"
  "bool[] bool_values_default [false, true, false]\n"
  "byte[] byte_values_default [0, 1, 255]\n"
  "char[] char_values_default [0, 1, 127]\n"
  "float32[] float32_values_default [1.125, 0.0, -1.125]\n"
  "float64[] float64_values_default [3.1415, 0.0, -3.1415]\n"
  "int8[] int8_values_default [0, 127, -128]\n"
  "uint8[] uint8_values_default [0, 1, 255]\n"
  "int16[] int16_values_default [0, 32767, -32768]\n"
  "uint16[] uint16_values_default [0, 1, 65535]\n"
  "int32[] int32_values_default [0, 2147483647, -2147483648]\n"
  "uint32[] uint32_values_default [0, 1, 4294967295]\n"
  "int64[] int64_values_default [0, 9223372036854775807, -9223372036854775808]\n"
  "uint64[] uint64_values_default [0, 1, 18446744073709551615]\n"
  "string[] string_values_default [\"\", \"max value\", \"min value\"]\n"
  "# Regression test: check alignment of basic field after a sequence field is correct\n"
  "int32 alignment_check";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
rosidl_generator_py__msg__UnboundedSequences__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {rosidl_generator_py__msg__UnboundedSequences__TYPE_NAME, 42, 42},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1255, 1255},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
rosidl_generator_py__msg__UnboundedSequences__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[4];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 4, 4};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *rosidl_generator_py__msg__UnboundedSequences__get_individual_type_description_source(NULL),
    sources[1] = *rosidl_generator_py__msg__BasicTypes__get_individual_type_description_source(NULL);
    sources[2] = *rosidl_generator_py__msg__Constants__get_individual_type_description_source(NULL);
    sources[3] = *rosidl_generator_py__msg__Defaults__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
