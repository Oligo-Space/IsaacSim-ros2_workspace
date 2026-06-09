// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from rosidl_generator_py:msg/MultiNested.idl
// generated code does not contain a copyright notice

#include "rosidl_generator_py/msg/detail/multi_nested__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_rosidl_generator_py
const rosidl_type_hash_t *
rosidl_generator_py__msg__MultiNested__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x5a, 0x8b, 0xb1, 0x34, 0x39, 0xa8, 0xab, 0x63,
      0x16, 0xff, 0x61, 0xd7, 0x2d, 0x5b, 0x16, 0x6d,
      0xf1, 0x55, 0x4e, 0x55, 0x64, 0xac, 0x31, 0x1a,
      0x3e, 0xe5, 0x0b, 0x0f, 0x6b, 0x11, 0x70, 0xa0,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "rosidl_generator_py/msg/detail/basic_types__functions.h"
#include "rosidl_generator_py/msg/detail/constants__functions.h"
#include "rosidl_generator_py/msg/detail/unbounded_sequences__functions.h"
#include "rosidl_generator_py/msg/detail/arrays__functions.h"
#include "rosidl_generator_py/msg/detail/bounded_sequences__functions.h"
#include "rosidl_generator_py/msg/detail/defaults__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t rosidl_generator_py__msg__Arrays__EXPECTED_HASH = {1, {
    0xb6, 0x9e, 0x33, 0x0e, 0x4c, 0xc8, 0x35, 0x8c,
    0x77, 0x7d, 0xf1, 0x11, 0x22, 0x87, 0x0c, 0x2e,
    0xda, 0xb9, 0x0e, 0xdb, 0x79, 0xcf, 0xf0, 0xae,
    0xb2, 0xbf, 0xce, 0xd0, 0x93, 0x38, 0xc4, 0x66,
  }};
static const rosidl_type_hash_t rosidl_generator_py__msg__BasicTypes__EXPECTED_HASH = {1, {
    0x4b, 0xc8, 0x63, 0x64, 0xfa, 0x62, 0x1e, 0xb4,
    0x18, 0x68, 0xe8, 0x97, 0xe0, 0x42, 0xbb, 0x03,
    0xed, 0x6c, 0xfc, 0x64, 0xc9, 0x5d, 0x10, 0xe2,
    0xe3, 0x85, 0x41, 0x7d, 0xb9, 0x5f, 0x46, 0x76,
  }};
static const rosidl_type_hash_t rosidl_generator_py__msg__BoundedSequences__EXPECTED_HASH = {1, {
    0xe4, 0xbe, 0xb2, 0x0e, 0x9f, 0x4e, 0x86, 0x64,
    0x51, 0x3d, 0x49, 0x95, 0x77, 0xe3, 0x50, 0x23,
    0x73, 0x10, 0xde, 0xf6, 0xaf, 0x41, 0x3c, 0x8f,
    0xd4, 0x5b, 0x4e, 0x21, 0xde, 0x70, 0xef, 0x83,
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
static const rosidl_type_hash_t rosidl_generator_py__msg__UnboundedSequences__EXPECTED_HASH = {1, {
    0x36, 0x58, 0x1e, 0x14, 0xfc, 0xc1, 0xac, 0x6f,
    0x4c, 0x7d, 0x52, 0x4a, 0xc5, 0xce, 0x01, 0xa1,
    0xd6, 0x6c, 0x84, 0x05, 0x1e, 0x1f, 0x93, 0x61,
    0xee, 0x0c, 0xc6, 0x6f, 0x9b, 0xf9, 0x56, 0xbf,
  }};
#endif

static char rosidl_generator_py__msg__MultiNested__TYPE_NAME[] = "rosidl_generator_py/msg/MultiNested";
static char rosidl_generator_py__msg__Arrays__TYPE_NAME[] = "rosidl_generator_py/msg/Arrays";
static char rosidl_generator_py__msg__BasicTypes__TYPE_NAME[] = "rosidl_generator_py/msg/BasicTypes";
static char rosidl_generator_py__msg__BoundedSequences__TYPE_NAME[] = "rosidl_generator_py/msg/BoundedSequences";
static char rosidl_generator_py__msg__Constants__TYPE_NAME[] = "rosidl_generator_py/msg/Constants";
static char rosidl_generator_py__msg__Defaults__TYPE_NAME[] = "rosidl_generator_py/msg/Defaults";
static char rosidl_generator_py__msg__UnboundedSequences__TYPE_NAME[] = "rosidl_generator_py/msg/UnboundedSequences";

// Define type names, field names, and default values
static char rosidl_generator_py__msg__MultiNested__FIELD_NAME__array_of_arrays[] = "array_of_arrays";
static char rosidl_generator_py__msg__MultiNested__FIELD_NAME__array_of_bounded_sequences[] = "array_of_bounded_sequences";
static char rosidl_generator_py__msg__MultiNested__FIELD_NAME__array_of_unbounded_sequences[] = "array_of_unbounded_sequences";
static char rosidl_generator_py__msg__MultiNested__FIELD_NAME__bounded_sequence_of_arrays[] = "bounded_sequence_of_arrays";
static char rosidl_generator_py__msg__MultiNested__FIELD_NAME__bounded_sequence_of_bounded_sequences[] = "bounded_sequence_of_bounded_sequences";
static char rosidl_generator_py__msg__MultiNested__FIELD_NAME__bounded_sequence_of_unbounded_sequences[] = "bounded_sequence_of_unbounded_sequences";
static char rosidl_generator_py__msg__MultiNested__FIELD_NAME__unbounded_sequence_of_arrays[] = "unbounded_sequence_of_arrays";
static char rosidl_generator_py__msg__MultiNested__FIELD_NAME__unbounded_sequence_of_bounded_sequences[] = "unbounded_sequence_of_bounded_sequences";
static char rosidl_generator_py__msg__MultiNested__FIELD_NAME__unbounded_sequence_of_unbounded_sequences[] = "unbounded_sequence_of_unbounded_sequences";

static rosidl_runtime_c__type_description__Field rosidl_generator_py__msg__MultiNested__FIELDS[] = {
  {
    {rosidl_generator_py__msg__MultiNested__FIELD_NAME__array_of_arrays, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_ARRAY,
      3,
      0,
      {rosidl_generator_py__msg__Arrays__TYPE_NAME, 30, 30},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__MultiNested__FIELD_NAME__array_of_bounded_sequences, 26, 26},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_ARRAY,
      3,
      0,
      {rosidl_generator_py__msg__BoundedSequences__TYPE_NAME, 40, 40},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__MultiNested__FIELD_NAME__array_of_unbounded_sequences, 28, 28},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_ARRAY,
      3,
      0,
      {rosidl_generator_py__msg__UnboundedSequences__TYPE_NAME, 42, 42},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__MultiNested__FIELD_NAME__bounded_sequence_of_arrays, 26, 26},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      3,
      0,
      {rosidl_generator_py__msg__Arrays__TYPE_NAME, 30, 30},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__MultiNested__FIELD_NAME__bounded_sequence_of_bounded_sequences, 37, 37},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      3,
      0,
      {rosidl_generator_py__msg__BoundedSequences__TYPE_NAME, 40, 40},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__MultiNested__FIELD_NAME__bounded_sequence_of_unbounded_sequences, 39, 39},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      3,
      0,
      {rosidl_generator_py__msg__UnboundedSequences__TYPE_NAME, 42, 42},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__MultiNested__FIELD_NAME__unbounded_sequence_of_arrays, 28, 28},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {rosidl_generator_py__msg__Arrays__TYPE_NAME, 30, 30},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__MultiNested__FIELD_NAME__unbounded_sequence_of_bounded_sequences, 39, 39},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {rosidl_generator_py__msg__BoundedSequences__TYPE_NAME, 40, 40},
    },
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__MultiNested__FIELD_NAME__unbounded_sequence_of_unbounded_sequences, 41, 41},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {rosidl_generator_py__msg__UnboundedSequences__TYPE_NAME, 42, 42},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription rosidl_generator_py__msg__MultiNested__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {rosidl_generator_py__msg__Arrays__TYPE_NAME, 30, 30},
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__BasicTypes__TYPE_NAME, 34, 34},
    {NULL, 0, 0},
  },
  {
    {rosidl_generator_py__msg__BoundedSequences__TYPE_NAME, 40, 40},
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
  {
    {rosidl_generator_py__msg__UnboundedSequences__TYPE_NAME, 42, 42},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
rosidl_generator_py__msg__MultiNested__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {rosidl_generator_py__msg__MultiNested__TYPE_NAME, 35, 35},
      {rosidl_generator_py__msg__MultiNested__FIELDS, 9, 9},
    },
    {rosidl_generator_py__msg__MultiNested__REFERENCED_TYPE_DESCRIPTIONS, 6, 6},
  };
  if (!constructed) {
    assert(0 == memcmp(&rosidl_generator_py__msg__Arrays__EXPECTED_HASH, rosidl_generator_py__msg__Arrays__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = rosidl_generator_py__msg__Arrays__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&rosidl_generator_py__msg__BasicTypes__EXPECTED_HASH, rosidl_generator_py__msg__BasicTypes__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = rosidl_generator_py__msg__BasicTypes__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&rosidl_generator_py__msg__BoundedSequences__EXPECTED_HASH, rosidl_generator_py__msg__BoundedSequences__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = rosidl_generator_py__msg__BoundedSequences__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&rosidl_generator_py__msg__Constants__EXPECTED_HASH, rosidl_generator_py__msg__Constants__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = rosidl_generator_py__msg__Constants__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&rosidl_generator_py__msg__Defaults__EXPECTED_HASH, rosidl_generator_py__msg__Defaults__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = rosidl_generator_py__msg__Defaults__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&rosidl_generator_py__msg__UnboundedSequences__EXPECTED_HASH, rosidl_generator_py__msg__UnboundedSequences__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = rosidl_generator_py__msg__UnboundedSequences__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# Mulitple levels of nested messages\n"
  "Arrays[3] array_of_arrays\n"
  "BoundedSequences[3] array_of_bounded_sequences\n"
  "UnboundedSequences[3] array_of_unbounded_sequences\n"
  "Arrays[<=3] bounded_sequence_of_arrays\n"
  "BoundedSequences[<=3] bounded_sequence_of_bounded_sequences\n"
  "UnboundedSequences[<=3] bounded_sequence_of_unbounded_sequences\n"
  "Arrays[] unbounded_sequence_of_arrays\n"
  "BoundedSequences[] unbounded_sequence_of_bounded_sequences\n"
  "UnboundedSequences[] unbounded_sequence_of_unbounded_sequences";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
rosidl_generator_py__msg__MultiNested__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {rosidl_generator_py__msg__MultiNested__TYPE_NAME, 35, 35},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 484, 484},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
rosidl_generator_py__msg__MultiNested__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[7];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 7, 7};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *rosidl_generator_py__msg__MultiNested__get_individual_type_description_source(NULL),
    sources[1] = *rosidl_generator_py__msg__Arrays__get_individual_type_description_source(NULL);
    sources[2] = *rosidl_generator_py__msg__BasicTypes__get_individual_type_description_source(NULL);
    sources[3] = *rosidl_generator_py__msg__BoundedSequences__get_individual_type_description_source(NULL);
    sources[4] = *rosidl_generator_py__msg__Constants__get_individual_type_description_source(NULL);
    sources[5] = *rosidl_generator_py__msg__Defaults__get_individual_type_description_source(NULL);
    sources[6] = *rosidl_generator_py__msg__UnboundedSequences__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
