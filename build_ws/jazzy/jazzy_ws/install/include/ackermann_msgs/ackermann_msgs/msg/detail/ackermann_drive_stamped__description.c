// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from ackermann_msgs:msg/AckermannDriveStamped.idl
// generated code does not contain a copyright notice

#include "ackermann_msgs/msg/detail/ackermann_drive_stamped__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_ackermann_msgs
const rosidl_type_hash_t *
ackermann_msgs__msg__AckermannDriveStamped__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x48, 0xca, 0x76, 0x12, 0xa0, 0x8d, 0x3b, 0xb7,
      0x27, 0x44, 0xfd, 0x98, 0xb7, 0x1b, 0x7c, 0xf2,
      0xea, 0x24, 0xc6, 0xad, 0x50, 0xfa, 0x4e, 0x1a,
      0xa0, 0xbb, 0xad, 0x96, 0x3c, 0x90, 0xd8, 0xcf,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "ackermann_msgs/msg/detail/ackermann_drive__functions.h"
#include "std_msgs/msg/detail/header__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t ackermann_msgs__msg__AckermannDrive__EXPECTED_HASH = {1, {
    0xac, 0xf2, 0x87, 0xa2, 0x24, 0xa9, 0x47, 0xdd,
    0x1b, 0x0b, 0x87, 0xd6, 0xd7, 0x6c, 0xdb, 0x73,
    0xf4, 0x97, 0xb0, 0x23, 0x7b, 0x8f, 0xc7, 0x3b,
    0xe2, 0x17, 0x3b, 0x2e, 0xbb, 0xb8, 0x2c, 0x99,
  }};
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char ackermann_msgs__msg__AckermannDriveStamped__TYPE_NAME[] = "ackermann_msgs/msg/AckermannDriveStamped";
static char ackermann_msgs__msg__AckermannDrive__TYPE_NAME[] = "ackermann_msgs/msg/AckermannDrive";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char ackermann_msgs__msg__AckermannDriveStamped__FIELD_NAME__header[] = "header";
static char ackermann_msgs__msg__AckermannDriveStamped__FIELD_NAME__drive[] = "drive";

static rosidl_runtime_c__type_description__Field ackermann_msgs__msg__AckermannDriveStamped__FIELDS[] = {
  {
    {ackermann_msgs__msg__AckermannDriveStamped__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {ackermann_msgs__msg__AckermannDriveStamped__FIELD_NAME__drive, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {ackermann_msgs__msg__AckermannDrive__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription ackermann_msgs__msg__AckermannDriveStamped__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {ackermann_msgs__msg__AckermannDrive__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
ackermann_msgs__msg__AckermannDriveStamped__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ackermann_msgs__msg__AckermannDriveStamped__TYPE_NAME, 40, 40},
      {ackermann_msgs__msg__AckermannDriveStamped__FIELDS, 2, 2},
    },
    {ackermann_msgs__msg__AckermannDriveStamped__REFERENCED_TYPE_DESCRIPTIONS, 3, 3},
  };
  if (!constructed) {
    assert(0 == memcmp(&ackermann_msgs__msg__AckermannDrive__EXPECTED_HASH, ackermann_msgs__msg__AckermannDrive__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = ackermann_msgs__msg__AckermannDrive__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "## Time stamped drive command for robots with Ackermann steering.\n"
  "#  $Id$\n"
  "\n"
  "std_msgs/Header header\n"
  "AckermannDrive  drive";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
ackermann_msgs__msg__AckermannDriveStamped__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ackermann_msgs__msg__AckermannDriveStamped__TYPE_NAME, 40, 40},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 120, 120},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ackermann_msgs__msg__AckermannDriveStamped__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[4];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 4, 4};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ackermann_msgs__msg__AckermannDriveStamped__get_individual_type_description_source(NULL),
    sources[1] = *ackermann_msgs__msg__AckermannDrive__get_individual_type_description_source(NULL);
    sources[2] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[3] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
