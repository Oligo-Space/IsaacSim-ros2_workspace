// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from ackermann_msgs:msg/AckermannDrive.idl
// generated code does not contain a copyright notice

#include "ackermann_msgs/msg/detail/ackermann_drive__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_ackermann_msgs
const rosidl_type_hash_t *
ackermann_msgs__msg__AckermannDrive__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xac, 0xf2, 0x87, 0xa2, 0x24, 0xa9, 0x47, 0xdd,
      0x1b, 0x0b, 0x87, 0xd6, 0xd7, 0x6c, 0xdb, 0x73,
      0xf4, 0x97, 0xb0, 0x23, 0x7b, 0x8f, 0xc7, 0x3b,
      0xe2, 0x17, 0x3b, 0x2e, 0xbb, 0xb8, 0x2c, 0x99,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char ackermann_msgs__msg__AckermannDrive__TYPE_NAME[] = "ackermann_msgs/msg/AckermannDrive";

// Define type names, field names, and default values
static char ackermann_msgs__msg__AckermannDrive__FIELD_NAME__steering_angle[] = "steering_angle";
static char ackermann_msgs__msg__AckermannDrive__FIELD_NAME__steering_angle_velocity[] = "steering_angle_velocity";
static char ackermann_msgs__msg__AckermannDrive__FIELD_NAME__speed[] = "speed";
static char ackermann_msgs__msg__AckermannDrive__FIELD_NAME__acceleration[] = "acceleration";
static char ackermann_msgs__msg__AckermannDrive__FIELD_NAME__jerk[] = "jerk";

static rosidl_runtime_c__type_description__Field ackermann_msgs__msg__AckermannDrive__FIELDS[] = {
  {
    {ackermann_msgs__msg__AckermannDrive__FIELD_NAME__steering_angle, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ackermann_msgs__msg__AckermannDrive__FIELD_NAME__steering_angle_velocity, 23, 23},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ackermann_msgs__msg__AckermannDrive__FIELD_NAME__speed, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ackermann_msgs__msg__AckermannDrive__FIELD_NAME__acceleration, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {ackermann_msgs__msg__AckermannDrive__FIELD_NAME__jerk, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
ackermann_msgs__msg__AckermannDrive__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {ackermann_msgs__msg__AckermannDrive__TYPE_NAME, 33, 33},
      {ackermann_msgs__msg__AckermannDrive__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "## Driving command for a car-like vehicle using Ackermann steering.\n"
  "#  $Id$\n"
  "\n"
  "# Assumes Ackermann front-wheel steering. The left and right front\n"
  "# wheels are generally at different angles. To simplify, the commanded\n"
  "# angle corresponds to the yaw of a virtual wheel located at the\n"
  "# center of the front axle, like on a tricycle.  Positive yaw is to\n"
  "# the left. (This is *not* the angle of the steering wheel inside the\n"
  "# passenger compartment.)\n"
  "#\n"
  "# Zero steering angle velocity means change the steering angle as\n"
  "# quickly as possible. Positive velocity indicates a desired absolute\n"
  "# rate of change either left or right. The controller tries not to\n"
  "# exceed this limit in either direction, but sometimes it might.\n"
  "#\n"
  "float32 steering_angle          # desired virtual angle (radians)\n"
  "float32 steering_angle_velocity # desired rate of change (radians/s)\n"
  "\n"
  "# Drive at requested speed, acceleration and jerk (the 1st, 2nd and\n"
  "# 3rd derivatives of position). All are measured at the vehicle's\n"
  "# center of rotation, typically the center of the rear axle. The\n"
  "# controller tries not to exceed these limits in either direction, but\n"
  "# sometimes it might.\n"
  "#\n"
  "# Speed is the desired scalar magnitude of the velocity vector.\n"
  "# Direction is forward unless the sign is negative, indicating reverse.\n"
  "#\n"
  "# Zero acceleration means change speed as quickly as\n"
  "# possible. Positive acceleration indicates a desired absolute\n"
  "# magnitude; that includes deceleration.\n"
  "#\n"
  "# Zero jerk means change acceleration as quickly as possible. Positive\n"
  "# jerk indicates a desired absolute rate of acceleration change in\n"
  "# either direction (increasing or decreasing).\n"
  "#\n"
  "float32 speed                   # desired forward speed (m/s)\n"
  "float32 acceleration            # desired acceleration (m/s^2)\n"
  "float32 jerk                    # desired jerk (m/s^3)";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
ackermann_msgs__msg__AckermannDrive__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {ackermann_msgs__msg__AckermannDrive__TYPE_NAME, 33, 33},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 1810, 1810},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
ackermann_msgs__msg__AckermannDrive__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *ackermann_msgs__msg__AckermannDrive__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
