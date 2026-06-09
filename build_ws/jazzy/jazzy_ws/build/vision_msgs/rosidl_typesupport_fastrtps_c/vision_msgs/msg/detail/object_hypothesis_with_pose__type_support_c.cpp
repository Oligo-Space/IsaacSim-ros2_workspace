// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from vision_msgs:msg/ObjectHypothesisWithPose.idl
// generated code does not contain a copyright notice
#include "vision_msgs/msg/detail/object_hypothesis_with_pose__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "vision_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "vision_msgs/msg/detail/object_hypothesis_with_pose__struct.h"
#include "vision_msgs/msg/detail/object_hypothesis_with_pose__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "geometry_msgs/msg/detail/pose_with_covariance__functions.h"  // pose
#include "vision_msgs/msg/detail/object_hypothesis__functions.h"  // hypothesis

// forward declare type support functions

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_vision_msgs
bool cdr_serialize_geometry_msgs__msg__PoseWithCovariance(
  const geometry_msgs__msg__PoseWithCovariance * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_vision_msgs
bool cdr_deserialize_geometry_msgs__msg__PoseWithCovariance(
  eprosima::fastcdr::Cdr & cdr,
  geometry_msgs__msg__PoseWithCovariance * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_vision_msgs
size_t get_serialized_size_geometry_msgs__msg__PoseWithCovariance(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_vision_msgs
size_t max_serialized_size_geometry_msgs__msg__PoseWithCovariance(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_vision_msgs
bool cdr_serialize_key_geometry_msgs__msg__PoseWithCovariance(
  const geometry_msgs__msg__PoseWithCovariance * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_vision_msgs
size_t get_serialized_size_key_geometry_msgs__msg__PoseWithCovariance(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_vision_msgs
size_t max_serialized_size_key_geometry_msgs__msg__PoseWithCovariance(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_vision_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, geometry_msgs, msg, PoseWithCovariance)();

bool cdr_serialize_vision_msgs__msg__ObjectHypothesis(
  const vision_msgs__msg__ObjectHypothesis * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_vision_msgs__msg__ObjectHypothesis(
  eprosima::fastcdr::Cdr & cdr,
  vision_msgs__msg__ObjectHypothesis * ros_message);

size_t get_serialized_size_vision_msgs__msg__ObjectHypothesis(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_vision_msgs__msg__ObjectHypothesis(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_vision_msgs__msg__ObjectHypothesis(
  const vision_msgs__msg__ObjectHypothesis * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_vision_msgs__msg__ObjectHypothesis(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_vision_msgs__msg__ObjectHypothesis(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, vision_msgs, msg, ObjectHypothesis)();


using _ObjectHypothesisWithPose__ros_msg_type = vision_msgs__msg__ObjectHypothesisWithPose;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vision_msgs
bool cdr_serialize_vision_msgs__msg__ObjectHypothesisWithPose(
  const vision_msgs__msg__ObjectHypothesisWithPose * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: hypothesis
  {
    cdr_serialize_vision_msgs__msg__ObjectHypothesis(
      &ros_message->hypothesis, cdr);
  }

  // Field name: pose
  {
    cdr_serialize_geometry_msgs__msg__PoseWithCovariance(
      &ros_message->pose, cdr);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vision_msgs
bool cdr_deserialize_vision_msgs__msg__ObjectHypothesisWithPose(
  eprosima::fastcdr::Cdr & cdr,
  vision_msgs__msg__ObjectHypothesisWithPose * ros_message)
{
  // Field name: hypothesis
  {
    cdr_deserialize_vision_msgs__msg__ObjectHypothesis(cdr, &ros_message->hypothesis);
  }

  // Field name: pose
  {
    cdr_deserialize_geometry_msgs__msg__PoseWithCovariance(cdr, &ros_message->pose);
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vision_msgs
size_t get_serialized_size_vision_msgs__msg__ObjectHypothesisWithPose(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ObjectHypothesisWithPose__ros_msg_type * ros_message = static_cast<const _ObjectHypothesisWithPose__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: hypothesis
  current_alignment += get_serialized_size_vision_msgs__msg__ObjectHypothesis(
    &(ros_message->hypothesis), current_alignment);

  // Field name: pose
  current_alignment += get_serialized_size_geometry_msgs__msg__PoseWithCovariance(
    &(ros_message->pose), current_alignment);

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vision_msgs
size_t max_serialized_size_vision_msgs__msg__ObjectHypothesisWithPose(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: hypothesis
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_vision_msgs__msg__ObjectHypothesis(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: pose
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_geometry_msgs__msg__PoseWithCovariance(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = vision_msgs__msg__ObjectHypothesisWithPose;
    is_plain =
      (
      offsetof(DataType, pose) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vision_msgs
bool cdr_serialize_key_vision_msgs__msg__ObjectHypothesisWithPose(
  const vision_msgs__msg__ObjectHypothesisWithPose * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: hypothesis
  {
    cdr_serialize_key_vision_msgs__msg__ObjectHypothesis(
      &ros_message->hypothesis, cdr);
  }

  // Field name: pose
  {
    cdr_serialize_key_geometry_msgs__msg__PoseWithCovariance(
      &ros_message->pose, cdr);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vision_msgs
size_t get_serialized_size_key_vision_msgs__msg__ObjectHypothesisWithPose(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ObjectHypothesisWithPose__ros_msg_type * ros_message = static_cast<const _ObjectHypothesisWithPose__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: hypothesis
  current_alignment += get_serialized_size_key_vision_msgs__msg__ObjectHypothesis(
    &(ros_message->hypothesis), current_alignment);

  // Field name: pose
  current_alignment += get_serialized_size_key_geometry_msgs__msg__PoseWithCovariance(
    &(ros_message->pose), current_alignment);

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_vision_msgs
size_t max_serialized_size_key_vision_msgs__msg__ObjectHypothesisWithPose(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: hypothesis
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_vision_msgs__msg__ObjectHypothesis(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: pose
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_geometry_msgs__msg__PoseWithCovariance(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = vision_msgs__msg__ObjectHypothesisWithPose;
    is_plain =
      (
      offsetof(DataType, pose) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _ObjectHypothesisWithPose__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const vision_msgs__msg__ObjectHypothesisWithPose * ros_message = static_cast<const vision_msgs__msg__ObjectHypothesisWithPose *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_vision_msgs__msg__ObjectHypothesisWithPose(ros_message, cdr);
}

static bool _ObjectHypothesisWithPose__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  vision_msgs__msg__ObjectHypothesisWithPose * ros_message = static_cast<vision_msgs__msg__ObjectHypothesisWithPose *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_vision_msgs__msg__ObjectHypothesisWithPose(cdr, ros_message);
}

static uint32_t _ObjectHypothesisWithPose__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_vision_msgs__msg__ObjectHypothesisWithPose(
      untyped_ros_message, 0));
}

static size_t _ObjectHypothesisWithPose__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_vision_msgs__msg__ObjectHypothesisWithPose(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ObjectHypothesisWithPose = {
  "vision_msgs::msg",
  "ObjectHypothesisWithPose",
  _ObjectHypothesisWithPose__cdr_serialize,
  _ObjectHypothesisWithPose__cdr_deserialize,
  _ObjectHypothesisWithPose__get_serialized_size,
  _ObjectHypothesisWithPose__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _ObjectHypothesisWithPose__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ObjectHypothesisWithPose,
  get_message_typesupport_handle_function,
  &vision_msgs__msg__ObjectHypothesisWithPose__get_type_hash,
  &vision_msgs__msg__ObjectHypothesisWithPose__get_type_description,
  &vision_msgs__msg__ObjectHypothesisWithPose__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, vision_msgs, msg, ObjectHypothesisWithPose)() {
  return &_ObjectHypothesisWithPose__type_support;
}

#if defined(__cplusplus)
}
#endif
