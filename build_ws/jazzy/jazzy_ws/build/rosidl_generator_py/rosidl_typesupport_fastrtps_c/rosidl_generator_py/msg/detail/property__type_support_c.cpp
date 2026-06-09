// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from rosidl_generator_py:msg/Property.idl
// generated code does not contain a copyright notice
#include "rosidl_generator_py/msg/detail/property__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_generator_py/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "rosidl_generator_py/msg/detail/property__struct.h"
#include "rosidl_generator_py/msg/detail/property__functions.h"
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

#include "rosidl_runtime_c/string.h"  // anything, property
#include "rosidl_runtime_c/string_functions.h"  // anything, property

// forward declare type support functions


using _Property__ros_msg_type = rosidl_generator_py__msg__Property;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rosidl_generator_py
bool cdr_serialize_rosidl_generator_py__msg__Property(
  const rosidl_generator_py__msg__Property * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: property
  {
    const rosidl_runtime_c__String * str = &ros_message->property;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: anything
  {
    const rosidl_runtime_c__String * str = &ros_message->anything;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rosidl_generator_py
bool cdr_deserialize_rosidl_generator_py__msg__Property(
  eprosima::fastcdr::Cdr & cdr,
  rosidl_generator_py__msg__Property * ros_message)
{
  // Field name: property
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->property.data) {
      rosidl_runtime_c__String__init(&ros_message->property);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->property,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'property'\n");
      return false;
    }
  }

  // Field name: anything
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->anything.data) {
      rosidl_runtime_c__String__init(&ros_message->anything);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->anything,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'anything'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rosidl_generator_py
size_t get_serialized_size_rosidl_generator_py__msg__Property(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Property__ros_msg_type * ros_message = static_cast<const _Property__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: property
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->property.size + 1);

  // Field name: anything
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->anything.size + 1);

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rosidl_generator_py
size_t max_serialized_size_rosidl_generator_py__msg__Property(
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

  // Field name: property
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: anything
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = rosidl_generator_py__msg__Property;
    is_plain =
      (
      offsetof(DataType, anything) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rosidl_generator_py
bool cdr_serialize_key_rosidl_generator_py__msg__Property(
  const rosidl_generator_py__msg__Property * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: property
  {
    const rosidl_runtime_c__String * str = &ros_message->property;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: anything
  {
    const rosidl_runtime_c__String * str = &ros_message->anything;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rosidl_generator_py
size_t get_serialized_size_key_rosidl_generator_py__msg__Property(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Property__ros_msg_type * ros_message = static_cast<const _Property__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: property
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->property.size + 1);

  // Field name: anything
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->anything.size + 1);

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rosidl_generator_py
size_t max_serialized_size_key_rosidl_generator_py__msg__Property(
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
  // Field name: property
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: anything
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = rosidl_generator_py__msg__Property;
    is_plain =
      (
      offsetof(DataType, anything) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _Property__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const rosidl_generator_py__msg__Property * ros_message = static_cast<const rosidl_generator_py__msg__Property *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_rosidl_generator_py__msg__Property(ros_message, cdr);
}

static bool _Property__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  rosidl_generator_py__msg__Property * ros_message = static_cast<rosidl_generator_py__msg__Property *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_rosidl_generator_py__msg__Property(cdr, ros_message);
}

static uint32_t _Property__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_rosidl_generator_py__msg__Property(
      untyped_ros_message, 0));
}

static size_t _Property__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_rosidl_generator_py__msg__Property(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_Property = {
  "rosidl_generator_py::msg",
  "Property",
  _Property__cdr_serialize,
  _Property__cdr_deserialize,
  _Property__get_serialized_size,
  _Property__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _Property__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Property,
  get_message_typesupport_handle_function,
  &rosidl_generator_py__msg__Property__get_type_hash,
  &rosidl_generator_py__msg__Property__get_type_description,
  &rosidl_generator_py__msg__Property__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rosidl_generator_py, msg, Property)() {
  return &_Property__type_support;
}

#if defined(__cplusplus)
}
#endif
