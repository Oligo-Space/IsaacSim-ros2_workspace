// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from rosidl_generator_py:msg/StringArrays.idl
// generated code does not contain a copyright notice
#ifndef ROSIDL_GENERATOR_PY__MSG__DETAIL__STRING_ARRAYS__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define ROSIDL_GENERATOR_PY__MSG__DETAIL__STRING_ARRAYS__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "rosidl_generator_py/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "rosidl_generator_py/msg/detail/string_arrays__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rosidl_generator_py
bool cdr_serialize_rosidl_generator_py__msg__StringArrays(
  const rosidl_generator_py__msg__StringArrays * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rosidl_generator_py
bool cdr_deserialize_rosidl_generator_py__msg__StringArrays(
  eprosima::fastcdr::Cdr &,
  rosidl_generator_py__msg__StringArrays * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rosidl_generator_py
size_t get_serialized_size_rosidl_generator_py__msg__StringArrays(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rosidl_generator_py
size_t max_serialized_size_rosidl_generator_py__msg__StringArrays(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rosidl_generator_py
bool cdr_serialize_key_rosidl_generator_py__msg__StringArrays(
  const rosidl_generator_py__msg__StringArrays * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rosidl_generator_py
size_t get_serialized_size_key_rosidl_generator_py__msg__StringArrays(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rosidl_generator_py
size_t max_serialized_size_key_rosidl_generator_py__msg__StringArrays(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_rosidl_generator_py
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, rosidl_generator_py, msg, StringArrays)();

#ifdef __cplusplus
}
#endif

#endif  // ROSIDL_GENERATOR_PY__MSG__DETAIL__STRING_ARRAYS__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
