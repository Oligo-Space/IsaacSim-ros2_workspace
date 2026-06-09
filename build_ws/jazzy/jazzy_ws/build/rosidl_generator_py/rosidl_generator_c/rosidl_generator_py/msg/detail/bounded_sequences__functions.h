// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from rosidl_generator_py:msg/BoundedSequences.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "rosidl_generator_py/msg/bounded_sequences.h"


#ifndef ROSIDL_GENERATOR_PY__MSG__DETAIL__BOUNDED_SEQUENCES__FUNCTIONS_H_
#define ROSIDL_GENERATOR_PY__MSG__DETAIL__BOUNDED_SEQUENCES__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "rosidl_generator_py/msg/rosidl_generator_c__visibility_control.h"

#include "rosidl_generator_py/msg/detail/bounded_sequences__struct.h"

/// Initialize msg/BoundedSequences message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * rosidl_generator_py__msg__BoundedSequences
 * )) before or use
 * rosidl_generator_py__msg__BoundedSequences__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_rosidl_generator_py
bool
rosidl_generator_py__msg__BoundedSequences__init(rosidl_generator_py__msg__BoundedSequences * msg);

/// Finalize msg/BoundedSequences message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosidl_generator_py
void
rosidl_generator_py__msg__BoundedSequences__fini(rosidl_generator_py__msg__BoundedSequences * msg);

/// Create msg/BoundedSequences message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * rosidl_generator_py__msg__BoundedSequences__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rosidl_generator_py
rosidl_generator_py__msg__BoundedSequences *
rosidl_generator_py__msg__BoundedSequences__create(void);

/// Destroy msg/BoundedSequences message.
/**
 * It calls
 * rosidl_generator_py__msg__BoundedSequences__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosidl_generator_py
void
rosidl_generator_py__msg__BoundedSequences__destroy(rosidl_generator_py__msg__BoundedSequences * msg);

/// Check for msg/BoundedSequences message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosidl_generator_py
bool
rosidl_generator_py__msg__BoundedSequences__are_equal(const rosidl_generator_py__msg__BoundedSequences * lhs, const rosidl_generator_py__msg__BoundedSequences * rhs);

/// Copy a msg/BoundedSequences message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosidl_generator_py
bool
rosidl_generator_py__msg__BoundedSequences__copy(
  const rosidl_generator_py__msg__BoundedSequences * input,
  rosidl_generator_py__msg__BoundedSequences * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_rosidl_generator_py
const rosidl_type_hash_t *
rosidl_generator_py__msg__BoundedSequences__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_rosidl_generator_py
const rosidl_runtime_c__type_description__TypeDescription *
rosidl_generator_py__msg__BoundedSequences__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_rosidl_generator_py
const rosidl_runtime_c__type_description__TypeSource *
rosidl_generator_py__msg__BoundedSequences__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_rosidl_generator_py
const rosidl_runtime_c__type_description__TypeSource__Sequence *
rosidl_generator_py__msg__BoundedSequences__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/BoundedSequences messages.
/**
 * It allocates the memory for the number of elements and calls
 * rosidl_generator_py__msg__BoundedSequences__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosidl_generator_py
bool
rosidl_generator_py__msg__BoundedSequences__Sequence__init(rosidl_generator_py__msg__BoundedSequences__Sequence * array, size_t size);

/// Finalize array of msg/BoundedSequences messages.
/**
 * It calls
 * rosidl_generator_py__msg__BoundedSequences__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosidl_generator_py
void
rosidl_generator_py__msg__BoundedSequences__Sequence__fini(rosidl_generator_py__msg__BoundedSequences__Sequence * array);

/// Create array of msg/BoundedSequences messages.
/**
 * It allocates the memory for the array and calls
 * rosidl_generator_py__msg__BoundedSequences__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_rosidl_generator_py
rosidl_generator_py__msg__BoundedSequences__Sequence *
rosidl_generator_py__msg__BoundedSequences__Sequence__create(size_t size);

/// Destroy array of msg/BoundedSequences messages.
/**
 * It calls
 * rosidl_generator_py__msg__BoundedSequences__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosidl_generator_py
void
rosidl_generator_py__msg__BoundedSequences__Sequence__destroy(rosidl_generator_py__msg__BoundedSequences__Sequence * array);

/// Check for msg/BoundedSequences message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosidl_generator_py
bool
rosidl_generator_py__msg__BoundedSequences__Sequence__are_equal(const rosidl_generator_py__msg__BoundedSequences__Sequence * lhs, const rosidl_generator_py__msg__BoundedSequences__Sequence * rhs);

/// Copy an array of msg/BoundedSequences messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_rosidl_generator_py
bool
rosidl_generator_py__msg__BoundedSequences__Sequence__copy(
  const rosidl_generator_py__msg__BoundedSequences__Sequence * input,
  rosidl_generator_py__msg__BoundedSequences__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ROSIDL_GENERATOR_PY__MSG__DETAIL__BOUNDED_SEQUENCES__FUNCTIONS_H_
