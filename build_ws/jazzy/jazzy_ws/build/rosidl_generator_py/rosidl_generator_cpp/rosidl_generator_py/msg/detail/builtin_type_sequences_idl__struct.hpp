// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rosidl_generator_py:msg/BuiltinTypeSequencesIdl.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "rosidl_generator_py/msg/builtin_type_sequences_idl.hpp"


#ifndef ROSIDL_GENERATOR_PY__MSG__DETAIL__BUILTIN_TYPE_SEQUENCES_IDL__STRUCT_HPP_
#define ROSIDL_GENERATOR_PY__MSG__DETAIL__BUILTIN_TYPE_SEQUENCES_IDL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rosidl_generator_py__msg__BuiltinTypeSequencesIdl __attribute__((deprecated))
#else
# define DEPRECATED__rosidl_generator_py__msg__BuiltinTypeSequencesIdl __declspec(deprecated)
#endif

namespace rosidl_generator_py
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BuiltinTypeSequencesIdl_
{
  using Type = BuiltinTypeSequencesIdl_<ContainerAllocator>;

  explicit BuiltinTypeSequencesIdl_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit BuiltinTypeSequencesIdl_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _char_sequence_unbounded_type =
    std::vector<unsigned char, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<unsigned char>>;
  _char_sequence_unbounded_type char_sequence_unbounded;

  // setters for named parameter idiom
  Type & set__char_sequence_unbounded(
    const std::vector<unsigned char, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<unsigned char>> & _arg)
  {
    this->char_sequence_unbounded = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rosidl_generator_py::msg::BuiltinTypeSequencesIdl_<ContainerAllocator> *;
  using ConstRawPtr =
    const rosidl_generator_py::msg::BuiltinTypeSequencesIdl_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rosidl_generator_py::msg::BuiltinTypeSequencesIdl_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rosidl_generator_py::msg::BuiltinTypeSequencesIdl_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rosidl_generator_py::msg::BuiltinTypeSequencesIdl_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rosidl_generator_py::msg::BuiltinTypeSequencesIdl_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rosidl_generator_py::msg::BuiltinTypeSequencesIdl_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rosidl_generator_py::msg::BuiltinTypeSequencesIdl_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rosidl_generator_py::msg::BuiltinTypeSequencesIdl_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rosidl_generator_py::msg::BuiltinTypeSequencesIdl_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rosidl_generator_py__msg__BuiltinTypeSequencesIdl
    std::shared_ptr<rosidl_generator_py::msg::BuiltinTypeSequencesIdl_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rosidl_generator_py__msg__BuiltinTypeSequencesIdl
    std::shared_ptr<rosidl_generator_py::msg::BuiltinTypeSequencesIdl_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BuiltinTypeSequencesIdl_ & other) const
  {
    if (this->char_sequence_unbounded != other.char_sequence_unbounded) {
      return false;
    }
    return true;
  }
  bool operator!=(const BuiltinTypeSequencesIdl_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BuiltinTypeSequencesIdl_

// alias to use template instance with default allocator
using BuiltinTypeSequencesIdl =
  rosidl_generator_py::msg::BuiltinTypeSequencesIdl_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace rosidl_generator_py

#endif  // ROSIDL_GENERATOR_PY__MSG__DETAIL__BUILTIN_TYPE_SEQUENCES_IDL__STRUCT_HPP_
