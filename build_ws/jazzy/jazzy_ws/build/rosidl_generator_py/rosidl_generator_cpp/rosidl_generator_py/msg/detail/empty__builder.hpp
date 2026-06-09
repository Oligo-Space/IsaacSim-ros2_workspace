// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosidl_generator_py:msg/Empty.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "rosidl_generator_py/msg/empty.hpp"


#ifndef ROSIDL_GENERATOR_PY__MSG__DETAIL__EMPTY__BUILDER_HPP_
#define ROSIDL_GENERATOR_PY__MSG__DETAIL__EMPTY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosidl_generator_py/msg/detail/empty__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rosidl_generator_py
{

namespace msg
{


}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosidl_generator_py::msg::Empty>()
{
  return ::rosidl_generator_py::msg::Empty(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace rosidl_generator_py

#endif  // ROSIDL_GENERATOR_PY__MSG__DETAIL__EMPTY__BUILDER_HPP_
