// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosidl_generator_py:msg/Nested.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "rosidl_generator_py/msg/nested.hpp"


#ifndef ROSIDL_GENERATOR_PY__MSG__DETAIL__NESTED__BUILDER_HPP_
#define ROSIDL_GENERATOR_PY__MSG__DETAIL__NESTED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosidl_generator_py/msg/detail/nested__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rosidl_generator_py
{

namespace msg
{

namespace builder
{

class Init_Nested_basic_types_value
{
public:
  Init_Nested_basic_types_value()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rosidl_generator_py::msg::Nested basic_types_value(::rosidl_generator_py::msg::Nested::_basic_types_value_type arg)
  {
    msg_.basic_types_value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosidl_generator_py::msg::Nested msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosidl_generator_py::msg::Nested>()
{
  return rosidl_generator_py::msg::builder::Init_Nested_basic_types_value();
}

}  // namespace rosidl_generator_py

#endif  // ROSIDL_GENERATOR_PY__MSG__DETAIL__NESTED__BUILDER_HPP_
