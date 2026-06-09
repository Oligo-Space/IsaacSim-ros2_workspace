// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosidl_generator_py:msg/BuiltinTypeSequencesIdl.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "rosidl_generator_py/msg/builtin_type_sequences_idl.hpp"


#ifndef ROSIDL_GENERATOR_PY__MSG__DETAIL__BUILTIN_TYPE_SEQUENCES_IDL__BUILDER_HPP_
#define ROSIDL_GENERATOR_PY__MSG__DETAIL__BUILTIN_TYPE_SEQUENCES_IDL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosidl_generator_py/msg/detail/builtin_type_sequences_idl__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rosidl_generator_py
{

namespace msg
{

namespace builder
{

class Init_BuiltinTypeSequencesIdl_char_sequence_unbounded
{
public:
  Init_BuiltinTypeSequencesIdl_char_sequence_unbounded()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::rosidl_generator_py::msg::BuiltinTypeSequencesIdl char_sequence_unbounded(::rosidl_generator_py::msg::BuiltinTypeSequencesIdl::_char_sequence_unbounded_type arg)
  {
    msg_.char_sequence_unbounded = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosidl_generator_py::msg::BuiltinTypeSequencesIdl msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosidl_generator_py::msg::BuiltinTypeSequencesIdl>()
{
  return rosidl_generator_py::msg::builder::Init_BuiltinTypeSequencesIdl_char_sequence_unbounded();
}

}  // namespace rosidl_generator_py

#endif  // ROSIDL_GENERATOR_PY__MSG__DETAIL__BUILTIN_TYPE_SEQUENCES_IDL__BUILDER_HPP_
