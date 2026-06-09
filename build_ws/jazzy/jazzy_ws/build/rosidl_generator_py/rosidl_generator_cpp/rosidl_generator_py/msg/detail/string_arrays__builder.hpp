// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosidl_generator_py:msg/StringArrays.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "rosidl_generator_py/msg/string_arrays.hpp"


#ifndef ROSIDL_GENERATOR_PY__MSG__DETAIL__STRING_ARRAYS__BUILDER_HPP_
#define ROSIDL_GENERATOR_PY__MSG__DETAIL__STRING_ARRAYS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosidl_generator_py/msg/detail/string_arrays__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rosidl_generator_py
{

namespace msg
{

namespace builder
{

class Init_StringArrays_def_various_commas
{
public:
  explicit Init_StringArrays_def_various_commas(::rosidl_generator_py::msg::StringArrays & msg)
  : msg_(msg)
  {}
  ::rosidl_generator_py::msg::StringArrays def_various_commas(::rosidl_generator_py::msg::StringArrays::_def_various_commas_type arg)
  {
    msg_.def_various_commas = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosidl_generator_py::msg::StringArrays msg_;
};

class Init_StringArrays_def_various_quotes
{
public:
  explicit Init_StringArrays_def_various_quotes(::rosidl_generator_py::msg::StringArrays & msg)
  : msg_(msg)
  {}
  Init_StringArrays_def_various_commas def_various_quotes(::rosidl_generator_py::msg::StringArrays::_def_various_quotes_type arg)
  {
    msg_.def_various_quotes = std::move(arg);
    return Init_StringArrays_def_various_commas(msg_);
  }

private:
  ::rosidl_generator_py::msg::StringArrays msg_;
};

class Init_StringArrays_def_string_bounded_array_value
{
public:
  explicit Init_StringArrays_def_string_bounded_array_value(::rosidl_generator_py::msg::StringArrays & msg)
  : msg_(msg)
  {}
  Init_StringArrays_def_various_quotes def_string_bounded_array_value(::rosidl_generator_py::msg::StringArrays::_def_string_bounded_array_value_type arg)
  {
    msg_.def_string_bounded_array_value = std::move(arg);
    return Init_StringArrays_def_various_quotes(msg_);
  }

private:
  ::rosidl_generator_py::msg::StringArrays msg_;
};

class Init_StringArrays_def_string_static_array_value
{
public:
  explicit Init_StringArrays_def_string_static_array_value(::rosidl_generator_py::msg::StringArrays & msg)
  : msg_(msg)
  {}
  Init_StringArrays_def_string_bounded_array_value def_string_static_array_value(::rosidl_generator_py::msg::StringArrays::_def_string_static_array_value_type arg)
  {
    msg_.def_string_static_array_value = std::move(arg);
    return Init_StringArrays_def_string_bounded_array_value(msg_);
  }

private:
  ::rosidl_generator_py::msg::StringArrays msg_;
};

class Init_StringArrays_def_string_dynamic_array_value
{
public:
  explicit Init_StringArrays_def_string_dynamic_array_value(::rosidl_generator_py::msg::StringArrays & msg)
  : msg_(msg)
  {}
  Init_StringArrays_def_string_static_array_value def_string_dynamic_array_value(::rosidl_generator_py::msg::StringArrays::_def_string_dynamic_array_value_type arg)
  {
    msg_.def_string_dynamic_array_value = std::move(arg);
    return Init_StringArrays_def_string_static_array_value(msg_);
  }

private:
  ::rosidl_generator_py::msg::StringArrays msg_;
};

class Init_StringArrays_string_bounded_array_value
{
public:
  explicit Init_StringArrays_string_bounded_array_value(::rosidl_generator_py::msg::StringArrays & msg)
  : msg_(msg)
  {}
  Init_StringArrays_def_string_dynamic_array_value string_bounded_array_value(::rosidl_generator_py::msg::StringArrays::_string_bounded_array_value_type arg)
  {
    msg_.string_bounded_array_value = std::move(arg);
    return Init_StringArrays_def_string_dynamic_array_value(msg_);
  }

private:
  ::rosidl_generator_py::msg::StringArrays msg_;
};

class Init_StringArrays_string_static_array_value
{
public:
  explicit Init_StringArrays_string_static_array_value(::rosidl_generator_py::msg::StringArrays & msg)
  : msg_(msg)
  {}
  Init_StringArrays_string_bounded_array_value string_static_array_value(::rosidl_generator_py::msg::StringArrays::_string_static_array_value_type arg)
  {
    msg_.string_static_array_value = std::move(arg);
    return Init_StringArrays_string_bounded_array_value(msg_);
  }

private:
  ::rosidl_generator_py::msg::StringArrays msg_;
};

class Init_StringArrays_string_dynamic_array_value
{
public:
  explicit Init_StringArrays_string_dynamic_array_value(::rosidl_generator_py::msg::StringArrays & msg)
  : msg_(msg)
  {}
  Init_StringArrays_string_static_array_value string_dynamic_array_value(::rosidl_generator_py::msg::StringArrays::_string_dynamic_array_value_type arg)
  {
    msg_.string_dynamic_array_value = std::move(arg);
    return Init_StringArrays_string_static_array_value(msg_);
  }

private:
  ::rosidl_generator_py::msg::StringArrays msg_;
};

class Init_StringArrays_ub_string_dynamic_array_value
{
public:
  explicit Init_StringArrays_ub_string_dynamic_array_value(::rosidl_generator_py::msg::StringArrays & msg)
  : msg_(msg)
  {}
  Init_StringArrays_string_dynamic_array_value ub_string_dynamic_array_value(::rosidl_generator_py::msg::StringArrays::_ub_string_dynamic_array_value_type arg)
  {
    msg_.ub_string_dynamic_array_value = std::move(arg);
    return Init_StringArrays_string_dynamic_array_value(msg_);
  }

private:
  ::rosidl_generator_py::msg::StringArrays msg_;
};

class Init_StringArrays_ub_string_ub_array_value
{
public:
  explicit Init_StringArrays_ub_string_ub_array_value(::rosidl_generator_py::msg::StringArrays & msg)
  : msg_(msg)
  {}
  Init_StringArrays_ub_string_dynamic_array_value ub_string_ub_array_value(::rosidl_generator_py::msg::StringArrays::_ub_string_ub_array_value_type arg)
  {
    msg_.ub_string_ub_array_value = std::move(arg);
    return Init_StringArrays_ub_string_dynamic_array_value(msg_);
  }

private:
  ::rosidl_generator_py::msg::StringArrays msg_;
};

class Init_StringArrays_ub_string_static_array_value
{
public:
  Init_StringArrays_ub_string_static_array_value()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StringArrays_ub_string_ub_array_value ub_string_static_array_value(::rosidl_generator_py::msg::StringArrays::_ub_string_static_array_value_type arg)
  {
    msg_.ub_string_static_array_value = std::move(arg);
    return Init_StringArrays_ub_string_ub_array_value(msg_);
  }

private:
  ::rosidl_generator_py::msg::StringArrays msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosidl_generator_py::msg::StringArrays>()
{
  return rosidl_generator_py::msg::builder::Init_StringArrays_ub_string_static_array_value();
}

}  // namespace rosidl_generator_py

#endif  // ROSIDL_GENERATOR_PY__MSG__DETAIL__STRING_ARRAYS__BUILDER_HPP_
