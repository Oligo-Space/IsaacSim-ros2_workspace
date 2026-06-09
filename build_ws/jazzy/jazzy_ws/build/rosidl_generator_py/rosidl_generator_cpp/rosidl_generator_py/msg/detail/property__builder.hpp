// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from rosidl_generator_py:msg/Property.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "rosidl_generator_py/msg/property.hpp"


#ifndef ROSIDL_GENERATOR_PY__MSG__DETAIL__PROPERTY__BUILDER_HPP_
#define ROSIDL_GENERATOR_PY__MSG__DETAIL__PROPERTY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "rosidl_generator_py/msg/detail/property__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace rosidl_generator_py
{

namespace msg
{

namespace builder
{

class Init_Property_anything
{
public:
  explicit Init_Property_anything(::rosidl_generator_py::msg::Property & msg)
  : msg_(msg)
  {}
  ::rosidl_generator_py::msg::Property anything(::rosidl_generator_py::msg::Property::_anything_type arg)
  {
    msg_.anything = std::move(arg);
    return std::move(msg_);
  }

private:
  ::rosidl_generator_py::msg::Property msg_;
};

class Init_Property_property
{
public:
  Init_Property_property()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Property_anything property(::rosidl_generator_py::msg::Property::_property_type arg)
  {
    msg_.property = std::move(arg);
    return Init_Property_anything(msg_);
  }

private:
  ::rosidl_generator_py::msg::Property msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::rosidl_generator_py::msg::Property>()
{
  return rosidl_generator_py::msg::builder::Init_Property_property();
}

}  // namespace rosidl_generator_py

#endif  // ROSIDL_GENERATOR_PY__MSG__DETAIL__PROPERTY__BUILDER_HPP_
