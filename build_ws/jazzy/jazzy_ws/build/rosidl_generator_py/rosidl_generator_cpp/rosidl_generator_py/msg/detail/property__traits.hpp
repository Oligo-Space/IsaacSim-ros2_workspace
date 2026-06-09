// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rosidl_generator_py:msg/Property.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "rosidl_generator_py/msg/property.hpp"


#ifndef ROSIDL_GENERATOR_PY__MSG__DETAIL__PROPERTY__TRAITS_HPP_
#define ROSIDL_GENERATOR_PY__MSG__DETAIL__PROPERTY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rosidl_generator_py/msg/detail/property__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rosidl_generator_py
{

namespace msg
{

inline void to_flow_style_yaml(
  const Property & msg,
  std::ostream & out)
{
  out << "{";
  // member: property
  {
    out << "property: ";
    rosidl_generator_traits::value_to_yaml(msg.property, out);
    out << ", ";
  }

  // member: anything
  {
    out << "anything: ";
    rosidl_generator_traits::value_to_yaml(msg.anything, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Property & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: property
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "property: ";
    rosidl_generator_traits::value_to_yaml(msg.property, out);
    out << "\n";
  }

  // member: anything
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "anything: ";
    rosidl_generator_traits::value_to_yaml(msg.anything, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Property & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace rosidl_generator_py

namespace rosidl_generator_traits
{

[[deprecated("use rosidl_generator_py::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const rosidl_generator_py::msg::Property & msg,
  std::ostream & out, size_t indentation = 0)
{
  rosidl_generator_py::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rosidl_generator_py::msg::to_yaml() instead")]]
inline std::string to_yaml(const rosidl_generator_py::msg::Property & msg)
{
  return rosidl_generator_py::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rosidl_generator_py::msg::Property>()
{
  return "rosidl_generator_py::msg::Property";
}

template<>
inline const char * name<rosidl_generator_py::msg::Property>()
{
  return "rosidl_generator_py/msg/Property";
}

template<>
struct has_fixed_size<rosidl_generator_py::msg::Property>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rosidl_generator_py::msg::Property>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rosidl_generator_py::msg::Property>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROSIDL_GENERATOR_PY__MSG__DETAIL__PROPERTY__TRAITS_HPP_
