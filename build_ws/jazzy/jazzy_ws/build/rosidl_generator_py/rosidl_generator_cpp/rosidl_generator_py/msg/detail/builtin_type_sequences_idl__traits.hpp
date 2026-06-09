// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rosidl_generator_py:msg/BuiltinTypeSequencesIdl.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "rosidl_generator_py/msg/builtin_type_sequences_idl.hpp"


#ifndef ROSIDL_GENERATOR_PY__MSG__DETAIL__BUILTIN_TYPE_SEQUENCES_IDL__TRAITS_HPP_
#define ROSIDL_GENERATOR_PY__MSG__DETAIL__BUILTIN_TYPE_SEQUENCES_IDL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rosidl_generator_py/msg/detail/builtin_type_sequences_idl__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rosidl_generator_py
{

namespace msg
{

inline void to_flow_style_yaml(
  const BuiltinTypeSequencesIdl & msg,
  std::ostream & out)
{
  out << "{";
  // member: char_sequence_unbounded
  {
    if (msg.char_sequence_unbounded.size() == 0) {
      out << "char_sequence_unbounded: []";
    } else {
      out << "char_sequence_unbounded: [";
      size_t pending_items = msg.char_sequence_unbounded.size();
      for (auto item : msg.char_sequence_unbounded) {
        rosidl_generator_traits::character_value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BuiltinTypeSequencesIdl & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: char_sequence_unbounded
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.char_sequence_unbounded.size() == 0) {
      out << "char_sequence_unbounded: []\n";
    } else {
      out << "char_sequence_unbounded:\n";
      for (auto item : msg.char_sequence_unbounded) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::character_value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BuiltinTypeSequencesIdl & msg, bool use_flow_style = false)
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
  const rosidl_generator_py::msg::BuiltinTypeSequencesIdl & msg,
  std::ostream & out, size_t indentation = 0)
{
  rosidl_generator_py::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rosidl_generator_py::msg::to_yaml() instead")]]
inline std::string to_yaml(const rosidl_generator_py::msg::BuiltinTypeSequencesIdl & msg)
{
  return rosidl_generator_py::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rosidl_generator_py::msg::BuiltinTypeSequencesIdl>()
{
  return "rosidl_generator_py::msg::BuiltinTypeSequencesIdl";
}

template<>
inline const char * name<rosidl_generator_py::msg::BuiltinTypeSequencesIdl>()
{
  return "rosidl_generator_py/msg/BuiltinTypeSequencesIdl";
}

template<>
struct has_fixed_size<rosidl_generator_py::msg::BuiltinTypeSequencesIdl>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rosidl_generator_py::msg::BuiltinTypeSequencesIdl>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rosidl_generator_py::msg::BuiltinTypeSequencesIdl>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROSIDL_GENERATOR_PY__MSG__DETAIL__BUILTIN_TYPE_SEQUENCES_IDL__TRAITS_HPP_
