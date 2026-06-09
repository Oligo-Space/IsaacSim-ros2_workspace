// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from rosidl_generator_py:msg/StringArrays.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "rosidl_generator_py/msg/string_arrays.hpp"


#ifndef ROSIDL_GENERATOR_PY__MSG__DETAIL__STRING_ARRAYS__TRAITS_HPP_
#define ROSIDL_GENERATOR_PY__MSG__DETAIL__STRING_ARRAYS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "rosidl_generator_py/msg/detail/string_arrays__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace rosidl_generator_py
{

namespace msg
{

inline void to_flow_style_yaml(
  const StringArrays & msg,
  std::ostream & out)
{
  out << "{";
  // member: ub_string_static_array_value
  {
    if (msg.ub_string_static_array_value.size() == 0) {
      out << "ub_string_static_array_value: []";
    } else {
      out << "ub_string_static_array_value: [";
      size_t pending_items = msg.ub_string_static_array_value.size();
      for (auto item : msg.ub_string_static_array_value) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: ub_string_ub_array_value
  {
    if (msg.ub_string_ub_array_value.size() == 0) {
      out << "ub_string_ub_array_value: []";
    } else {
      out << "ub_string_ub_array_value: [";
      size_t pending_items = msg.ub_string_ub_array_value.size();
      for (auto item : msg.ub_string_ub_array_value) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: ub_string_dynamic_array_value
  {
    if (msg.ub_string_dynamic_array_value.size() == 0) {
      out << "ub_string_dynamic_array_value: []";
    } else {
      out << "ub_string_dynamic_array_value: [";
      size_t pending_items = msg.ub_string_dynamic_array_value.size();
      for (auto item : msg.ub_string_dynamic_array_value) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: string_dynamic_array_value
  {
    if (msg.string_dynamic_array_value.size() == 0) {
      out << "string_dynamic_array_value: []";
    } else {
      out << "string_dynamic_array_value: [";
      size_t pending_items = msg.string_dynamic_array_value.size();
      for (auto item : msg.string_dynamic_array_value) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: string_static_array_value
  {
    if (msg.string_static_array_value.size() == 0) {
      out << "string_static_array_value: []";
    } else {
      out << "string_static_array_value: [";
      size_t pending_items = msg.string_static_array_value.size();
      for (auto item : msg.string_static_array_value) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: string_bounded_array_value
  {
    if (msg.string_bounded_array_value.size() == 0) {
      out << "string_bounded_array_value: []";
    } else {
      out << "string_bounded_array_value: [";
      size_t pending_items = msg.string_bounded_array_value.size();
      for (auto item : msg.string_bounded_array_value) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: def_string_dynamic_array_value
  {
    if (msg.def_string_dynamic_array_value.size() == 0) {
      out << "def_string_dynamic_array_value: []";
    } else {
      out << "def_string_dynamic_array_value: [";
      size_t pending_items = msg.def_string_dynamic_array_value.size();
      for (auto item : msg.def_string_dynamic_array_value) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: def_string_static_array_value
  {
    if (msg.def_string_static_array_value.size() == 0) {
      out << "def_string_static_array_value: []";
    } else {
      out << "def_string_static_array_value: [";
      size_t pending_items = msg.def_string_static_array_value.size();
      for (auto item : msg.def_string_static_array_value) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: def_string_bounded_array_value
  {
    if (msg.def_string_bounded_array_value.size() == 0) {
      out << "def_string_bounded_array_value: []";
    } else {
      out << "def_string_bounded_array_value: [";
      size_t pending_items = msg.def_string_bounded_array_value.size();
      for (auto item : msg.def_string_bounded_array_value) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: def_various_quotes
  {
    if (msg.def_various_quotes.size() == 0) {
      out << "def_various_quotes: []";
    } else {
      out << "def_various_quotes: [";
      size_t pending_items = msg.def_various_quotes.size();
      for (auto item : msg.def_various_quotes) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: def_various_commas
  {
    if (msg.def_various_commas.size() == 0) {
      out << "def_various_commas: []";
    } else {
      out << "def_various_commas: [";
      size_t pending_items = msg.def_various_commas.size();
      for (auto item : msg.def_various_commas) {
        rosidl_generator_traits::value_to_yaml(item, out);
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
  const StringArrays & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: ub_string_static_array_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.ub_string_static_array_value.size() == 0) {
      out << "ub_string_static_array_value: []\n";
    } else {
      out << "ub_string_static_array_value:\n";
      for (auto item : msg.ub_string_static_array_value) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: ub_string_ub_array_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.ub_string_ub_array_value.size() == 0) {
      out << "ub_string_ub_array_value: []\n";
    } else {
      out << "ub_string_ub_array_value:\n";
      for (auto item : msg.ub_string_ub_array_value) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: ub_string_dynamic_array_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.ub_string_dynamic_array_value.size() == 0) {
      out << "ub_string_dynamic_array_value: []\n";
    } else {
      out << "ub_string_dynamic_array_value:\n";
      for (auto item : msg.ub_string_dynamic_array_value) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: string_dynamic_array_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.string_dynamic_array_value.size() == 0) {
      out << "string_dynamic_array_value: []\n";
    } else {
      out << "string_dynamic_array_value:\n";
      for (auto item : msg.string_dynamic_array_value) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: string_static_array_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.string_static_array_value.size() == 0) {
      out << "string_static_array_value: []\n";
    } else {
      out << "string_static_array_value:\n";
      for (auto item : msg.string_static_array_value) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: string_bounded_array_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.string_bounded_array_value.size() == 0) {
      out << "string_bounded_array_value: []\n";
    } else {
      out << "string_bounded_array_value:\n";
      for (auto item : msg.string_bounded_array_value) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: def_string_dynamic_array_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.def_string_dynamic_array_value.size() == 0) {
      out << "def_string_dynamic_array_value: []\n";
    } else {
      out << "def_string_dynamic_array_value:\n";
      for (auto item : msg.def_string_dynamic_array_value) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: def_string_static_array_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.def_string_static_array_value.size() == 0) {
      out << "def_string_static_array_value: []\n";
    } else {
      out << "def_string_static_array_value:\n";
      for (auto item : msg.def_string_static_array_value) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: def_string_bounded_array_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.def_string_bounded_array_value.size() == 0) {
      out << "def_string_bounded_array_value: []\n";
    } else {
      out << "def_string_bounded_array_value:\n";
      for (auto item : msg.def_string_bounded_array_value) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: def_various_quotes
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.def_various_quotes.size() == 0) {
      out << "def_various_quotes: []\n";
    } else {
      out << "def_various_quotes:\n";
      for (auto item : msg.def_various_quotes) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: def_various_commas
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.def_various_commas.size() == 0) {
      out << "def_various_commas: []\n";
    } else {
      out << "def_various_commas:\n";
      for (auto item : msg.def_various_commas) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const StringArrays & msg, bool use_flow_style = false)
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
  const rosidl_generator_py::msg::StringArrays & msg,
  std::ostream & out, size_t indentation = 0)
{
  rosidl_generator_py::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use rosidl_generator_py::msg::to_yaml() instead")]]
inline std::string to_yaml(const rosidl_generator_py::msg::StringArrays & msg)
{
  return rosidl_generator_py::msg::to_yaml(msg);
}

template<>
inline const char * data_type<rosidl_generator_py::msg::StringArrays>()
{
  return "rosidl_generator_py::msg::StringArrays";
}

template<>
inline const char * name<rosidl_generator_py::msg::StringArrays>()
{
  return "rosidl_generator_py/msg/StringArrays";
}

template<>
struct has_fixed_size<rosidl_generator_py::msg::StringArrays>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<rosidl_generator_py::msg::StringArrays>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<rosidl_generator_py::msg::StringArrays>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROSIDL_GENERATOR_PY__MSG__DETAIL__STRING_ARRAYS__TRAITS_HPP_
