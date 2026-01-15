// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_interfaces:msg/StitchData.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__STITCH_DATA__TRAITS_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__STITCH_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_interfaces/msg/detail/stitch_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace custom_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const StitchData & msg,
  std::ostream & out)
{
  out << "{";
  // member: data
  {
    out << "data: ";
    rosidl_generator_traits::value_to_yaml(msg.data, out);
    out << ", ";
  }

  // member: names
  {
    if (msg.names.size() == 0) {
      out << "names: []";
    } else {
      out << "names: [";
      size_t pending_items = msg.names.size();
      for (auto item : msg.names) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: gps_latitude
  {
    if (msg.gps_latitude.size() == 0) {
      out << "gps_latitude: []";
    } else {
      out << "gps_latitude: [";
      size_t pending_items = msg.gps_latitude.size();
      for (auto item : msg.gps_latitude) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: gps_longitude
  {
    if (msg.gps_longitude.size() == 0) {
      out << "gps_longitude: []";
    } else {
      out << "gps_longitude: [";
      size_t pending_items = msg.gps_longitude.size();
      for (auto item : msg.gps_longitude) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: gps_altitude
  {
    if (msg.gps_altitude.size() == 0) {
      out << "gps_altitude: []";
    } else {
      out << "gps_altitude: [";
      size_t pending_items = msg.gps_altitude.size();
      for (auto item : msg.gps_altitude) {
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
  const StitchData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "data: ";
    rosidl_generator_traits::value_to_yaml(msg.data, out);
    out << "\n";
  }

  // member: names
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.names.size() == 0) {
      out << "names: []\n";
    } else {
      out << "names:\n";
      for (auto item : msg.names) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: gps_latitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.gps_latitude.size() == 0) {
      out << "gps_latitude: []\n";
    } else {
      out << "gps_latitude:\n";
      for (auto item : msg.gps_latitude) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: gps_longitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.gps_longitude.size() == 0) {
      out << "gps_longitude: []\n";
    } else {
      out << "gps_longitude:\n";
      for (auto item : msg.gps_longitude) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: gps_altitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.gps_altitude.size() == 0) {
      out << "gps_altitude: []\n";
    } else {
      out << "gps_altitude:\n";
      for (auto item : msg.gps_altitude) {
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

inline std::string to_yaml(const StitchData & msg, bool use_flow_style = false)
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

}  // namespace custom_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use custom_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_interfaces::msg::StitchData & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const custom_interfaces::msg::StitchData & msg)
{
  return custom_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<custom_interfaces::msg::StitchData>()
{
  return "custom_interfaces::msg::StitchData";
}

template<>
inline const char * name<custom_interfaces::msg::StitchData>()
{
  return "custom_interfaces/msg/StitchData";
}

template<>
struct has_fixed_size<custom_interfaces::msg::StitchData>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<custom_interfaces::msg::StitchData>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<custom_interfaces::msg::StitchData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__STITCH_DATA__TRAITS_HPP_
