// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_interfaces:msg/RGBMetadata.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__RGB_METADATA__TRAITS_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__RGB_METADATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_interfaces/msg/detail/rgb_metadata__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace custom_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const RGBMetadata & msg,
  std::ostream & out)
{
  out << "{";
  // member: name
  {
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << ", ";
  }

  // member: gps_latitude
  {
    out << "gps_latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.gps_latitude, out);
    out << ", ";
  }

  // member: gps_longitude
  {
    out << "gps_longitude: ";
    rosidl_generator_traits::value_to_yaml(msg.gps_longitude, out);
    out << ", ";
  }

  // member: gps_altitude
  {
    out << "gps_altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.gps_altitude, out);
    out << ", ";
  }

  // member: gps_signal_level
  {
    out << "gps_signal_level: ";
    rosidl_generator_traits::value_to_yaml(msg.gps_signal_level, out);
    out << ", ";
  }

  // member: compass
  {
    out << "compass: ";
    rosidl_generator_traits::value_to_yaml(msg.compass, out);
    out << ", ";
  }

  // member: roll
  {
    out << "roll: ";
    rosidl_generator_traits::value_to_yaml(msg.roll, out);
    out << ", ";
  }

  // member: pitch
  {
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << ", ";
  }

  // member: gimbal_roll
  {
    out << "gimbal_roll: ";
    rosidl_generator_traits::value_to_yaml(msg.gimbal_roll, out);
    out << ", ";
  }

  // member: gimbal_pitch
  {
    out << "gimbal_pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.gimbal_pitch, out);
    out << ", ";
  }

  // member: gimbal_yaw
  {
    out << "gimbal_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.gimbal_yaw, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RGBMetadata & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << "\n";
  }

  // member: gps_latitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gps_latitude: ";
    rosidl_generator_traits::value_to_yaml(msg.gps_latitude, out);
    out << "\n";
  }

  // member: gps_longitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gps_longitude: ";
    rosidl_generator_traits::value_to_yaml(msg.gps_longitude, out);
    out << "\n";
  }

  // member: gps_altitude
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gps_altitude: ";
    rosidl_generator_traits::value_to_yaml(msg.gps_altitude, out);
    out << "\n";
  }

  // member: gps_signal_level
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gps_signal_level: ";
    rosidl_generator_traits::value_to_yaml(msg.gps_signal_level, out);
    out << "\n";
  }

  // member: compass
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "compass: ";
    rosidl_generator_traits::value_to_yaml(msg.compass, out);
    out << "\n";
  }

  // member: roll
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roll: ";
    rosidl_generator_traits::value_to_yaml(msg.roll, out);
    out << "\n";
  }

  // member: pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.pitch, out);
    out << "\n";
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }

  // member: gimbal_roll
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gimbal_roll: ";
    rosidl_generator_traits::value_to_yaml(msg.gimbal_roll, out);
    out << "\n";
  }

  // member: gimbal_pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gimbal_pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.gimbal_pitch, out);
    out << "\n";
  }

  // member: gimbal_yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gimbal_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.gimbal_yaw, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RGBMetadata & msg, bool use_flow_style = false)
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
  const custom_interfaces::msg::RGBMetadata & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const custom_interfaces::msg::RGBMetadata & msg)
{
  return custom_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<custom_interfaces::msg::RGBMetadata>()
{
  return "custom_interfaces::msg::RGBMetadata";
}

template<>
inline const char * name<custom_interfaces::msg::RGBMetadata>()
{
  return "custom_interfaces/msg/RGBMetadata";
}

template<>
struct has_fixed_size<custom_interfaces::msg::RGBMetadata>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<custom_interfaces::msg::RGBMetadata>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<custom_interfaces::msg::RGBMetadata>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__RGB_METADATA__TRAITS_HPP_
