// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:msg/RGBMetadata.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__RGB_METADATA__BUILDER_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__RGB_METADATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/msg/detail/rgb_metadata__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
{

namespace msg
{

namespace builder
{

class Init_RGBMetadata_gimbal_yaw
{
public:
  explicit Init_RGBMetadata_gimbal_yaw(::custom_interfaces::msg::RGBMetadata & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::msg::RGBMetadata gimbal_yaw(::custom_interfaces::msg::RGBMetadata::_gimbal_yaw_type arg)
  {
    msg_.gimbal_yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::msg::RGBMetadata msg_;
};

class Init_RGBMetadata_gimbal_pitch
{
public:
  explicit Init_RGBMetadata_gimbal_pitch(::custom_interfaces::msg::RGBMetadata & msg)
  : msg_(msg)
  {}
  Init_RGBMetadata_gimbal_yaw gimbal_pitch(::custom_interfaces::msg::RGBMetadata::_gimbal_pitch_type arg)
  {
    msg_.gimbal_pitch = std::move(arg);
    return Init_RGBMetadata_gimbal_yaw(msg_);
  }

private:
  ::custom_interfaces::msg::RGBMetadata msg_;
};

class Init_RGBMetadata_gimbal_roll
{
public:
  explicit Init_RGBMetadata_gimbal_roll(::custom_interfaces::msg::RGBMetadata & msg)
  : msg_(msg)
  {}
  Init_RGBMetadata_gimbal_pitch gimbal_roll(::custom_interfaces::msg::RGBMetadata::_gimbal_roll_type arg)
  {
    msg_.gimbal_roll = std::move(arg);
    return Init_RGBMetadata_gimbal_pitch(msg_);
  }

private:
  ::custom_interfaces::msg::RGBMetadata msg_;
};

class Init_RGBMetadata_yaw
{
public:
  explicit Init_RGBMetadata_yaw(::custom_interfaces::msg::RGBMetadata & msg)
  : msg_(msg)
  {}
  Init_RGBMetadata_gimbal_roll yaw(::custom_interfaces::msg::RGBMetadata::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_RGBMetadata_gimbal_roll(msg_);
  }

private:
  ::custom_interfaces::msg::RGBMetadata msg_;
};

class Init_RGBMetadata_pitch
{
public:
  explicit Init_RGBMetadata_pitch(::custom_interfaces::msg::RGBMetadata & msg)
  : msg_(msg)
  {}
  Init_RGBMetadata_yaw pitch(::custom_interfaces::msg::RGBMetadata::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_RGBMetadata_yaw(msg_);
  }

private:
  ::custom_interfaces::msg::RGBMetadata msg_;
};

class Init_RGBMetadata_roll
{
public:
  explicit Init_RGBMetadata_roll(::custom_interfaces::msg::RGBMetadata & msg)
  : msg_(msg)
  {}
  Init_RGBMetadata_pitch roll(::custom_interfaces::msg::RGBMetadata::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_RGBMetadata_pitch(msg_);
  }

private:
  ::custom_interfaces::msg::RGBMetadata msg_;
};

class Init_RGBMetadata_compass
{
public:
  explicit Init_RGBMetadata_compass(::custom_interfaces::msg::RGBMetadata & msg)
  : msg_(msg)
  {}
  Init_RGBMetadata_roll compass(::custom_interfaces::msg::RGBMetadata::_compass_type arg)
  {
    msg_.compass = std::move(arg);
    return Init_RGBMetadata_roll(msg_);
  }

private:
  ::custom_interfaces::msg::RGBMetadata msg_;
};

class Init_RGBMetadata_gps_signal_level
{
public:
  explicit Init_RGBMetadata_gps_signal_level(::custom_interfaces::msg::RGBMetadata & msg)
  : msg_(msg)
  {}
  Init_RGBMetadata_compass gps_signal_level(::custom_interfaces::msg::RGBMetadata::_gps_signal_level_type arg)
  {
    msg_.gps_signal_level = std::move(arg);
    return Init_RGBMetadata_compass(msg_);
  }

private:
  ::custom_interfaces::msg::RGBMetadata msg_;
};

class Init_RGBMetadata_gps_altitude
{
public:
  explicit Init_RGBMetadata_gps_altitude(::custom_interfaces::msg::RGBMetadata & msg)
  : msg_(msg)
  {}
  Init_RGBMetadata_gps_signal_level gps_altitude(::custom_interfaces::msg::RGBMetadata::_gps_altitude_type arg)
  {
    msg_.gps_altitude = std::move(arg);
    return Init_RGBMetadata_gps_signal_level(msg_);
  }

private:
  ::custom_interfaces::msg::RGBMetadata msg_;
};

class Init_RGBMetadata_gps_longitude
{
public:
  explicit Init_RGBMetadata_gps_longitude(::custom_interfaces::msg::RGBMetadata & msg)
  : msg_(msg)
  {}
  Init_RGBMetadata_gps_altitude gps_longitude(::custom_interfaces::msg::RGBMetadata::_gps_longitude_type arg)
  {
    msg_.gps_longitude = std::move(arg);
    return Init_RGBMetadata_gps_altitude(msg_);
  }

private:
  ::custom_interfaces::msg::RGBMetadata msg_;
};

class Init_RGBMetadata_gps_latitude
{
public:
  explicit Init_RGBMetadata_gps_latitude(::custom_interfaces::msg::RGBMetadata & msg)
  : msg_(msg)
  {}
  Init_RGBMetadata_gps_longitude gps_latitude(::custom_interfaces::msg::RGBMetadata::_gps_latitude_type arg)
  {
    msg_.gps_latitude = std::move(arg);
    return Init_RGBMetadata_gps_longitude(msg_);
  }

private:
  ::custom_interfaces::msg::RGBMetadata msg_;
};

class Init_RGBMetadata_name
{
public:
  Init_RGBMetadata_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RGBMetadata_gps_latitude name(::custom_interfaces::msg::RGBMetadata::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_RGBMetadata_gps_latitude(msg_);
  }

private:
  ::custom_interfaces::msg::RGBMetadata msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::msg::RGBMetadata>()
{
  return custom_interfaces::msg::builder::Init_RGBMetadata_name();
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__RGB_METADATA__BUILDER_HPP_
