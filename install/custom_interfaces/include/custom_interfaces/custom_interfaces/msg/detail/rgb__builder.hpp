// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:msg/RGB.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__RGB__BUILDER_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__RGB__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/msg/detail/rgb__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
{

namespace msg
{

namespace builder
{

class Init_RGB_image
{
public:
  explicit Init_RGB_image(::custom_interfaces::msg::RGB & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::msg::RGB image(::custom_interfaces::msg::RGB::_image_type arg)
  {
    msg_.image = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::msg::RGB msg_;
};

class Init_RGB_gimbal_yaw
{
public:
  explicit Init_RGB_gimbal_yaw(::custom_interfaces::msg::RGB & msg)
  : msg_(msg)
  {}
  Init_RGB_image gimbal_yaw(::custom_interfaces::msg::RGB::_gimbal_yaw_type arg)
  {
    msg_.gimbal_yaw = std::move(arg);
    return Init_RGB_image(msg_);
  }

private:
  ::custom_interfaces::msg::RGB msg_;
};

class Init_RGB_gimbal_pitch
{
public:
  explicit Init_RGB_gimbal_pitch(::custom_interfaces::msg::RGB & msg)
  : msg_(msg)
  {}
  Init_RGB_gimbal_yaw gimbal_pitch(::custom_interfaces::msg::RGB::_gimbal_pitch_type arg)
  {
    msg_.gimbal_pitch = std::move(arg);
    return Init_RGB_gimbal_yaw(msg_);
  }

private:
  ::custom_interfaces::msg::RGB msg_;
};

class Init_RGB_gimbal_roll
{
public:
  explicit Init_RGB_gimbal_roll(::custom_interfaces::msg::RGB & msg)
  : msg_(msg)
  {}
  Init_RGB_gimbal_pitch gimbal_roll(::custom_interfaces::msg::RGB::_gimbal_roll_type arg)
  {
    msg_.gimbal_roll = std::move(arg);
    return Init_RGB_gimbal_pitch(msg_);
  }

private:
  ::custom_interfaces::msg::RGB msg_;
};

class Init_RGB_yaw
{
public:
  explicit Init_RGB_yaw(::custom_interfaces::msg::RGB & msg)
  : msg_(msg)
  {}
  Init_RGB_gimbal_roll yaw(::custom_interfaces::msg::RGB::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_RGB_gimbal_roll(msg_);
  }

private:
  ::custom_interfaces::msg::RGB msg_;
};

class Init_RGB_pitch
{
public:
  explicit Init_RGB_pitch(::custom_interfaces::msg::RGB & msg)
  : msg_(msg)
  {}
  Init_RGB_yaw pitch(::custom_interfaces::msg::RGB::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_RGB_yaw(msg_);
  }

private:
  ::custom_interfaces::msg::RGB msg_;
};

class Init_RGB_roll
{
public:
  explicit Init_RGB_roll(::custom_interfaces::msg::RGB & msg)
  : msg_(msg)
  {}
  Init_RGB_pitch roll(::custom_interfaces::msg::RGB::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_RGB_pitch(msg_);
  }

private:
  ::custom_interfaces::msg::RGB msg_;
};

class Init_RGB_compass
{
public:
  explicit Init_RGB_compass(::custom_interfaces::msg::RGB & msg)
  : msg_(msg)
  {}
  Init_RGB_roll compass(::custom_interfaces::msg::RGB::_compass_type arg)
  {
    msg_.compass = std::move(arg);
    return Init_RGB_roll(msg_);
  }

private:
  ::custom_interfaces::msg::RGB msg_;
};

class Init_RGB_gps_signal_level
{
public:
  explicit Init_RGB_gps_signal_level(::custom_interfaces::msg::RGB & msg)
  : msg_(msg)
  {}
  Init_RGB_compass gps_signal_level(::custom_interfaces::msg::RGB::_gps_signal_level_type arg)
  {
    msg_.gps_signal_level = std::move(arg);
    return Init_RGB_compass(msg_);
  }

private:
  ::custom_interfaces::msg::RGB msg_;
};

class Init_RGB_gps_altitude
{
public:
  explicit Init_RGB_gps_altitude(::custom_interfaces::msg::RGB & msg)
  : msg_(msg)
  {}
  Init_RGB_gps_signal_level gps_altitude(::custom_interfaces::msg::RGB::_gps_altitude_type arg)
  {
    msg_.gps_altitude = std::move(arg);
    return Init_RGB_gps_signal_level(msg_);
  }

private:
  ::custom_interfaces::msg::RGB msg_;
};

class Init_RGB_gps_longitude
{
public:
  explicit Init_RGB_gps_longitude(::custom_interfaces::msg::RGB & msg)
  : msg_(msg)
  {}
  Init_RGB_gps_altitude gps_longitude(::custom_interfaces::msg::RGB::_gps_longitude_type arg)
  {
    msg_.gps_longitude = std::move(arg);
    return Init_RGB_gps_altitude(msg_);
  }

private:
  ::custom_interfaces::msg::RGB msg_;
};

class Init_RGB_gps_latitude
{
public:
  explicit Init_RGB_gps_latitude(::custom_interfaces::msg::RGB & msg)
  : msg_(msg)
  {}
  Init_RGB_gps_longitude gps_latitude(::custom_interfaces::msg::RGB::_gps_latitude_type arg)
  {
    msg_.gps_latitude = std::move(arg);
    return Init_RGB_gps_longitude(msg_);
  }

private:
  ::custom_interfaces::msg::RGB msg_;
};

class Init_RGB_name
{
public:
  Init_RGB_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RGB_gps_latitude name(::custom_interfaces::msg::RGB::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_RGB_gps_latitude(msg_);
  }

private:
  ::custom_interfaces::msg::RGB msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::msg::RGB>()
{
  return custom_interfaces::msg::builder::Init_RGB_name();
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__RGB__BUILDER_HPP_
