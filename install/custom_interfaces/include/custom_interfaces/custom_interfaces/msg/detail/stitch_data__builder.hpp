// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:msg/StitchData.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__STITCH_DATA__BUILDER_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__STITCH_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/msg/detail/stitch_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
{

namespace msg
{

namespace builder
{

class Init_StitchData_gps_altitude
{
public:
  explicit Init_StitchData_gps_altitude(::custom_interfaces::msg::StitchData & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::msg::StitchData gps_altitude(::custom_interfaces::msg::StitchData::_gps_altitude_type arg)
  {
    msg_.gps_altitude = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::msg::StitchData msg_;
};

class Init_StitchData_gps_longitude
{
public:
  explicit Init_StitchData_gps_longitude(::custom_interfaces::msg::StitchData & msg)
  : msg_(msg)
  {}
  Init_StitchData_gps_altitude gps_longitude(::custom_interfaces::msg::StitchData::_gps_longitude_type arg)
  {
    msg_.gps_longitude = std::move(arg);
    return Init_StitchData_gps_altitude(msg_);
  }

private:
  ::custom_interfaces::msg::StitchData msg_;
};

class Init_StitchData_gps_latitude
{
public:
  explicit Init_StitchData_gps_latitude(::custom_interfaces::msg::StitchData & msg)
  : msg_(msg)
  {}
  Init_StitchData_gps_longitude gps_latitude(::custom_interfaces::msg::StitchData::_gps_latitude_type arg)
  {
    msg_.gps_latitude = std::move(arg);
    return Init_StitchData_gps_longitude(msg_);
  }

private:
  ::custom_interfaces::msg::StitchData msg_;
};

class Init_StitchData_names
{
public:
  explicit Init_StitchData_names(::custom_interfaces::msg::StitchData & msg)
  : msg_(msg)
  {}
  Init_StitchData_gps_latitude names(::custom_interfaces::msg::StitchData::_names_type arg)
  {
    msg_.names = std::move(arg);
    return Init_StitchData_gps_latitude(msg_);
  }

private:
  ::custom_interfaces::msg::StitchData msg_;
};

class Init_StitchData_data
{
public:
  Init_StitchData_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StitchData_names data(::custom_interfaces::msg::StitchData::_data_type arg)
  {
    msg_.data = std::move(arg);
    return Init_StitchData_names(msg_);
  }

private:
  ::custom_interfaces::msg::StitchData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::msg::StitchData>()
{
  return custom_interfaces::msg::builder::Init_StitchData_data();
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__STITCH_DATA__BUILDER_HPP_
