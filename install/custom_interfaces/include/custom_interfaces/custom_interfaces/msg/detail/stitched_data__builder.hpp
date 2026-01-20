// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:msg/StitchedData.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__STITCHED_DATA__BUILDER_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__STITCHED_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/msg/detail/stitched_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
{

namespace msg
{

namespace builder
{

class Init_StitchedData_image
{
public:
  explicit Init_StitchedData_image(::custom_interfaces::msg::StitchedData & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::msg::StitchedData image(::custom_interfaces::msg::StitchedData::_image_type arg)
  {
    msg_.image = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::msg::StitchedData msg_;
};

class Init_StitchedData_rotation_degree
{
public:
  Init_StitchedData_rotation_degree()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StitchedData_image rotation_degree(::custom_interfaces::msg::StitchedData::_rotation_degree_type arg)
  {
    msg_.rotation_degree = std::move(arg);
    return Init_StitchedData_image(msg_);
  }

private:
  ::custom_interfaces::msg::StitchedData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::msg::StitchedData>()
{
  return custom_interfaces::msg::builder::Init_StitchedData_rotation_degree();
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__STITCHED_DATA__BUILDER_HPP_
