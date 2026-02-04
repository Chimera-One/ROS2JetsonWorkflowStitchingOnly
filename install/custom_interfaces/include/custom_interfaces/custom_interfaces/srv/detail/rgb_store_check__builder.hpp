// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:srv/RGBStoreCheck.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__SRV__DETAIL__RGB_STORE_CHECK__BUILDER_HPP_
#define CUSTOM_INTERFACES__SRV__DETAIL__RGB_STORE_CHECK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/srv/detail/rgb_store_check__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
{

namespace srv
{

namespace builder
{

class Init_RGBStoreCheck_Request_image
{
public:
  Init_RGBStoreCheck_Request_image()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_interfaces::srv::RGBStoreCheck_Request image(::custom_interfaces::srv::RGBStoreCheck_Request::_image_type arg)
  {
    msg_.image = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::srv::RGBStoreCheck_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::srv::RGBStoreCheck_Request>()
{
  return custom_interfaces::srv::builder::Init_RGBStoreCheck_Request_image();
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace srv
{

namespace builder
{

class Init_RGBStoreCheck_Response_numberofimagesreceived
{
public:
  explicit Init_RGBStoreCheck_Response_numberofimagesreceived(::custom_interfaces::srv::RGBStoreCheck_Response & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::srv::RGBStoreCheck_Response numberofimagesreceived(::custom_interfaces::srv::RGBStoreCheck_Response::_numberofimagesreceived_type arg)
  {
    msg_.numberofimagesreceived = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::srv::RGBStoreCheck_Response msg_;
};

class Init_RGBStoreCheck_Response_message
{
public:
  explicit Init_RGBStoreCheck_Response_message(::custom_interfaces::srv::RGBStoreCheck_Response & msg)
  : msg_(msg)
  {}
  Init_RGBStoreCheck_Response_numberofimagesreceived message(::custom_interfaces::srv::RGBStoreCheck_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_RGBStoreCheck_Response_numberofimagesreceived(msg_);
  }

private:
  ::custom_interfaces::srv::RGBStoreCheck_Response msg_;
};

class Init_RGBStoreCheck_Response_success
{
public:
  Init_RGBStoreCheck_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RGBStoreCheck_Response_message success(::custom_interfaces::srv::RGBStoreCheck_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_RGBStoreCheck_Response_message(msg_);
  }

private:
  ::custom_interfaces::srv::RGBStoreCheck_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::srv::RGBStoreCheck_Response>()
{
  return custom_interfaces::srv::builder::Init_RGBStoreCheck_Response_success();
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__SRV__DETAIL__RGB_STORE_CHECK__BUILDER_HPP_
