// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:srv/StitchReqCheck.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__SRV__DETAIL__STITCH_REQ_CHECK__BUILDER_HPP_
#define CUSTOM_INTERFACES__SRV__DETAIL__STITCH_REQ_CHECK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/srv/detail/stitch_req_check__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
{

namespace srv
{

namespace builder
{

class Init_StitchReqCheck_Request_numberofimages
{
public:
  Init_StitchReqCheck_Request_numberofimages()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_interfaces::srv::StitchReqCheck_Request numberofimages(::custom_interfaces::srv::StitchReqCheck_Request::_numberofimages_type arg)
  {
    msg_.numberofimages = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::srv::StitchReqCheck_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::srv::StitchReqCheck_Request>()
{
  return custom_interfaces::srv::builder::Init_StitchReqCheck_Request_numberofimages();
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace srv
{

namespace builder
{

class Init_StitchReqCheck_Response_message
{
public:
  explicit Init_StitchReqCheck_Response_message(::custom_interfaces::srv::StitchReqCheck_Response & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::srv::StitchReqCheck_Response message(::custom_interfaces::srv::StitchReqCheck_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::srv::StitchReqCheck_Response msg_;
};

class Init_StitchReqCheck_Response_success
{
public:
  Init_StitchReqCheck_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StitchReqCheck_Response_message success(::custom_interfaces::srv::StitchReqCheck_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_StitchReqCheck_Response_message(msg_);
  }

private:
  ::custom_interfaces::srv::StitchReqCheck_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::srv::StitchReqCheck_Response>()
{
  return custom_interfaces::srv::builder::Init_StitchReqCheck_Response_success();
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__SRV__DETAIL__STITCH_REQ_CHECK__BUILDER_HPP_
