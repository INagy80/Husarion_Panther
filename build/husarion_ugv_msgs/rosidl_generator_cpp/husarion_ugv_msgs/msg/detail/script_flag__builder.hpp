// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from husarion_ugv_msgs:msg/ScriptFlag.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/script_flag.hpp"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__SCRIPT_FLAG__BUILDER_HPP_
#define HUSARION_UGV_MSGS__MSG__DETAIL__SCRIPT_FLAG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "husarion_ugv_msgs/msg/detail/script_flag__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace husarion_ugv_msgs
{

namespace msg
{

namespace builder
{

class Init_ScriptFlag_amp_limiter
{
public:
  explicit Init_ScriptFlag_amp_limiter(::husarion_ugv_msgs::msg::ScriptFlag & msg)
  : msg_(msg)
  {}
  ::husarion_ugv_msgs::msg::ScriptFlag amp_limiter(::husarion_ugv_msgs::msg::ScriptFlag::_amp_limiter_type arg)
  {
    msg_.amp_limiter = std::move(arg);
    return std::move(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::ScriptFlag msg_;
};

class Init_ScriptFlag_encoder_disconnected
{
public:
  explicit Init_ScriptFlag_encoder_disconnected(::husarion_ugv_msgs::msg::ScriptFlag & msg)
  : msg_(msg)
  {}
  Init_ScriptFlag_amp_limiter encoder_disconnected(::husarion_ugv_msgs::msg::ScriptFlag::_encoder_disconnected_type arg)
  {
    msg_.encoder_disconnected = std::move(arg);
    return Init_ScriptFlag_amp_limiter(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::ScriptFlag msg_;
};

class Init_ScriptFlag_loop_error
{
public:
  Init_ScriptFlag_loop_error()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ScriptFlag_encoder_disconnected loop_error(::husarion_ugv_msgs::msg::ScriptFlag::_loop_error_type arg)
  {
    msg_.loop_error = std::move(arg);
    return Init_ScriptFlag_encoder_disconnected(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::ScriptFlag msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::husarion_ugv_msgs::msg::ScriptFlag>()
{
  return husarion_ugv_msgs::msg::builder::Init_ScriptFlag_loop_error();
}

}  // namespace husarion_ugv_msgs

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__SCRIPT_FLAG__BUILDER_HPP_
