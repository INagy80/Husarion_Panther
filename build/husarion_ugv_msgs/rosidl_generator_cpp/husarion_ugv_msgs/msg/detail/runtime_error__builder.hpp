// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from husarion_ugv_msgs:msg/RuntimeError.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/runtime_error.hpp"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__RUNTIME_ERROR__BUILDER_HPP_
#define HUSARION_UGV_MSGS__MSG__DETAIL__RUNTIME_ERROR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "husarion_ugv_msgs/msg/detail/runtime_error__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace husarion_ugv_msgs
{

namespace msg
{

namespace builder
{

class Init_RuntimeError_amps_trigger_activated
{
public:
  explicit Init_RuntimeError_amps_trigger_activated(::husarion_ugv_msgs::msg::RuntimeError & msg)
  : msg_(msg)
  {}
  ::husarion_ugv_msgs::msg::RuntimeError amps_trigger_activated(::husarion_ugv_msgs::msg::RuntimeError::_amps_trigger_activated_type arg)
  {
    msg_.amps_trigger_activated = std::move(arg);
    return std::move(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::RuntimeError msg_;
};

class Init_RuntimeError_reverse_limit_triggered
{
public:
  explicit Init_RuntimeError_reverse_limit_triggered(::husarion_ugv_msgs::msg::RuntimeError & msg)
  : msg_(msg)
  {}
  Init_RuntimeError_amps_trigger_activated reverse_limit_triggered(::husarion_ugv_msgs::msg::RuntimeError::_reverse_limit_triggered_type arg)
  {
    msg_.reverse_limit_triggered = std::move(arg);
    return Init_RuntimeError_amps_trigger_activated(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::RuntimeError msg_;
};

class Init_RuntimeError_forward_limit_triggered
{
public:
  explicit Init_RuntimeError_forward_limit_triggered(::husarion_ugv_msgs::msg::RuntimeError & msg)
  : msg_(msg)
  {}
  Init_RuntimeError_reverse_limit_triggered forward_limit_triggered(::husarion_ugv_msgs::msg::RuntimeError::_forward_limit_triggered_type arg)
  {
    msg_.forward_limit_triggered = std::move(arg);
    return Init_RuntimeError_reverse_limit_triggered(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::RuntimeError msg_;
};

class Init_RuntimeError_safety_stop_active
{
public:
  explicit Init_RuntimeError_safety_stop_active(::husarion_ugv_msgs::msg::RuntimeError & msg)
  : msg_(msg)
  {}
  Init_RuntimeError_forward_limit_triggered safety_stop_active(::husarion_ugv_msgs::msg::RuntimeError::_safety_stop_active_type arg)
  {
    msg_.safety_stop_active = std::move(arg);
    return Init_RuntimeError_forward_limit_triggered(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::RuntimeError msg_;
};

class Init_RuntimeError_loop_error
{
public:
  explicit Init_RuntimeError_loop_error(::husarion_ugv_msgs::msg::RuntimeError & msg)
  : msg_(msg)
  {}
  Init_RuntimeError_safety_stop_active loop_error(::husarion_ugv_msgs::msg::RuntimeError::_loop_error_type arg)
  {
    msg_.loop_error = std::move(arg);
    return Init_RuntimeError_safety_stop_active(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::RuntimeError msg_;
};

class Init_RuntimeError_motor_stall
{
public:
  explicit Init_RuntimeError_motor_stall(::husarion_ugv_msgs::msg::RuntimeError & msg)
  : msg_(msg)
  {}
  Init_RuntimeError_loop_error motor_stall(::husarion_ugv_msgs::msg::RuntimeError::_motor_stall_type arg)
  {
    msg_.motor_stall = std::move(arg);
    return Init_RuntimeError_loop_error(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::RuntimeError msg_;
};

class Init_RuntimeError_amps_limit_active
{
public:
  Init_RuntimeError_amps_limit_active()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RuntimeError_motor_stall amps_limit_active(::husarion_ugv_msgs::msg::RuntimeError::_amps_limit_active_type arg)
  {
    msg_.amps_limit_active = std::move(arg);
    return Init_RuntimeError_motor_stall(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::RuntimeError msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::husarion_ugv_msgs::msg::RuntimeError>()
{
  return husarion_ugv_msgs::msg::builder::Init_RuntimeError_amps_limit_active();
}

}  // namespace husarion_ugv_msgs

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__RUNTIME_ERROR__BUILDER_HPP_
