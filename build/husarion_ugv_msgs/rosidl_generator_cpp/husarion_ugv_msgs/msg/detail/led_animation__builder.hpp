// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from husarion_ugv_msgs:msg/LEDAnimation.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/led_animation.hpp"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__LED_ANIMATION__BUILDER_HPP_
#define HUSARION_UGV_MSGS__MSG__DETAIL__LED_ANIMATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "husarion_ugv_msgs/msg/detail/led_animation__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace husarion_ugv_msgs
{

namespace msg
{

namespace builder
{

class Init_LEDAnimation_param
{
public:
  explicit Init_LEDAnimation_param(::husarion_ugv_msgs::msg::LEDAnimation & msg)
  : msg_(msg)
  {}
  ::husarion_ugv_msgs::msg::LEDAnimation param(::husarion_ugv_msgs::msg::LEDAnimation::_param_type arg)
  {
    msg_.param = std::move(arg);
    return std::move(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::LEDAnimation msg_;
};

class Init_LEDAnimation_id
{
public:
  Init_LEDAnimation_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LEDAnimation_param id(::husarion_ugv_msgs::msg::LEDAnimation::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_LEDAnimation_param(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::LEDAnimation msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::husarion_ugv_msgs::msg::LEDAnimation>()
{
  return husarion_ugv_msgs::msg::builder::Init_LEDAnimation_id();
}

}  // namespace husarion_ugv_msgs

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__LED_ANIMATION__BUILDER_HPP_
