// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from husarion_ugv_msgs:msg/LEDAnimationQueue.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/led_animation_queue.hpp"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__LED_ANIMATION_QUEUE__BUILDER_HPP_
#define HUSARION_UGV_MSGS__MSG__DETAIL__LED_ANIMATION_QUEUE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "husarion_ugv_msgs/msg/detail/led_animation_queue__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace husarion_ugv_msgs
{

namespace msg
{

namespace builder
{

class Init_LEDAnimationQueue_queue
{
public:
  Init_LEDAnimationQueue_queue()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::husarion_ugv_msgs::msg::LEDAnimationQueue queue(::husarion_ugv_msgs::msg::LEDAnimationQueue::_queue_type arg)
  {
    msg_.queue = std::move(arg);
    return std::move(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::LEDAnimationQueue msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::husarion_ugv_msgs::msg::LEDAnimationQueue>()
{
  return husarion_ugv_msgs::msg::builder::Init_LEDAnimationQueue_queue();
}

}  // namespace husarion_ugv_msgs

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__LED_ANIMATION_QUEUE__BUILDER_HPP_
