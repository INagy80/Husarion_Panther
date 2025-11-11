// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from husarion_ugv_msgs:msg/LEDImageAnimation.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/led_image_animation.hpp"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__LED_IMAGE_ANIMATION__BUILDER_HPP_
#define HUSARION_UGV_MSGS__MSG__DETAIL__LED_IMAGE_ANIMATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "husarion_ugv_msgs/msg/detail/led_image_animation__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace husarion_ugv_msgs
{

namespace msg
{

namespace builder
{

class Init_LEDImageAnimation_color
{
public:
  explicit Init_LEDImageAnimation_color(::husarion_ugv_msgs::msg::LEDImageAnimation & msg)
  : msg_(msg)
  {}
  ::husarion_ugv_msgs::msg::LEDImageAnimation color(::husarion_ugv_msgs::msg::LEDImageAnimation::_color_type arg)
  {
    msg_.color = std::move(arg);
    return std::move(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::LEDImageAnimation msg_;
};

class Init_LEDImageAnimation_repeat
{
public:
  explicit Init_LEDImageAnimation_repeat(::husarion_ugv_msgs::msg::LEDImageAnimation & msg)
  : msg_(msg)
  {}
  Init_LEDImageAnimation_color repeat(::husarion_ugv_msgs::msg::LEDImageAnimation::_repeat_type arg)
  {
    msg_.repeat = std::move(arg);
    return Init_LEDImageAnimation_color(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::LEDImageAnimation msg_;
};

class Init_LEDImageAnimation_brightness
{
public:
  explicit Init_LEDImageAnimation_brightness(::husarion_ugv_msgs::msg::LEDImageAnimation & msg)
  : msg_(msg)
  {}
  Init_LEDImageAnimation_repeat brightness(::husarion_ugv_msgs::msg::LEDImageAnimation::_brightness_type arg)
  {
    msg_.brightness = std::move(arg);
    return Init_LEDImageAnimation_repeat(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::LEDImageAnimation msg_;
};

class Init_LEDImageAnimation_duration
{
public:
  explicit Init_LEDImageAnimation_duration(::husarion_ugv_msgs::msg::LEDImageAnimation & msg)
  : msg_(msg)
  {}
  Init_LEDImageAnimation_brightness duration(::husarion_ugv_msgs::msg::LEDImageAnimation::_duration_type arg)
  {
    msg_.duration = std::move(arg);
    return Init_LEDImageAnimation_brightness(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::LEDImageAnimation msg_;
};

class Init_LEDImageAnimation_image
{
public:
  Init_LEDImageAnimation_image()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LEDImageAnimation_duration image(::husarion_ugv_msgs::msg::LEDImageAnimation::_image_type arg)
  {
    msg_.image = std::move(arg);
    return Init_LEDImageAnimation_duration(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::LEDImageAnimation msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::husarion_ugv_msgs::msg::LEDImageAnimation>()
{
  return husarion_ugv_msgs::msg::builder::Init_LEDImageAnimation_image();
}

}  // namespace husarion_ugv_msgs

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__LED_IMAGE_ANIMATION__BUILDER_HPP_
