// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from husarion_ugv_msgs:srv/SetLEDAnimation.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/srv/set_led_animation.hpp"


#ifndef HUSARION_UGV_MSGS__SRV__DETAIL__SET_LED_ANIMATION__BUILDER_HPP_
#define HUSARION_UGV_MSGS__SRV__DETAIL__SET_LED_ANIMATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "husarion_ugv_msgs/srv/detail/set_led_animation__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace husarion_ugv_msgs
{

namespace srv
{

namespace builder
{

class Init_SetLEDAnimation_Request_repeating
{
public:
  explicit Init_SetLEDAnimation_Request_repeating(::husarion_ugv_msgs::srv::SetLEDAnimation_Request & msg)
  : msg_(msg)
  {}
  ::husarion_ugv_msgs::srv::SetLEDAnimation_Request repeating(::husarion_ugv_msgs::srv::SetLEDAnimation_Request::_repeating_type arg)
  {
    msg_.repeating = std::move(arg);
    return std::move(msg_);
  }

private:
  ::husarion_ugv_msgs::srv::SetLEDAnimation_Request msg_;
};

class Init_SetLEDAnimation_Request_animation
{
public:
  Init_SetLEDAnimation_Request_animation()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetLEDAnimation_Request_repeating animation(::husarion_ugv_msgs::srv::SetLEDAnimation_Request::_animation_type arg)
  {
    msg_.animation = std::move(arg);
    return Init_SetLEDAnimation_Request_repeating(msg_);
  }

private:
  ::husarion_ugv_msgs::srv::SetLEDAnimation_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::husarion_ugv_msgs::srv::SetLEDAnimation_Request>()
{
  return husarion_ugv_msgs::srv::builder::Init_SetLEDAnimation_Request_animation();
}

}  // namespace husarion_ugv_msgs


namespace husarion_ugv_msgs
{

namespace srv
{

namespace builder
{

class Init_SetLEDAnimation_Response_message
{
public:
  explicit Init_SetLEDAnimation_Response_message(::husarion_ugv_msgs::srv::SetLEDAnimation_Response & msg)
  : msg_(msg)
  {}
  ::husarion_ugv_msgs::srv::SetLEDAnimation_Response message(::husarion_ugv_msgs::srv::SetLEDAnimation_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::husarion_ugv_msgs::srv::SetLEDAnimation_Response msg_;
};

class Init_SetLEDAnimation_Response_success
{
public:
  Init_SetLEDAnimation_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetLEDAnimation_Response_message success(::husarion_ugv_msgs::srv::SetLEDAnimation_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_SetLEDAnimation_Response_message(msg_);
  }

private:
  ::husarion_ugv_msgs::srv::SetLEDAnimation_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::husarion_ugv_msgs::srv::SetLEDAnimation_Response>()
{
  return husarion_ugv_msgs::srv::builder::Init_SetLEDAnimation_Response_success();
}

}  // namespace husarion_ugv_msgs


namespace husarion_ugv_msgs
{

namespace srv
{

namespace builder
{

class Init_SetLEDAnimation_Event_response
{
public:
  explicit Init_SetLEDAnimation_Event_response(::husarion_ugv_msgs::srv::SetLEDAnimation_Event & msg)
  : msg_(msg)
  {}
  ::husarion_ugv_msgs::srv::SetLEDAnimation_Event response(::husarion_ugv_msgs::srv::SetLEDAnimation_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::husarion_ugv_msgs::srv::SetLEDAnimation_Event msg_;
};

class Init_SetLEDAnimation_Event_request
{
public:
  explicit Init_SetLEDAnimation_Event_request(::husarion_ugv_msgs::srv::SetLEDAnimation_Event & msg)
  : msg_(msg)
  {}
  Init_SetLEDAnimation_Event_response request(::husarion_ugv_msgs::srv::SetLEDAnimation_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_SetLEDAnimation_Event_response(msg_);
  }

private:
  ::husarion_ugv_msgs::srv::SetLEDAnimation_Event msg_;
};

class Init_SetLEDAnimation_Event_info
{
public:
  Init_SetLEDAnimation_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetLEDAnimation_Event_request info(::husarion_ugv_msgs::srv::SetLEDAnimation_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_SetLEDAnimation_Event_request(msg_);
  }

private:
  ::husarion_ugv_msgs::srv::SetLEDAnimation_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::husarion_ugv_msgs::srv::SetLEDAnimation_Event>()
{
  return husarion_ugv_msgs::srv::builder::Init_SetLEDAnimation_Event_info();
}

}  // namespace husarion_ugv_msgs

#endif  // HUSARION_UGV_MSGS__SRV__DETAIL__SET_LED_ANIMATION__BUILDER_HPP_
