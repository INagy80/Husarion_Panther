// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from husarion_ugv_msgs:msg/IOState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/io_state.hpp"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__IO_STATE__BUILDER_HPP_
#define HUSARION_UGV_MSGS__MSG__DETAIL__IO_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "husarion_ugv_msgs/msg/detail/io_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace husarion_ugv_msgs
{

namespace msg
{

namespace builder
{

class Init_IOState_power_button
{
public:
  explicit Init_IOState_power_button(::husarion_ugv_msgs::msg::IOState & msg)
  : msg_(msg)
  {}
  ::husarion_ugv_msgs::msg::IOState power_button(::husarion_ugv_msgs::msg::IOState::_power_button_type arg)
  {
    msg_.power_button = std::move(arg);
    return std::move(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::IOState msg_;
};

class Init_IOState_motor_on
{
public:
  explicit Init_IOState_motor_on(::husarion_ugv_msgs::msg::IOState & msg)
  : msg_(msg)
  {}
  Init_IOState_power_button motor_on(::husarion_ugv_msgs::msg::IOState::_motor_on_type arg)
  {
    msg_.motor_on = std::move(arg);
    return Init_IOState_power_button(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::IOState msg_;
};

class Init_IOState_led_control
{
public:
  explicit Init_IOState_led_control(::husarion_ugv_msgs::msg::IOState & msg)
  : msg_(msg)
  {}
  Init_IOState_motor_on led_control(::husarion_ugv_msgs::msg::IOState::_led_control_type arg)
  {
    msg_.led_control = std::move(arg);
    return Init_IOState_motor_on(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::IOState msg_;
};

class Init_IOState_fan
{
public:
  explicit Init_IOState_fan(::husarion_ugv_msgs::msg::IOState & msg)
  : msg_(msg)
  {}
  Init_IOState_led_control fan(::husarion_ugv_msgs::msg::IOState::_fan_type arg)
  {
    msg_.fan = std::move(arg);
    return Init_IOState_led_control(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::IOState msg_;
};

class Init_IOState_digital_power
{
public:
  explicit Init_IOState_digital_power(::husarion_ugv_msgs::msg::IOState & msg)
  : msg_(msg)
  {}
  Init_IOState_fan digital_power(::husarion_ugv_msgs::msg::IOState::_digital_power_type arg)
  {
    msg_.digital_power = std::move(arg);
    return Init_IOState_fan(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::IOState msg_;
};

class Init_IOState_charger_enabled
{
public:
  explicit Init_IOState_charger_enabled(::husarion_ugv_msgs::msg::IOState & msg)
  : msg_(msg)
  {}
  Init_IOState_digital_power charger_enabled(::husarion_ugv_msgs::msg::IOState::_charger_enabled_type arg)
  {
    msg_.charger_enabled = std::move(arg);
    return Init_IOState_digital_power(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::IOState msg_;
};

class Init_IOState_charger_connected
{
public:
  explicit Init_IOState_charger_connected(::husarion_ugv_msgs::msg::IOState & msg)
  : msg_(msg)
  {}
  Init_IOState_charger_enabled charger_connected(::husarion_ugv_msgs::msg::IOState::_charger_connected_type arg)
  {
    msg_.charger_connected = std::move(arg);
    return Init_IOState_charger_enabled(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::IOState msg_;
};

class Init_IOState_aux_power
{
public:
  Init_IOState_aux_power()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_IOState_charger_connected aux_power(::husarion_ugv_msgs::msg::IOState::_aux_power_type arg)
  {
    msg_.aux_power = std::move(arg);
    return Init_IOState_charger_connected(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::IOState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::husarion_ugv_msgs::msg::IOState>()
{
  return husarion_ugv_msgs::msg::builder::Init_IOState_aux_power();
}

}  // namespace husarion_ugv_msgs

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__IO_STATE__BUILDER_HPP_
