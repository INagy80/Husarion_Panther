// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from husarion_ugv_msgs:msg/DriverState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/driver_state.hpp"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__DRIVER_STATE__BUILDER_HPP_
#define HUSARION_UGV_MSGS__MSG__DETAIL__DRIVER_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "husarion_ugv_msgs/msg/detail/driver_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace husarion_ugv_msgs
{

namespace msg
{

namespace builder
{

class Init_DriverState_heartbeat_timeout
{
public:
  explicit Init_DriverState_heartbeat_timeout(::husarion_ugv_msgs::msg::DriverState & msg)
  : msg_(msg)
  {}
  ::husarion_ugv_msgs::msg::DriverState heartbeat_timeout(::husarion_ugv_msgs::msg::DriverState::_heartbeat_timeout_type arg)
  {
    msg_.heartbeat_timeout = std::move(arg);
    return std::move(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::DriverState msg_;
};

class Init_DriverState_can_error
{
public:
  explicit Init_DriverState_can_error(::husarion_ugv_msgs::msg::DriverState & msg)
  : msg_(msg)
  {}
  Init_DriverState_heartbeat_timeout can_error(::husarion_ugv_msgs::msg::DriverState::_can_error_type arg)
  {
    msg_.can_error = std::move(arg);
    return Init_DriverState_heartbeat_timeout(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::DriverState msg_;
};

class Init_DriverState_driver_state_data_timed_out
{
public:
  explicit Init_DriverState_driver_state_data_timed_out(::husarion_ugv_msgs::msg::DriverState & msg)
  : msg_(msg)
  {}
  Init_DriverState_can_error driver_state_data_timed_out(::husarion_ugv_msgs::msg::DriverState::_driver_state_data_timed_out_type arg)
  {
    msg_.driver_state_data_timed_out = std::move(arg);
    return Init_DriverState_can_error(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::DriverState msg_;
};

class Init_DriverState_motor_states_data_timed_out
{
public:
  explicit Init_DriverState_motor_states_data_timed_out(::husarion_ugv_msgs::msg::DriverState & msg)
  : msg_(msg)
  {}
  Init_DriverState_driver_state_data_timed_out motor_states_data_timed_out(::husarion_ugv_msgs::msg::DriverState::_motor_states_data_timed_out_type arg)
  {
    msg_.motor_states_data_timed_out = std::move(arg);
    return Init_DriverState_driver_state_data_timed_out(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::DriverState msg_;
};

class Init_DriverState_channel_2_motor_runtime_error
{
public:
  explicit Init_DriverState_channel_2_motor_runtime_error(::husarion_ugv_msgs::msg::DriverState & msg)
  : msg_(msg)
  {}
  Init_DriverState_motor_states_data_timed_out channel_2_motor_runtime_error(::husarion_ugv_msgs::msg::DriverState::_channel_2_motor_runtime_error_type arg)
  {
    msg_.channel_2_motor_runtime_error = std::move(arg);
    return Init_DriverState_motor_states_data_timed_out(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::DriverState msg_;
};

class Init_DriverState_channel_1_motor_runtime_error
{
public:
  explicit Init_DriverState_channel_1_motor_runtime_error(::husarion_ugv_msgs::msg::DriverState & msg)
  : msg_(msg)
  {}
  Init_DriverState_channel_2_motor_runtime_error channel_1_motor_runtime_error(::husarion_ugv_msgs::msg::DriverState::_channel_1_motor_runtime_error_type arg)
  {
    msg_.channel_1_motor_runtime_error = std::move(arg);
    return Init_DriverState_channel_2_motor_runtime_error(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::DriverState msg_;
};

class Init_DriverState_script_flag
{
public:
  explicit Init_DriverState_script_flag(::husarion_ugv_msgs::msg::DriverState & msg)
  : msg_(msg)
  {}
  Init_DriverState_channel_1_motor_runtime_error script_flag(::husarion_ugv_msgs::msg::DriverState::_script_flag_type arg)
  {
    msg_.script_flag = std::move(arg);
    return Init_DriverState_channel_1_motor_runtime_error(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::DriverState msg_;
};

class Init_DriverState_fault_flag
{
public:
  explicit Init_DriverState_fault_flag(::husarion_ugv_msgs::msg::DriverState & msg)
  : msg_(msg)
  {}
  Init_DriverState_script_flag fault_flag(::husarion_ugv_msgs::msg::DriverState::_fault_flag_type arg)
  {
    msg_.fault_flag = std::move(arg);
    return Init_DriverState_script_flag(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::DriverState msg_;
};

class Init_DriverState_heatsink_temperature
{
public:
  explicit Init_DriverState_heatsink_temperature(::husarion_ugv_msgs::msg::DriverState & msg)
  : msg_(msg)
  {}
  Init_DriverState_fault_flag heatsink_temperature(::husarion_ugv_msgs::msg::DriverState::_heatsink_temperature_type arg)
  {
    msg_.heatsink_temperature = std::move(arg);
    return Init_DriverState_fault_flag(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::DriverState msg_;
};

class Init_DriverState_temperature
{
public:
  explicit Init_DriverState_temperature(::husarion_ugv_msgs::msg::DriverState & msg)
  : msg_(msg)
  {}
  Init_DriverState_heatsink_temperature temperature(::husarion_ugv_msgs::msg::DriverState::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return Init_DriverState_heatsink_temperature(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::DriverState msg_;
};

class Init_DriverState_current
{
public:
  explicit Init_DriverState_current(::husarion_ugv_msgs::msg::DriverState & msg)
  : msg_(msg)
  {}
  Init_DriverState_temperature current(::husarion_ugv_msgs::msg::DriverState::_current_type arg)
  {
    msg_.current = std::move(arg);
    return Init_DriverState_temperature(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::DriverState msg_;
};

class Init_DriverState_voltage
{
public:
  Init_DriverState_voltage()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DriverState_current voltage(::husarion_ugv_msgs::msg::DriverState::_voltage_type arg)
  {
    msg_.voltage = std::move(arg);
    return Init_DriverState_current(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::DriverState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::husarion_ugv_msgs::msg::DriverState>()
{
  return husarion_ugv_msgs::msg::builder::Init_DriverState_voltage();
}

}  // namespace husarion_ugv_msgs

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__DRIVER_STATE__BUILDER_HPP_
