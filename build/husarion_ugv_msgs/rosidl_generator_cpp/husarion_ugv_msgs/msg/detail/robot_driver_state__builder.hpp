// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from husarion_ugv_msgs:msg/RobotDriverState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/robot_driver_state.hpp"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__ROBOT_DRIVER_STATE__BUILDER_HPP_
#define HUSARION_UGV_MSGS__MSG__DETAIL__ROBOT_DRIVER_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "husarion_ugv_msgs/msg/detail/robot_driver_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace husarion_ugv_msgs
{

namespace msg
{

namespace builder
{

class Init_RobotDriverState_read_pdo_driver_state_error
{
public:
  explicit Init_RobotDriverState_read_pdo_driver_state_error(::husarion_ugv_msgs::msg::RobotDriverState & msg)
  : msg_(msg)
  {}
  ::husarion_ugv_msgs::msg::RobotDriverState read_pdo_driver_state_error(::husarion_ugv_msgs::msg::RobotDriverState::_read_pdo_driver_state_error_type arg)
  {
    msg_.read_pdo_driver_state_error = std::move(arg);
    return std::move(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::RobotDriverState msg_;
};

class Init_RobotDriverState_read_pdo_motor_states_error
{
public:
  explicit Init_RobotDriverState_read_pdo_motor_states_error(::husarion_ugv_msgs::msg::RobotDriverState & msg)
  : msg_(msg)
  {}
  Init_RobotDriverState_read_pdo_driver_state_error read_pdo_motor_states_error(::husarion_ugv_msgs::msg::RobotDriverState::_read_pdo_motor_states_error_type arg)
  {
    msg_.read_pdo_motor_states_error = std::move(arg);
    return Init_RobotDriverState_read_pdo_driver_state_error(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::RobotDriverState msg_;
};

class Init_RobotDriverState_write_pdo_cmds_error
{
public:
  explicit Init_RobotDriverState_write_pdo_cmds_error(::husarion_ugv_msgs::msg::RobotDriverState & msg)
  : msg_(msg)
  {}
  Init_RobotDriverState_read_pdo_motor_states_error write_pdo_cmds_error(::husarion_ugv_msgs::msg::RobotDriverState::_write_pdo_cmds_error_type arg)
  {
    msg_.write_pdo_cmds_error = std::move(arg);
    return Init_RobotDriverState_read_pdo_motor_states_error(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::RobotDriverState msg_;
};

class Init_RobotDriverState_error
{
public:
  explicit Init_RobotDriverState_error(::husarion_ugv_msgs::msg::RobotDriverState & msg)
  : msg_(msg)
  {}
  Init_RobotDriverState_write_pdo_cmds_error error(::husarion_ugv_msgs::msg::RobotDriverState::_error_type arg)
  {
    msg_.error = std::move(arg);
    return Init_RobotDriverState_write_pdo_cmds_error(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::RobotDriverState msg_;
};

class Init_RobotDriverState_driver_states
{
public:
  explicit Init_RobotDriverState_driver_states(::husarion_ugv_msgs::msg::RobotDriverState & msg)
  : msg_(msg)
  {}
  Init_RobotDriverState_error driver_states(::husarion_ugv_msgs::msg::RobotDriverState::_driver_states_type arg)
  {
    msg_.driver_states = std::move(arg);
    return Init_RobotDriverState_error(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::RobotDriverState msg_;
};

class Init_RobotDriverState_header
{
public:
  Init_RobotDriverState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotDriverState_driver_states header(::husarion_ugv_msgs::msg::RobotDriverState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RobotDriverState_driver_states(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::RobotDriverState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::husarion_ugv_msgs::msg::RobotDriverState>()
{
  return husarion_ugv_msgs::msg::builder::Init_RobotDriverState_header();
}

}  // namespace husarion_ugv_msgs

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__ROBOT_DRIVER_STATE__BUILDER_HPP_
