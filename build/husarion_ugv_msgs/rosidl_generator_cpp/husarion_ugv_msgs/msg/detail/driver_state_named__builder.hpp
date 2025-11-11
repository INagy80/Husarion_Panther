// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from husarion_ugv_msgs:msg/DriverStateNamed.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/driver_state_named.hpp"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__DRIVER_STATE_NAMED__BUILDER_HPP_
#define HUSARION_UGV_MSGS__MSG__DETAIL__DRIVER_STATE_NAMED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "husarion_ugv_msgs/msg/detail/driver_state_named__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace husarion_ugv_msgs
{

namespace msg
{

namespace builder
{

class Init_DriverStateNamed_state
{
public:
  explicit Init_DriverStateNamed_state(::husarion_ugv_msgs::msg::DriverStateNamed & msg)
  : msg_(msg)
  {}
  ::husarion_ugv_msgs::msg::DriverStateNamed state(::husarion_ugv_msgs::msg::DriverStateNamed::_state_type arg)
  {
    msg_.state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::DriverStateNamed msg_;
};

class Init_DriverStateNamed_name
{
public:
  Init_DriverStateNamed_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DriverStateNamed_state name(::husarion_ugv_msgs::msg::DriverStateNamed::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_DriverStateNamed_state(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::DriverStateNamed msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::husarion_ugv_msgs::msg::DriverStateNamed>()
{
  return husarion_ugv_msgs::msg::builder::Init_DriverStateNamed_name();
}

}  // namespace husarion_ugv_msgs

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__DRIVER_STATE_NAMED__BUILDER_HPP_
