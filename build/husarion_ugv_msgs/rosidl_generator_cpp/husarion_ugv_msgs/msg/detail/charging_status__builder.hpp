// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from husarion_ugv_msgs:msg/ChargingStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/charging_status.hpp"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__CHARGING_STATUS__BUILDER_HPP_
#define HUSARION_UGV_MSGS__MSG__DETAIL__CHARGING_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "husarion_ugv_msgs/msg/detail/charging_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace husarion_ugv_msgs
{

namespace msg
{

namespace builder
{

class Init_ChargingStatus_charger_type
{
public:
  explicit Init_ChargingStatus_charger_type(::husarion_ugv_msgs::msg::ChargingStatus & msg)
  : msg_(msg)
  {}
  ::husarion_ugv_msgs::msg::ChargingStatus charger_type(::husarion_ugv_msgs::msg::ChargingStatus::_charger_type_type arg)
  {
    msg_.charger_type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::ChargingStatus msg_;
};

class Init_ChargingStatus_current_battery_2
{
public:
  explicit Init_ChargingStatus_current_battery_2(::husarion_ugv_msgs::msg::ChargingStatus & msg)
  : msg_(msg)
  {}
  Init_ChargingStatus_charger_type current_battery_2(::husarion_ugv_msgs::msg::ChargingStatus::_current_battery_2_type arg)
  {
    msg_.current_battery_2 = std::move(arg);
    return Init_ChargingStatus_charger_type(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::ChargingStatus msg_;
};

class Init_ChargingStatus_current_battery_1
{
public:
  explicit Init_ChargingStatus_current_battery_1(::husarion_ugv_msgs::msg::ChargingStatus & msg)
  : msg_(msg)
  {}
  Init_ChargingStatus_current_battery_2 current_battery_1(::husarion_ugv_msgs::msg::ChargingStatus::_current_battery_1_type arg)
  {
    msg_.current_battery_1 = std::move(arg);
    return Init_ChargingStatus_current_battery_2(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::ChargingStatus msg_;
};

class Init_ChargingStatus_current
{
public:
  explicit Init_ChargingStatus_current(::husarion_ugv_msgs::msg::ChargingStatus & msg)
  : msg_(msg)
  {}
  Init_ChargingStatus_current_battery_1 current(::husarion_ugv_msgs::msg::ChargingStatus::_current_type arg)
  {
    msg_.current = std::move(arg);
    return Init_ChargingStatus_current_battery_1(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::ChargingStatus msg_;
};

class Init_ChargingStatus_charging
{
public:
  explicit Init_ChargingStatus_charging(::husarion_ugv_msgs::msg::ChargingStatus & msg)
  : msg_(msg)
  {}
  Init_ChargingStatus_current charging(::husarion_ugv_msgs::msg::ChargingStatus::_charging_type arg)
  {
    msg_.charging = std::move(arg);
    return Init_ChargingStatus_current(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::ChargingStatus msg_;
};

class Init_ChargingStatus_header
{
public:
  Init_ChargingStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ChargingStatus_charging header(::husarion_ugv_msgs::msg::ChargingStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ChargingStatus_charging(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::ChargingStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::husarion_ugv_msgs::msg::ChargingStatus>()
{
  return husarion_ugv_msgs::msg::builder::Init_ChargingStatus_header();
}

}  // namespace husarion_ugv_msgs

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__CHARGING_STATUS__BUILDER_HPP_
