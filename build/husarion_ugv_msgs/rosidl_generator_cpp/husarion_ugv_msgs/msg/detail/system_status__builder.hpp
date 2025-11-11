// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from husarion_ugv_msgs:msg/SystemStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/system_status.hpp"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__SYSTEM_STATUS__BUILDER_HPP_
#define HUSARION_UGV_MSGS__MSG__DETAIL__SYSTEM_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "husarion_ugv_msgs/msg/detail/system_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace husarion_ugv_msgs
{

namespace msg
{

namespace builder
{

class Init_SystemStatus_disc_usage_percent
{
public:
  explicit Init_SystemStatus_disc_usage_percent(::husarion_ugv_msgs::msg::SystemStatus & msg)
  : msg_(msg)
  {}
  ::husarion_ugv_msgs::msg::SystemStatus disc_usage_percent(::husarion_ugv_msgs::msg::SystemStatus::_disc_usage_percent_type arg)
  {
    msg_.disc_usage_percent = std::move(arg);
    return std::move(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::SystemStatus msg_;
};

class Init_SystemStatus_ram_usage_percent
{
public:
  explicit Init_SystemStatus_ram_usage_percent(::husarion_ugv_msgs::msg::SystemStatus & msg)
  : msg_(msg)
  {}
  Init_SystemStatus_disc_usage_percent ram_usage_percent(::husarion_ugv_msgs::msg::SystemStatus::_ram_usage_percent_type arg)
  {
    msg_.ram_usage_percent = std::move(arg);
    return Init_SystemStatus_disc_usage_percent(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::SystemStatus msg_;
};

class Init_SystemStatus_avg_load_percent
{
public:
  explicit Init_SystemStatus_avg_load_percent(::husarion_ugv_msgs::msg::SystemStatus & msg)
  : msg_(msg)
  {}
  Init_SystemStatus_ram_usage_percent avg_load_percent(::husarion_ugv_msgs::msg::SystemStatus::_avg_load_percent_type arg)
  {
    msg_.avg_load_percent = std::move(arg);
    return Init_SystemStatus_ram_usage_percent(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::SystemStatus msg_;
};

class Init_SystemStatus_cpu_temp
{
public:
  explicit Init_SystemStatus_cpu_temp(::husarion_ugv_msgs::msg::SystemStatus & msg)
  : msg_(msg)
  {}
  Init_SystemStatus_avg_load_percent cpu_temp(::husarion_ugv_msgs::msg::SystemStatus::_cpu_temp_type arg)
  {
    msg_.cpu_temp = std::move(arg);
    return Init_SystemStatus_avg_load_percent(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::SystemStatus msg_;
};

class Init_SystemStatus_cpu_percent
{
public:
  explicit Init_SystemStatus_cpu_percent(::husarion_ugv_msgs::msg::SystemStatus & msg)
  : msg_(msg)
  {}
  Init_SystemStatus_cpu_temp cpu_percent(::husarion_ugv_msgs::msg::SystemStatus::_cpu_percent_type arg)
  {
    msg_.cpu_percent = std::move(arg);
    return Init_SystemStatus_cpu_temp(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::SystemStatus msg_;
};

class Init_SystemStatus_header
{
public:
  Init_SystemStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SystemStatus_cpu_percent header(::husarion_ugv_msgs::msg::SystemStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_SystemStatus_cpu_percent(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::SystemStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::husarion_ugv_msgs::msg::SystemStatus>()
{
  return husarion_ugv_msgs::msg::builder::Init_SystemStatus_header();
}

}  // namespace husarion_ugv_msgs

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__SYSTEM_STATUS__BUILDER_HPP_
