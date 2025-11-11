// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from husarion_ugv_msgs:msg/FaultFlag.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/fault_flag.hpp"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__FAULT_FLAG__BUILDER_HPP_
#define HUSARION_UGV_MSGS__MSG__DETAIL__FAULT_FLAG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "husarion_ugv_msgs/msg/detail/fault_flag__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace husarion_ugv_msgs
{

namespace msg
{

namespace builder
{

class Init_FaultFlag_default_config_loaded_at_startup
{
public:
  explicit Init_FaultFlag_default_config_loaded_at_startup(::husarion_ugv_msgs::msg::FaultFlag & msg)
  : msg_(msg)
  {}
  ::husarion_ugv_msgs::msg::FaultFlag default_config_loaded_at_startup(::husarion_ugv_msgs::msg::FaultFlag::_default_config_loaded_at_startup_type arg)
  {
    msg_.default_config_loaded_at_startup = std::move(arg);
    return std::move(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::FaultFlag msg_;
};

class Init_FaultFlag_mosfet_failure
{
public:
  explicit Init_FaultFlag_mosfet_failure(::husarion_ugv_msgs::msg::FaultFlag & msg)
  : msg_(msg)
  {}
  Init_FaultFlag_default_config_loaded_at_startup mosfet_failure(::husarion_ugv_msgs::msg::FaultFlag::_mosfet_failure_type arg)
  {
    msg_.mosfet_failure = std::move(arg);
    return Init_FaultFlag_default_config_loaded_at_startup(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::FaultFlag msg_;
};

class Init_FaultFlag_motor_or_sensor_setup_fault
{
public:
  explicit Init_FaultFlag_motor_or_sensor_setup_fault(::husarion_ugv_msgs::msg::FaultFlag & msg)
  : msg_(msg)
  {}
  Init_FaultFlag_mosfet_failure motor_or_sensor_setup_fault(::husarion_ugv_msgs::msg::FaultFlag::_motor_or_sensor_setup_fault_type arg)
  {
    msg_.motor_or_sensor_setup_fault = std::move(arg);
    return Init_FaultFlag_mosfet_failure(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::FaultFlag msg_;
};

class Init_FaultFlag_emergency_stop
{
public:
  explicit Init_FaultFlag_emergency_stop(::husarion_ugv_msgs::msg::FaultFlag & msg)
  : msg_(msg)
  {}
  Init_FaultFlag_motor_or_sensor_setup_fault emergency_stop(::husarion_ugv_msgs::msg::FaultFlag::_emergency_stop_type arg)
  {
    msg_.emergency_stop = std::move(arg);
    return Init_FaultFlag_motor_or_sensor_setup_fault(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::FaultFlag msg_;
};

class Init_FaultFlag_short_circuit
{
public:
  explicit Init_FaultFlag_short_circuit(::husarion_ugv_msgs::msg::FaultFlag & msg)
  : msg_(msg)
  {}
  Init_FaultFlag_emergency_stop short_circuit(::husarion_ugv_msgs::msg::FaultFlag::_short_circuit_type arg)
  {
    msg_.short_circuit = std::move(arg);
    return Init_FaultFlag_emergency_stop(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::FaultFlag msg_;
};

class Init_FaultFlag_undervoltage
{
public:
  explicit Init_FaultFlag_undervoltage(::husarion_ugv_msgs::msg::FaultFlag & msg)
  : msg_(msg)
  {}
  Init_FaultFlag_short_circuit undervoltage(::husarion_ugv_msgs::msg::FaultFlag::_undervoltage_type arg)
  {
    msg_.undervoltage = std::move(arg);
    return Init_FaultFlag_short_circuit(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::FaultFlag msg_;
};

class Init_FaultFlag_overvoltage
{
public:
  explicit Init_FaultFlag_overvoltage(::husarion_ugv_msgs::msg::FaultFlag & msg)
  : msg_(msg)
  {}
  Init_FaultFlag_undervoltage overvoltage(::husarion_ugv_msgs::msg::FaultFlag::_overvoltage_type arg)
  {
    msg_.overvoltage = std::move(arg);
    return Init_FaultFlag_undervoltage(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::FaultFlag msg_;
};

class Init_FaultFlag_overheat
{
public:
  Init_FaultFlag_overheat()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FaultFlag_overvoltage overheat(::husarion_ugv_msgs::msg::FaultFlag::_overheat_type arg)
  {
    msg_.overheat = std::move(arg);
    return Init_FaultFlag_overvoltage(msg_);
  }

private:
  ::husarion_ugv_msgs::msg::FaultFlag msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::husarion_ugv_msgs::msg::FaultFlag>()
{
  return husarion_ugv_msgs::msg::builder::Init_FaultFlag_overheat();
}

}  // namespace husarion_ugv_msgs

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__FAULT_FLAG__BUILDER_HPP_
