// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from husarion_ugv_msgs:msg/DriverState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/driver_state.hpp"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__DRIVER_STATE__STRUCT_HPP_
#define HUSARION_UGV_MSGS__MSG__DETAIL__DRIVER_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'fault_flag'
#include "husarion_ugv_msgs/msg/detail/fault_flag__struct.hpp"
// Member 'script_flag'
#include "husarion_ugv_msgs/msg/detail/script_flag__struct.hpp"
// Member 'channel_1_motor_runtime_error'
// Member 'channel_2_motor_runtime_error'
#include "husarion_ugv_msgs/msg/detail/runtime_error__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__husarion_ugv_msgs__msg__DriverState __attribute__((deprecated))
#else
# define DEPRECATED__husarion_ugv_msgs__msg__DriverState __declspec(deprecated)
#endif

namespace husarion_ugv_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DriverState_
{
  using Type = DriverState_<ContainerAllocator>;

  explicit DriverState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : fault_flag(_init),
    script_flag(_init),
    channel_1_motor_runtime_error(_init),
    channel_2_motor_runtime_error(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->voltage = 0.0f;
      this->current = 0.0f;
      this->temperature = 0.0f;
      this->heatsink_temperature = 0.0f;
      this->motor_states_data_timed_out = false;
      this->driver_state_data_timed_out = false;
      this->can_error = false;
      this->heartbeat_timeout = false;
    }
  }

  explicit DriverState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : fault_flag(_alloc, _init),
    script_flag(_alloc, _init),
    channel_1_motor_runtime_error(_alloc, _init),
    channel_2_motor_runtime_error(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->voltage = 0.0f;
      this->current = 0.0f;
      this->temperature = 0.0f;
      this->heatsink_temperature = 0.0f;
      this->motor_states_data_timed_out = false;
      this->driver_state_data_timed_out = false;
      this->can_error = false;
      this->heartbeat_timeout = false;
    }
  }

  // field types and members
  using _voltage_type =
    float;
  _voltage_type voltage;
  using _current_type =
    float;
  _current_type current;
  using _temperature_type =
    float;
  _temperature_type temperature;
  using _heatsink_temperature_type =
    float;
  _heatsink_temperature_type heatsink_temperature;
  using _fault_flag_type =
    husarion_ugv_msgs::msg::FaultFlag_<ContainerAllocator>;
  _fault_flag_type fault_flag;
  using _script_flag_type =
    husarion_ugv_msgs::msg::ScriptFlag_<ContainerAllocator>;
  _script_flag_type script_flag;
  using _channel_1_motor_runtime_error_type =
    husarion_ugv_msgs::msg::RuntimeError_<ContainerAllocator>;
  _channel_1_motor_runtime_error_type channel_1_motor_runtime_error;
  using _channel_2_motor_runtime_error_type =
    husarion_ugv_msgs::msg::RuntimeError_<ContainerAllocator>;
  _channel_2_motor_runtime_error_type channel_2_motor_runtime_error;
  using _motor_states_data_timed_out_type =
    bool;
  _motor_states_data_timed_out_type motor_states_data_timed_out;
  using _driver_state_data_timed_out_type =
    bool;
  _driver_state_data_timed_out_type driver_state_data_timed_out;
  using _can_error_type =
    bool;
  _can_error_type can_error;
  using _heartbeat_timeout_type =
    bool;
  _heartbeat_timeout_type heartbeat_timeout;

  // setters for named parameter idiom
  Type & set__voltage(
    const float & _arg)
  {
    this->voltage = _arg;
    return *this;
  }
  Type & set__current(
    const float & _arg)
  {
    this->current = _arg;
    return *this;
  }
  Type & set__temperature(
    const float & _arg)
  {
    this->temperature = _arg;
    return *this;
  }
  Type & set__heatsink_temperature(
    const float & _arg)
  {
    this->heatsink_temperature = _arg;
    return *this;
  }
  Type & set__fault_flag(
    const husarion_ugv_msgs::msg::FaultFlag_<ContainerAllocator> & _arg)
  {
    this->fault_flag = _arg;
    return *this;
  }
  Type & set__script_flag(
    const husarion_ugv_msgs::msg::ScriptFlag_<ContainerAllocator> & _arg)
  {
    this->script_flag = _arg;
    return *this;
  }
  Type & set__channel_1_motor_runtime_error(
    const husarion_ugv_msgs::msg::RuntimeError_<ContainerAllocator> & _arg)
  {
    this->channel_1_motor_runtime_error = _arg;
    return *this;
  }
  Type & set__channel_2_motor_runtime_error(
    const husarion_ugv_msgs::msg::RuntimeError_<ContainerAllocator> & _arg)
  {
    this->channel_2_motor_runtime_error = _arg;
    return *this;
  }
  Type & set__motor_states_data_timed_out(
    const bool & _arg)
  {
    this->motor_states_data_timed_out = _arg;
    return *this;
  }
  Type & set__driver_state_data_timed_out(
    const bool & _arg)
  {
    this->driver_state_data_timed_out = _arg;
    return *this;
  }
  Type & set__can_error(
    const bool & _arg)
  {
    this->can_error = _arg;
    return *this;
  }
  Type & set__heartbeat_timeout(
    const bool & _arg)
  {
    this->heartbeat_timeout = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    husarion_ugv_msgs::msg::DriverState_<ContainerAllocator> *;
  using ConstRawPtr =
    const husarion_ugv_msgs::msg::DriverState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<husarion_ugv_msgs::msg::DriverState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<husarion_ugv_msgs::msg::DriverState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      husarion_ugv_msgs::msg::DriverState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<husarion_ugv_msgs::msg::DriverState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      husarion_ugv_msgs::msg::DriverState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<husarion_ugv_msgs::msg::DriverState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<husarion_ugv_msgs::msg::DriverState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<husarion_ugv_msgs::msg::DriverState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__husarion_ugv_msgs__msg__DriverState
    std::shared_ptr<husarion_ugv_msgs::msg::DriverState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__husarion_ugv_msgs__msg__DriverState
    std::shared_ptr<husarion_ugv_msgs::msg::DriverState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DriverState_ & other) const
  {
    if (this->voltage != other.voltage) {
      return false;
    }
    if (this->current != other.current) {
      return false;
    }
    if (this->temperature != other.temperature) {
      return false;
    }
    if (this->heatsink_temperature != other.heatsink_temperature) {
      return false;
    }
    if (this->fault_flag != other.fault_flag) {
      return false;
    }
    if (this->script_flag != other.script_flag) {
      return false;
    }
    if (this->channel_1_motor_runtime_error != other.channel_1_motor_runtime_error) {
      return false;
    }
    if (this->channel_2_motor_runtime_error != other.channel_2_motor_runtime_error) {
      return false;
    }
    if (this->motor_states_data_timed_out != other.motor_states_data_timed_out) {
      return false;
    }
    if (this->driver_state_data_timed_out != other.driver_state_data_timed_out) {
      return false;
    }
    if (this->can_error != other.can_error) {
      return false;
    }
    if (this->heartbeat_timeout != other.heartbeat_timeout) {
      return false;
    }
    return true;
  }
  bool operator!=(const DriverState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DriverState_

// alias to use template instance with default allocator
using DriverState =
  husarion_ugv_msgs::msg::DriverState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace husarion_ugv_msgs

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__DRIVER_STATE__STRUCT_HPP_
