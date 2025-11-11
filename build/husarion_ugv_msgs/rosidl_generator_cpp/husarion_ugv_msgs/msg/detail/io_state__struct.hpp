// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from husarion_ugv_msgs:msg/IOState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/io_state.hpp"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__IO_STATE__STRUCT_HPP_
#define HUSARION_UGV_MSGS__MSG__DETAIL__IO_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__husarion_ugv_msgs__msg__IOState __attribute__((deprecated))
#else
# define DEPRECATED__husarion_ugv_msgs__msg__IOState __declspec(deprecated)
#endif

namespace husarion_ugv_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct IOState_
{
  using Type = IOState_<ContainerAllocator>;

  explicit IOState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->aux_power = false;
      this->charger_connected = false;
      this->charger_enabled = false;
      this->digital_power = false;
      this->fan = false;
      this->led_control = false;
      this->motor_on = false;
      this->power_button = false;
    }
  }

  explicit IOState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->aux_power = false;
      this->charger_connected = false;
      this->charger_enabled = false;
      this->digital_power = false;
      this->fan = false;
      this->led_control = false;
      this->motor_on = false;
      this->power_button = false;
    }
  }

  // field types and members
  using _aux_power_type =
    bool;
  _aux_power_type aux_power;
  using _charger_connected_type =
    bool;
  _charger_connected_type charger_connected;
  using _charger_enabled_type =
    bool;
  _charger_enabled_type charger_enabled;
  using _digital_power_type =
    bool;
  _digital_power_type digital_power;
  using _fan_type =
    bool;
  _fan_type fan;
  using _led_control_type =
    bool;
  _led_control_type led_control;
  using _motor_on_type =
    bool;
  _motor_on_type motor_on;
  using _power_button_type =
    bool;
  _power_button_type power_button;

  // setters for named parameter idiom
  Type & set__aux_power(
    const bool & _arg)
  {
    this->aux_power = _arg;
    return *this;
  }
  Type & set__charger_connected(
    const bool & _arg)
  {
    this->charger_connected = _arg;
    return *this;
  }
  Type & set__charger_enabled(
    const bool & _arg)
  {
    this->charger_enabled = _arg;
    return *this;
  }
  Type & set__digital_power(
    const bool & _arg)
  {
    this->digital_power = _arg;
    return *this;
  }
  Type & set__fan(
    const bool & _arg)
  {
    this->fan = _arg;
    return *this;
  }
  Type & set__led_control(
    const bool & _arg)
  {
    this->led_control = _arg;
    return *this;
  }
  Type & set__motor_on(
    const bool & _arg)
  {
    this->motor_on = _arg;
    return *this;
  }
  Type & set__power_button(
    const bool & _arg)
  {
    this->power_button = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    husarion_ugv_msgs::msg::IOState_<ContainerAllocator> *;
  using ConstRawPtr =
    const husarion_ugv_msgs::msg::IOState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<husarion_ugv_msgs::msg::IOState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<husarion_ugv_msgs::msg::IOState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      husarion_ugv_msgs::msg::IOState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<husarion_ugv_msgs::msg::IOState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      husarion_ugv_msgs::msg::IOState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<husarion_ugv_msgs::msg::IOState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<husarion_ugv_msgs::msg::IOState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<husarion_ugv_msgs::msg::IOState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__husarion_ugv_msgs__msg__IOState
    std::shared_ptr<husarion_ugv_msgs::msg::IOState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__husarion_ugv_msgs__msg__IOState
    std::shared_ptr<husarion_ugv_msgs::msg::IOState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const IOState_ & other) const
  {
    if (this->aux_power != other.aux_power) {
      return false;
    }
    if (this->charger_connected != other.charger_connected) {
      return false;
    }
    if (this->charger_enabled != other.charger_enabled) {
      return false;
    }
    if (this->digital_power != other.digital_power) {
      return false;
    }
    if (this->fan != other.fan) {
      return false;
    }
    if (this->led_control != other.led_control) {
      return false;
    }
    if (this->motor_on != other.motor_on) {
      return false;
    }
    if (this->power_button != other.power_button) {
      return false;
    }
    return true;
  }
  bool operator!=(const IOState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct IOState_

// alias to use template instance with default allocator
using IOState =
  husarion_ugv_msgs::msg::IOState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace husarion_ugv_msgs

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__IO_STATE__STRUCT_HPP_
