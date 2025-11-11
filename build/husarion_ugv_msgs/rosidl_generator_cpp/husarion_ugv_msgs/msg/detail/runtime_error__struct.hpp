// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from husarion_ugv_msgs:msg/RuntimeError.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/runtime_error.hpp"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__RUNTIME_ERROR__STRUCT_HPP_
#define HUSARION_UGV_MSGS__MSG__DETAIL__RUNTIME_ERROR__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__husarion_ugv_msgs__msg__RuntimeError __attribute__((deprecated))
#else
# define DEPRECATED__husarion_ugv_msgs__msg__RuntimeError __declspec(deprecated)
#endif

namespace husarion_ugv_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RuntimeError_
{
  using Type = RuntimeError_<ContainerAllocator>;

  explicit RuntimeError_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->amps_limit_active = false;
      this->motor_stall = false;
      this->loop_error = false;
      this->safety_stop_active = false;
      this->forward_limit_triggered = false;
      this->reverse_limit_triggered = false;
      this->amps_trigger_activated = false;
    }
  }

  explicit RuntimeError_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->amps_limit_active = false;
      this->motor_stall = false;
      this->loop_error = false;
      this->safety_stop_active = false;
      this->forward_limit_triggered = false;
      this->reverse_limit_triggered = false;
      this->amps_trigger_activated = false;
    }
  }

  // field types and members
  using _amps_limit_active_type =
    bool;
  _amps_limit_active_type amps_limit_active;
  using _motor_stall_type =
    bool;
  _motor_stall_type motor_stall;
  using _loop_error_type =
    bool;
  _loop_error_type loop_error;
  using _safety_stop_active_type =
    bool;
  _safety_stop_active_type safety_stop_active;
  using _forward_limit_triggered_type =
    bool;
  _forward_limit_triggered_type forward_limit_triggered;
  using _reverse_limit_triggered_type =
    bool;
  _reverse_limit_triggered_type reverse_limit_triggered;
  using _amps_trigger_activated_type =
    bool;
  _amps_trigger_activated_type amps_trigger_activated;

  // setters for named parameter idiom
  Type & set__amps_limit_active(
    const bool & _arg)
  {
    this->amps_limit_active = _arg;
    return *this;
  }
  Type & set__motor_stall(
    const bool & _arg)
  {
    this->motor_stall = _arg;
    return *this;
  }
  Type & set__loop_error(
    const bool & _arg)
  {
    this->loop_error = _arg;
    return *this;
  }
  Type & set__safety_stop_active(
    const bool & _arg)
  {
    this->safety_stop_active = _arg;
    return *this;
  }
  Type & set__forward_limit_triggered(
    const bool & _arg)
  {
    this->forward_limit_triggered = _arg;
    return *this;
  }
  Type & set__reverse_limit_triggered(
    const bool & _arg)
  {
    this->reverse_limit_triggered = _arg;
    return *this;
  }
  Type & set__amps_trigger_activated(
    const bool & _arg)
  {
    this->amps_trigger_activated = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    husarion_ugv_msgs::msg::RuntimeError_<ContainerAllocator> *;
  using ConstRawPtr =
    const husarion_ugv_msgs::msg::RuntimeError_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<husarion_ugv_msgs::msg::RuntimeError_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<husarion_ugv_msgs::msg::RuntimeError_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      husarion_ugv_msgs::msg::RuntimeError_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<husarion_ugv_msgs::msg::RuntimeError_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      husarion_ugv_msgs::msg::RuntimeError_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<husarion_ugv_msgs::msg::RuntimeError_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<husarion_ugv_msgs::msg::RuntimeError_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<husarion_ugv_msgs::msg::RuntimeError_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__husarion_ugv_msgs__msg__RuntimeError
    std::shared_ptr<husarion_ugv_msgs::msg::RuntimeError_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__husarion_ugv_msgs__msg__RuntimeError
    std::shared_ptr<husarion_ugv_msgs::msg::RuntimeError_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RuntimeError_ & other) const
  {
    if (this->amps_limit_active != other.amps_limit_active) {
      return false;
    }
    if (this->motor_stall != other.motor_stall) {
      return false;
    }
    if (this->loop_error != other.loop_error) {
      return false;
    }
    if (this->safety_stop_active != other.safety_stop_active) {
      return false;
    }
    if (this->forward_limit_triggered != other.forward_limit_triggered) {
      return false;
    }
    if (this->reverse_limit_triggered != other.reverse_limit_triggered) {
      return false;
    }
    if (this->amps_trigger_activated != other.amps_trigger_activated) {
      return false;
    }
    return true;
  }
  bool operator!=(const RuntimeError_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RuntimeError_

// alias to use template instance with default allocator
using RuntimeError =
  husarion_ugv_msgs::msg::RuntimeError_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace husarion_ugv_msgs

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__RUNTIME_ERROR__STRUCT_HPP_
