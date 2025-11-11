// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from husarion_ugv_msgs:msg/FaultFlag.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/fault_flag.hpp"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__FAULT_FLAG__STRUCT_HPP_
#define HUSARION_UGV_MSGS__MSG__DETAIL__FAULT_FLAG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__husarion_ugv_msgs__msg__FaultFlag __attribute__((deprecated))
#else
# define DEPRECATED__husarion_ugv_msgs__msg__FaultFlag __declspec(deprecated)
#endif

namespace husarion_ugv_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct FaultFlag_
{
  using Type = FaultFlag_<ContainerAllocator>;

  explicit FaultFlag_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->overheat = false;
      this->overvoltage = false;
      this->undervoltage = false;
      this->short_circuit = false;
      this->emergency_stop = false;
      this->motor_or_sensor_setup_fault = false;
      this->mosfet_failure = false;
      this->default_config_loaded_at_startup = false;
    }
  }

  explicit FaultFlag_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->overheat = false;
      this->overvoltage = false;
      this->undervoltage = false;
      this->short_circuit = false;
      this->emergency_stop = false;
      this->motor_or_sensor_setup_fault = false;
      this->mosfet_failure = false;
      this->default_config_loaded_at_startup = false;
    }
  }

  // field types and members
  using _overheat_type =
    bool;
  _overheat_type overheat;
  using _overvoltage_type =
    bool;
  _overvoltage_type overvoltage;
  using _undervoltage_type =
    bool;
  _undervoltage_type undervoltage;
  using _short_circuit_type =
    bool;
  _short_circuit_type short_circuit;
  using _emergency_stop_type =
    bool;
  _emergency_stop_type emergency_stop;
  using _motor_or_sensor_setup_fault_type =
    bool;
  _motor_or_sensor_setup_fault_type motor_or_sensor_setup_fault;
  using _mosfet_failure_type =
    bool;
  _mosfet_failure_type mosfet_failure;
  using _default_config_loaded_at_startup_type =
    bool;
  _default_config_loaded_at_startup_type default_config_loaded_at_startup;

  // setters for named parameter idiom
  Type & set__overheat(
    const bool & _arg)
  {
    this->overheat = _arg;
    return *this;
  }
  Type & set__overvoltage(
    const bool & _arg)
  {
    this->overvoltage = _arg;
    return *this;
  }
  Type & set__undervoltage(
    const bool & _arg)
  {
    this->undervoltage = _arg;
    return *this;
  }
  Type & set__short_circuit(
    const bool & _arg)
  {
    this->short_circuit = _arg;
    return *this;
  }
  Type & set__emergency_stop(
    const bool & _arg)
  {
    this->emergency_stop = _arg;
    return *this;
  }
  Type & set__motor_or_sensor_setup_fault(
    const bool & _arg)
  {
    this->motor_or_sensor_setup_fault = _arg;
    return *this;
  }
  Type & set__mosfet_failure(
    const bool & _arg)
  {
    this->mosfet_failure = _arg;
    return *this;
  }
  Type & set__default_config_loaded_at_startup(
    const bool & _arg)
  {
    this->default_config_loaded_at_startup = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    husarion_ugv_msgs::msg::FaultFlag_<ContainerAllocator> *;
  using ConstRawPtr =
    const husarion_ugv_msgs::msg::FaultFlag_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<husarion_ugv_msgs::msg::FaultFlag_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<husarion_ugv_msgs::msg::FaultFlag_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      husarion_ugv_msgs::msg::FaultFlag_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<husarion_ugv_msgs::msg::FaultFlag_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      husarion_ugv_msgs::msg::FaultFlag_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<husarion_ugv_msgs::msg::FaultFlag_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<husarion_ugv_msgs::msg::FaultFlag_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<husarion_ugv_msgs::msg::FaultFlag_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__husarion_ugv_msgs__msg__FaultFlag
    std::shared_ptr<husarion_ugv_msgs::msg::FaultFlag_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__husarion_ugv_msgs__msg__FaultFlag
    std::shared_ptr<husarion_ugv_msgs::msg::FaultFlag_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const FaultFlag_ & other) const
  {
    if (this->overheat != other.overheat) {
      return false;
    }
    if (this->overvoltage != other.overvoltage) {
      return false;
    }
    if (this->undervoltage != other.undervoltage) {
      return false;
    }
    if (this->short_circuit != other.short_circuit) {
      return false;
    }
    if (this->emergency_stop != other.emergency_stop) {
      return false;
    }
    if (this->motor_or_sensor_setup_fault != other.motor_or_sensor_setup_fault) {
      return false;
    }
    if (this->mosfet_failure != other.mosfet_failure) {
      return false;
    }
    if (this->default_config_loaded_at_startup != other.default_config_loaded_at_startup) {
      return false;
    }
    return true;
  }
  bool operator!=(const FaultFlag_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct FaultFlag_

// alias to use template instance with default allocator
using FaultFlag =
  husarion_ugv_msgs::msg::FaultFlag_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace husarion_ugv_msgs

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__FAULT_FLAG__STRUCT_HPP_
