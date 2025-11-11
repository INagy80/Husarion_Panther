// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from husarion_ugv_msgs:msg/ChargingStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/charging_status.hpp"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__CHARGING_STATUS__STRUCT_HPP_
#define HUSARION_UGV_MSGS__MSG__DETAIL__CHARGING_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__husarion_ugv_msgs__msg__ChargingStatus __attribute__((deprecated))
#else
# define DEPRECATED__husarion_ugv_msgs__msg__ChargingStatus __declspec(deprecated)
#endif

namespace husarion_ugv_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ChargingStatus_
{
  using Type = ChargingStatus_<ContainerAllocator>;

  explicit ChargingStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->charging = false;
      this->current = 0.0f;
      this->current_battery_1 = 0.0f;
      this->current_battery_2 = 0.0f;
      this->charger_type = 0;
    }
  }

  explicit ChargingStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->charging = false;
      this->current = 0.0f;
      this->current_battery_1 = 0.0f;
      this->current_battery_2 = 0.0f;
      this->charger_type = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _charging_type =
    bool;
  _charging_type charging;
  using _current_type =
    float;
  _current_type current;
  using _current_battery_1_type =
    float;
  _current_battery_1_type current_battery_1;
  using _current_battery_2_type =
    float;
  _current_battery_2_type current_battery_2;
  using _charger_type_type =
    uint8_t;
  _charger_type_type charger_type;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__charging(
    const bool & _arg)
  {
    this->charging = _arg;
    return *this;
  }
  Type & set__current(
    const float & _arg)
  {
    this->current = _arg;
    return *this;
  }
  Type & set__current_battery_1(
    const float & _arg)
  {
    this->current_battery_1 = _arg;
    return *this;
  }
  Type & set__current_battery_2(
    const float & _arg)
  {
    this->current_battery_2 = _arg;
    return *this;
  }
  Type & set__charger_type(
    const uint8_t & _arg)
  {
    this->charger_type = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t UNKNOWN =
    0u;
  static constexpr uint8_t WIRED =
    1u;
  static constexpr uint8_t WIRELESS =
    2u;

  // pointer types
  using RawPtr =
    husarion_ugv_msgs::msg::ChargingStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const husarion_ugv_msgs::msg::ChargingStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<husarion_ugv_msgs::msg::ChargingStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<husarion_ugv_msgs::msg::ChargingStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      husarion_ugv_msgs::msg::ChargingStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<husarion_ugv_msgs::msg::ChargingStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      husarion_ugv_msgs::msg::ChargingStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<husarion_ugv_msgs::msg::ChargingStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<husarion_ugv_msgs::msg::ChargingStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<husarion_ugv_msgs::msg::ChargingStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__husarion_ugv_msgs__msg__ChargingStatus
    std::shared_ptr<husarion_ugv_msgs::msg::ChargingStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__husarion_ugv_msgs__msg__ChargingStatus
    std::shared_ptr<husarion_ugv_msgs::msg::ChargingStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ChargingStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->charging != other.charging) {
      return false;
    }
    if (this->current != other.current) {
      return false;
    }
    if (this->current_battery_1 != other.current_battery_1) {
      return false;
    }
    if (this->current_battery_2 != other.current_battery_2) {
      return false;
    }
    if (this->charger_type != other.charger_type) {
      return false;
    }
    return true;
  }
  bool operator!=(const ChargingStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ChargingStatus_

// alias to use template instance with default allocator
using ChargingStatus =
  husarion_ugv_msgs::msg::ChargingStatus_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ChargingStatus_<ContainerAllocator>::UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ChargingStatus_<ContainerAllocator>::WIRED;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t ChargingStatus_<ContainerAllocator>::WIRELESS;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace husarion_ugv_msgs

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__CHARGING_STATUS__STRUCT_HPP_
