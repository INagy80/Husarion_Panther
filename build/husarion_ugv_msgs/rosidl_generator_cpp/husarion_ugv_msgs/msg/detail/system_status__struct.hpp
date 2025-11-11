// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from husarion_ugv_msgs:msg/SystemStatus.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/system_status.hpp"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__SYSTEM_STATUS__STRUCT_HPP_
#define HUSARION_UGV_MSGS__MSG__DETAIL__SYSTEM_STATUS__STRUCT_HPP_

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
# define DEPRECATED__husarion_ugv_msgs__msg__SystemStatus __attribute__((deprecated))
#else
# define DEPRECATED__husarion_ugv_msgs__msg__SystemStatus __declspec(deprecated)
#endif

namespace husarion_ugv_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SystemStatus_
{
  using Type = SystemStatus_<ContainerAllocator>;

  explicit SystemStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cpu_temp = 0.0f;
      this->avg_load_percent = 0.0f;
      this->ram_usage_percent = 0.0f;
      this->disc_usage_percent = 0.0f;
    }
  }

  explicit SystemStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->cpu_temp = 0.0f;
      this->avg_load_percent = 0.0f;
      this->ram_usage_percent = 0.0f;
      this->disc_usage_percent = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _cpu_percent_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _cpu_percent_type cpu_percent;
  using _cpu_temp_type =
    float;
  _cpu_temp_type cpu_temp;
  using _avg_load_percent_type =
    float;
  _avg_load_percent_type avg_load_percent;
  using _ram_usage_percent_type =
    float;
  _ram_usage_percent_type ram_usage_percent;
  using _disc_usage_percent_type =
    float;
  _disc_usage_percent_type disc_usage_percent;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__cpu_percent(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->cpu_percent = _arg;
    return *this;
  }
  Type & set__cpu_temp(
    const float & _arg)
  {
    this->cpu_temp = _arg;
    return *this;
  }
  Type & set__avg_load_percent(
    const float & _arg)
  {
    this->avg_load_percent = _arg;
    return *this;
  }
  Type & set__ram_usage_percent(
    const float & _arg)
  {
    this->ram_usage_percent = _arg;
    return *this;
  }
  Type & set__disc_usage_percent(
    const float & _arg)
  {
    this->disc_usage_percent = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    husarion_ugv_msgs::msg::SystemStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const husarion_ugv_msgs::msg::SystemStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<husarion_ugv_msgs::msg::SystemStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<husarion_ugv_msgs::msg::SystemStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      husarion_ugv_msgs::msg::SystemStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<husarion_ugv_msgs::msg::SystemStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      husarion_ugv_msgs::msg::SystemStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<husarion_ugv_msgs::msg::SystemStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<husarion_ugv_msgs::msg::SystemStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<husarion_ugv_msgs::msg::SystemStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__husarion_ugv_msgs__msg__SystemStatus
    std::shared_ptr<husarion_ugv_msgs::msg::SystemStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__husarion_ugv_msgs__msg__SystemStatus
    std::shared_ptr<husarion_ugv_msgs::msg::SystemStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SystemStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->cpu_percent != other.cpu_percent) {
      return false;
    }
    if (this->cpu_temp != other.cpu_temp) {
      return false;
    }
    if (this->avg_load_percent != other.avg_load_percent) {
      return false;
    }
    if (this->ram_usage_percent != other.ram_usage_percent) {
      return false;
    }
    if (this->disc_usage_percent != other.disc_usage_percent) {
      return false;
    }
    return true;
  }
  bool operator!=(const SystemStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SystemStatus_

// alias to use template instance with default allocator
using SystemStatus =
  husarion_ugv_msgs::msg::SystemStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace husarion_ugv_msgs

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__SYSTEM_STATUS__STRUCT_HPP_
