// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from husarion_ugv_msgs:msg/ScriptFlag.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/script_flag.hpp"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__SCRIPT_FLAG__STRUCT_HPP_
#define HUSARION_UGV_MSGS__MSG__DETAIL__SCRIPT_FLAG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__husarion_ugv_msgs__msg__ScriptFlag __attribute__((deprecated))
#else
# define DEPRECATED__husarion_ugv_msgs__msg__ScriptFlag __declspec(deprecated)
#endif

namespace husarion_ugv_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ScriptFlag_
{
  using Type = ScriptFlag_<ContainerAllocator>;

  explicit ScriptFlag_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->loop_error = false;
      this->encoder_disconnected = false;
      this->amp_limiter = false;
    }
  }

  explicit ScriptFlag_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->loop_error = false;
      this->encoder_disconnected = false;
      this->amp_limiter = false;
    }
  }

  // field types and members
  using _loop_error_type =
    bool;
  _loop_error_type loop_error;
  using _encoder_disconnected_type =
    bool;
  _encoder_disconnected_type encoder_disconnected;
  using _amp_limiter_type =
    bool;
  _amp_limiter_type amp_limiter;

  // setters for named parameter idiom
  Type & set__loop_error(
    const bool & _arg)
  {
    this->loop_error = _arg;
    return *this;
  }
  Type & set__encoder_disconnected(
    const bool & _arg)
  {
    this->encoder_disconnected = _arg;
    return *this;
  }
  Type & set__amp_limiter(
    const bool & _arg)
  {
    this->amp_limiter = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    husarion_ugv_msgs::msg::ScriptFlag_<ContainerAllocator> *;
  using ConstRawPtr =
    const husarion_ugv_msgs::msg::ScriptFlag_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<husarion_ugv_msgs::msg::ScriptFlag_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<husarion_ugv_msgs::msg::ScriptFlag_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      husarion_ugv_msgs::msg::ScriptFlag_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<husarion_ugv_msgs::msg::ScriptFlag_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      husarion_ugv_msgs::msg::ScriptFlag_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<husarion_ugv_msgs::msg::ScriptFlag_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<husarion_ugv_msgs::msg::ScriptFlag_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<husarion_ugv_msgs::msg::ScriptFlag_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__husarion_ugv_msgs__msg__ScriptFlag
    std::shared_ptr<husarion_ugv_msgs::msg::ScriptFlag_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__husarion_ugv_msgs__msg__ScriptFlag
    std::shared_ptr<husarion_ugv_msgs::msg::ScriptFlag_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ScriptFlag_ & other) const
  {
    if (this->loop_error != other.loop_error) {
      return false;
    }
    if (this->encoder_disconnected != other.encoder_disconnected) {
      return false;
    }
    if (this->amp_limiter != other.amp_limiter) {
      return false;
    }
    return true;
  }
  bool operator!=(const ScriptFlag_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ScriptFlag_

// alias to use template instance with default allocator
using ScriptFlag =
  husarion_ugv_msgs::msg::ScriptFlag_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace husarion_ugv_msgs

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__SCRIPT_FLAG__STRUCT_HPP_
