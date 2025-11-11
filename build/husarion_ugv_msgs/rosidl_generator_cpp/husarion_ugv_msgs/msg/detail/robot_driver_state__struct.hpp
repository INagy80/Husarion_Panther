// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from husarion_ugv_msgs:msg/RobotDriverState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/robot_driver_state.hpp"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__ROBOT_DRIVER_STATE__STRUCT_HPP_
#define HUSARION_UGV_MSGS__MSG__DETAIL__ROBOT_DRIVER_STATE__STRUCT_HPP_

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
// Member 'driver_states'
#include "husarion_ugv_msgs/msg/detail/driver_state_named__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__husarion_ugv_msgs__msg__RobotDriverState __attribute__((deprecated))
#else
# define DEPRECATED__husarion_ugv_msgs__msg__RobotDriverState __declspec(deprecated)
#endif

namespace husarion_ugv_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RobotDriverState_
{
  using Type = RobotDriverState_<ContainerAllocator>;

  explicit RobotDriverState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->error = false;
      this->write_pdo_cmds_error = false;
      this->read_pdo_motor_states_error = false;
      this->read_pdo_driver_state_error = false;
    }
  }

  explicit RobotDriverState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->error = false;
      this->write_pdo_cmds_error = false;
      this->read_pdo_motor_states_error = false;
      this->read_pdo_driver_state_error = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _driver_states_type =
    std::vector<husarion_ugv_msgs::msg::DriverStateNamed_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<husarion_ugv_msgs::msg::DriverStateNamed_<ContainerAllocator>>>;
  _driver_states_type driver_states;
  using _error_type =
    bool;
  _error_type error;
  using _write_pdo_cmds_error_type =
    bool;
  _write_pdo_cmds_error_type write_pdo_cmds_error;
  using _read_pdo_motor_states_error_type =
    bool;
  _read_pdo_motor_states_error_type read_pdo_motor_states_error;
  using _read_pdo_driver_state_error_type =
    bool;
  _read_pdo_driver_state_error_type read_pdo_driver_state_error;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__driver_states(
    const std::vector<husarion_ugv_msgs::msg::DriverStateNamed_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<husarion_ugv_msgs::msg::DriverStateNamed_<ContainerAllocator>>> & _arg)
  {
    this->driver_states = _arg;
    return *this;
  }
  Type & set__error(
    const bool & _arg)
  {
    this->error = _arg;
    return *this;
  }
  Type & set__write_pdo_cmds_error(
    const bool & _arg)
  {
    this->write_pdo_cmds_error = _arg;
    return *this;
  }
  Type & set__read_pdo_motor_states_error(
    const bool & _arg)
  {
    this->read_pdo_motor_states_error = _arg;
    return *this;
  }
  Type & set__read_pdo_driver_state_error(
    const bool & _arg)
  {
    this->read_pdo_driver_state_error = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    husarion_ugv_msgs::msg::RobotDriverState_<ContainerAllocator> *;
  using ConstRawPtr =
    const husarion_ugv_msgs::msg::RobotDriverState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<husarion_ugv_msgs::msg::RobotDriverState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<husarion_ugv_msgs::msg::RobotDriverState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      husarion_ugv_msgs::msg::RobotDriverState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<husarion_ugv_msgs::msg::RobotDriverState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      husarion_ugv_msgs::msg::RobotDriverState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<husarion_ugv_msgs::msg::RobotDriverState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<husarion_ugv_msgs::msg::RobotDriverState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<husarion_ugv_msgs::msg::RobotDriverState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__husarion_ugv_msgs__msg__RobotDriverState
    std::shared_ptr<husarion_ugv_msgs::msg::RobotDriverState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__husarion_ugv_msgs__msg__RobotDriverState
    std::shared_ptr<husarion_ugv_msgs::msg::RobotDriverState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotDriverState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->driver_states != other.driver_states) {
      return false;
    }
    if (this->error != other.error) {
      return false;
    }
    if (this->write_pdo_cmds_error != other.write_pdo_cmds_error) {
      return false;
    }
    if (this->read_pdo_motor_states_error != other.read_pdo_motor_states_error) {
      return false;
    }
    if (this->read_pdo_driver_state_error != other.read_pdo_driver_state_error) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotDriverState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotDriverState_

// alias to use template instance with default allocator
using RobotDriverState =
  husarion_ugv_msgs::msg::RobotDriverState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace husarion_ugv_msgs

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__ROBOT_DRIVER_STATE__STRUCT_HPP_
