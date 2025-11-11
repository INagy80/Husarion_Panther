// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from husarion_ugv_msgs:msg/LEDAnimationQueue.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/led_animation_queue.hpp"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__LED_ANIMATION_QUEUE__STRUCT_HPP_
#define HUSARION_UGV_MSGS__MSG__DETAIL__LED_ANIMATION_QUEUE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__husarion_ugv_msgs__msg__LEDAnimationQueue __attribute__((deprecated))
#else
# define DEPRECATED__husarion_ugv_msgs__msg__LEDAnimationQueue __declspec(deprecated)
#endif

namespace husarion_ugv_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LEDAnimationQueue_
{
  using Type = LEDAnimationQueue_<ContainerAllocator>;

  explicit LEDAnimationQueue_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit LEDAnimationQueue_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _queue_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>>;
  _queue_type queue;

  // setters for named parameter idiom
  Type & set__queue(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> & _arg)
  {
    this->queue = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    husarion_ugv_msgs::msg::LEDAnimationQueue_<ContainerAllocator> *;
  using ConstRawPtr =
    const husarion_ugv_msgs::msg::LEDAnimationQueue_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<husarion_ugv_msgs::msg::LEDAnimationQueue_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<husarion_ugv_msgs::msg::LEDAnimationQueue_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      husarion_ugv_msgs::msg::LEDAnimationQueue_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<husarion_ugv_msgs::msg::LEDAnimationQueue_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      husarion_ugv_msgs::msg::LEDAnimationQueue_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<husarion_ugv_msgs::msg::LEDAnimationQueue_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<husarion_ugv_msgs::msg::LEDAnimationQueue_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<husarion_ugv_msgs::msg::LEDAnimationQueue_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__husarion_ugv_msgs__msg__LEDAnimationQueue
    std::shared_ptr<husarion_ugv_msgs::msg::LEDAnimationQueue_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__husarion_ugv_msgs__msg__LEDAnimationQueue
    std::shared_ptr<husarion_ugv_msgs::msg::LEDAnimationQueue_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LEDAnimationQueue_ & other) const
  {
    if (this->queue != other.queue) {
      return false;
    }
    return true;
  }
  bool operator!=(const LEDAnimationQueue_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LEDAnimationQueue_

// alias to use template instance with default allocator
using LEDAnimationQueue =
  husarion_ugv_msgs::msg::LEDAnimationQueue_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace husarion_ugv_msgs

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__LED_ANIMATION_QUEUE__STRUCT_HPP_
