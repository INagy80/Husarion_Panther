// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from husarion_ugv_msgs:msg/LEDImageAnimation.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/led_image_animation.hpp"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__LED_IMAGE_ANIMATION__STRUCT_HPP_
#define HUSARION_UGV_MSGS__MSG__DETAIL__LED_IMAGE_ANIMATION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__husarion_ugv_msgs__msg__LEDImageAnimation __attribute__((deprecated))
#else
# define DEPRECATED__husarion_ugv_msgs__msg__LEDImageAnimation __declspec(deprecated)
#endif

namespace husarion_ugv_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LEDImageAnimation_
{
  using Type = LEDImageAnimation_<ContainerAllocator>;

  explicit LEDImageAnimation_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->image = "";
      this->duration = 0.0f;
      this->brightness = 0.0f;
      this->repeat = 0;
      this->color = 0ul;
    }
  }

  explicit LEDImageAnimation_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : image(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->image = "";
      this->duration = 0.0f;
      this->brightness = 0.0f;
      this->repeat = 0;
      this->color = 0ul;
    }
  }

  // field types and members
  using _image_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _image_type image;
  using _duration_type =
    float;
  _duration_type duration;
  using _brightness_type =
    float;
  _brightness_type brightness;
  using _repeat_type =
    uint8_t;
  _repeat_type repeat;
  using _color_type =
    uint32_t;
  _color_type color;

  // setters for named parameter idiom
  Type & set__image(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->image = _arg;
    return *this;
  }
  Type & set__duration(
    const float & _arg)
  {
    this->duration = _arg;
    return *this;
  }
  Type & set__brightness(
    const float & _arg)
  {
    this->brightness = _arg;
    return *this;
  }
  Type & set__repeat(
    const uint8_t & _arg)
  {
    this->repeat = _arg;
    return *this;
  }
  Type & set__color(
    const uint32_t & _arg)
  {
    this->color = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    husarion_ugv_msgs::msg::LEDImageAnimation_<ContainerAllocator> *;
  using ConstRawPtr =
    const husarion_ugv_msgs::msg::LEDImageAnimation_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<husarion_ugv_msgs::msg::LEDImageAnimation_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<husarion_ugv_msgs::msg::LEDImageAnimation_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      husarion_ugv_msgs::msg::LEDImageAnimation_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<husarion_ugv_msgs::msg::LEDImageAnimation_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      husarion_ugv_msgs::msg::LEDImageAnimation_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<husarion_ugv_msgs::msg::LEDImageAnimation_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<husarion_ugv_msgs::msg::LEDImageAnimation_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<husarion_ugv_msgs::msg::LEDImageAnimation_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__husarion_ugv_msgs__msg__LEDImageAnimation
    std::shared_ptr<husarion_ugv_msgs::msg::LEDImageAnimation_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__husarion_ugv_msgs__msg__LEDImageAnimation
    std::shared_ptr<husarion_ugv_msgs::msg::LEDImageAnimation_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LEDImageAnimation_ & other) const
  {
    if (this->image != other.image) {
      return false;
    }
    if (this->duration != other.duration) {
      return false;
    }
    if (this->brightness != other.brightness) {
      return false;
    }
    if (this->repeat != other.repeat) {
      return false;
    }
    if (this->color != other.color) {
      return false;
    }
    return true;
  }
  bool operator!=(const LEDImageAnimation_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LEDImageAnimation_

// alias to use template instance with default allocator
using LEDImageAnimation =
  husarion_ugv_msgs::msg::LEDImageAnimation_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace husarion_ugv_msgs

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__LED_IMAGE_ANIMATION__STRUCT_HPP_
