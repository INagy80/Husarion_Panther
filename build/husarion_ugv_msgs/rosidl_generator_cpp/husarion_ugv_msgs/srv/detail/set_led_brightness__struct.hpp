// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from husarion_ugv_msgs:srv/SetLEDBrightness.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/srv/set_led_brightness.hpp"


#ifndef HUSARION_UGV_MSGS__SRV__DETAIL__SET_LED_BRIGHTNESS__STRUCT_HPP_
#define HUSARION_UGV_MSGS__SRV__DETAIL__SET_LED_BRIGHTNESS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__husarion_ugv_msgs__srv__SetLEDBrightness_Request __attribute__((deprecated))
#else
# define DEPRECATED__husarion_ugv_msgs__srv__SetLEDBrightness_Request __declspec(deprecated)
#endif

namespace husarion_ugv_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetLEDBrightness_Request_
{
  using Type = SetLEDBrightness_Request_<ContainerAllocator>;

  explicit SetLEDBrightness_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = 0.0f;
    }
  }

  explicit SetLEDBrightness_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = 0.0f;
    }
  }

  // field types and members
  using _data_type =
    float;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const float & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    husarion_ugv_msgs::srv::SetLEDBrightness_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const husarion_ugv_msgs::srv::SetLEDBrightness_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<husarion_ugv_msgs::srv::SetLEDBrightness_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<husarion_ugv_msgs::srv::SetLEDBrightness_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      husarion_ugv_msgs::srv::SetLEDBrightness_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<husarion_ugv_msgs::srv::SetLEDBrightness_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      husarion_ugv_msgs::srv::SetLEDBrightness_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<husarion_ugv_msgs::srv::SetLEDBrightness_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<husarion_ugv_msgs::srv::SetLEDBrightness_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<husarion_ugv_msgs::srv::SetLEDBrightness_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__husarion_ugv_msgs__srv__SetLEDBrightness_Request
    std::shared_ptr<husarion_ugv_msgs::srv::SetLEDBrightness_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__husarion_ugv_msgs__srv__SetLEDBrightness_Request
    std::shared_ptr<husarion_ugv_msgs::srv::SetLEDBrightness_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetLEDBrightness_Request_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetLEDBrightness_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetLEDBrightness_Request_

// alias to use template instance with default allocator
using SetLEDBrightness_Request =
  husarion_ugv_msgs::srv::SetLEDBrightness_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace husarion_ugv_msgs


#ifndef _WIN32
# define DEPRECATED__husarion_ugv_msgs__srv__SetLEDBrightness_Response __attribute__((deprecated))
#else
# define DEPRECATED__husarion_ugv_msgs__srv__SetLEDBrightness_Response __declspec(deprecated)
#endif

namespace husarion_ugv_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetLEDBrightness_Response_
{
  using Type = SetLEDBrightness_Response_<ContainerAllocator>;

  explicit SetLEDBrightness_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit SetLEDBrightness_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    husarion_ugv_msgs::srv::SetLEDBrightness_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const husarion_ugv_msgs::srv::SetLEDBrightness_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<husarion_ugv_msgs::srv::SetLEDBrightness_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<husarion_ugv_msgs::srv::SetLEDBrightness_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      husarion_ugv_msgs::srv::SetLEDBrightness_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<husarion_ugv_msgs::srv::SetLEDBrightness_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      husarion_ugv_msgs::srv::SetLEDBrightness_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<husarion_ugv_msgs::srv::SetLEDBrightness_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<husarion_ugv_msgs::srv::SetLEDBrightness_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<husarion_ugv_msgs::srv::SetLEDBrightness_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__husarion_ugv_msgs__srv__SetLEDBrightness_Response
    std::shared_ptr<husarion_ugv_msgs::srv::SetLEDBrightness_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__husarion_ugv_msgs__srv__SetLEDBrightness_Response
    std::shared_ptr<husarion_ugv_msgs::srv::SetLEDBrightness_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetLEDBrightness_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetLEDBrightness_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetLEDBrightness_Response_

// alias to use template instance with default allocator
using SetLEDBrightness_Response =
  husarion_ugv_msgs::srv::SetLEDBrightness_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace husarion_ugv_msgs


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__husarion_ugv_msgs__srv__SetLEDBrightness_Event __attribute__((deprecated))
#else
# define DEPRECATED__husarion_ugv_msgs__srv__SetLEDBrightness_Event __declspec(deprecated)
#endif

namespace husarion_ugv_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetLEDBrightness_Event_
{
  using Type = SetLEDBrightness_Event_<ContainerAllocator>;

  explicit SetLEDBrightness_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit SetLEDBrightness_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<husarion_ugv_msgs::srv::SetLEDBrightness_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<husarion_ugv_msgs::srv::SetLEDBrightness_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<husarion_ugv_msgs::srv::SetLEDBrightness_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<husarion_ugv_msgs::srv::SetLEDBrightness_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<husarion_ugv_msgs::srv::SetLEDBrightness_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<husarion_ugv_msgs::srv::SetLEDBrightness_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<husarion_ugv_msgs::srv::SetLEDBrightness_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<husarion_ugv_msgs::srv::SetLEDBrightness_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    husarion_ugv_msgs::srv::SetLEDBrightness_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const husarion_ugv_msgs::srv::SetLEDBrightness_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<husarion_ugv_msgs::srv::SetLEDBrightness_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<husarion_ugv_msgs::srv::SetLEDBrightness_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      husarion_ugv_msgs::srv::SetLEDBrightness_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<husarion_ugv_msgs::srv::SetLEDBrightness_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      husarion_ugv_msgs::srv::SetLEDBrightness_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<husarion_ugv_msgs::srv::SetLEDBrightness_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<husarion_ugv_msgs::srv::SetLEDBrightness_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<husarion_ugv_msgs::srv::SetLEDBrightness_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__husarion_ugv_msgs__srv__SetLEDBrightness_Event
    std::shared_ptr<husarion_ugv_msgs::srv::SetLEDBrightness_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__husarion_ugv_msgs__srv__SetLEDBrightness_Event
    std::shared_ptr<husarion_ugv_msgs::srv::SetLEDBrightness_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetLEDBrightness_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetLEDBrightness_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetLEDBrightness_Event_

// alias to use template instance with default allocator
using SetLEDBrightness_Event =
  husarion_ugv_msgs::srv::SetLEDBrightness_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace husarion_ugv_msgs

namespace husarion_ugv_msgs
{

namespace srv
{

struct SetLEDBrightness
{
  using Request = husarion_ugv_msgs::srv::SetLEDBrightness_Request;
  using Response = husarion_ugv_msgs::srv::SetLEDBrightness_Response;
  using Event = husarion_ugv_msgs::srv::SetLEDBrightness_Event;
};

}  // namespace srv

}  // namespace husarion_ugv_msgs

#endif  // HUSARION_UGV_MSGS__SRV__DETAIL__SET_LED_BRIGHTNESS__STRUCT_HPP_
