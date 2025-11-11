// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from husarion_ugv_msgs:msg/LEDAnimationQueue.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "husarion_ugv_msgs/msg/led_animation_queue.hpp"


#ifndef HUSARION_UGV_MSGS__MSG__DETAIL__LED_ANIMATION_QUEUE__TRAITS_HPP_
#define HUSARION_UGV_MSGS__MSG__DETAIL__LED_ANIMATION_QUEUE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "husarion_ugv_msgs/msg/detail/led_animation_queue__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace husarion_ugv_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const LEDAnimationQueue & msg,
  std::ostream & out)
{
  out << "{";
  // member: queue
  {
    if (msg.queue.size() == 0) {
      out << "queue: []";
    } else {
      out << "queue: [";
      size_t pending_items = msg.queue.size();
      for (auto item : msg.queue) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const LEDAnimationQueue & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: queue
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.queue.size() == 0) {
      out << "queue: []\n";
    } else {
      out << "queue:\n";
      for (auto item : msg.queue) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const LEDAnimationQueue & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace husarion_ugv_msgs

namespace rosidl_generator_traits
{

[[deprecated("use husarion_ugv_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const husarion_ugv_msgs::msg::LEDAnimationQueue & msg,
  std::ostream & out, size_t indentation = 0)
{
  husarion_ugv_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use husarion_ugv_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const husarion_ugv_msgs::msg::LEDAnimationQueue & msg)
{
  return husarion_ugv_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<husarion_ugv_msgs::msg::LEDAnimationQueue>()
{
  return "husarion_ugv_msgs::msg::LEDAnimationQueue";
}

template<>
inline const char * name<husarion_ugv_msgs::msg::LEDAnimationQueue>()
{
  return "husarion_ugv_msgs/msg/LEDAnimationQueue";
}

template<>
struct has_fixed_size<husarion_ugv_msgs::msg::LEDAnimationQueue>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<husarion_ugv_msgs::msg::LEDAnimationQueue>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<husarion_ugv_msgs::msg::LEDAnimationQueue>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HUSARION_UGV_MSGS__MSG__DETAIL__LED_ANIMATION_QUEUE__TRAITS_HPP_
