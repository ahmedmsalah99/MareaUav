// generated from rosidl_generator_cpp/resource/msg__traits.hpp.em
// generated code does not contain a copyright notice

#ifndef APRILTAG_MSGS__MSG__POINT__TRAITS_HPP_
#define APRILTAG_MSGS__MSG__POINT__TRAITS_HPP_

#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

#ifndef __ROSIDL_GENERATOR_CPP_TRAITS
#define __ROSIDL_GENERATOR_CPP_TRAITS

template<typename T>
inline const char * data_type();

template<typename T>
struct has_fixed_size : std::false_type {};

template<typename T>
struct has_bounded_size : std::false_type {};

#endif  // __ROSIDL_GENERATOR_CPP_TRAITS

#include "apriltag_msgs/msg/point__struct.hpp"


template<>
struct has_fixed_size<apriltag_msgs::msg::Point>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<apriltag_msgs::msg::Point>
  : std::integral_constant<bool, true> {};

template<>
inline const char * data_type<apriltag_msgs::msg::Point>()
{
  return "apriltag_msgs::msg::Point";
}

}  // namespace rosidl_generator_traits

#endif  // APRILTAG_MSGS__MSG__POINT__TRAITS_HPP_
