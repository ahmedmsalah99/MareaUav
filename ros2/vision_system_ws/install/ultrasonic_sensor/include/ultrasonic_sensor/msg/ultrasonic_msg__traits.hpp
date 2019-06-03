// generated from rosidl_generator_cpp/resource/msg__traits.hpp.em
// generated code does not contain a copyright notice

#ifndef ULTRASONIC_SENSOR__MSG__ULTRASONIC_MSG__TRAITS_HPP_
#define ULTRASONIC_SENSOR__MSG__ULTRASONIC_MSG__TRAITS_HPP_

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

#include "ultrasonic_sensor/msg/ultrasonic_msg__struct.hpp"


template<>
struct has_fixed_size<ultrasonic_sensor::msg::UltrasonicMsg>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ultrasonic_sensor::msg::UltrasonicMsg>
  : std::integral_constant<bool, true> {};

template<>
inline const char * data_type<ultrasonic_sensor::msg::UltrasonicMsg>()
{
  return "ultrasonic_sensor::msg::UltrasonicMsg";
}

}  // namespace rosidl_generator_traits

#endif  // ULTRASONIC_SENSOR__MSG__ULTRASONIC_MSG__TRAITS_HPP_
