// generated from
// rosidl_typesupport_fastrtps_cpp/resource/msg__rosidl_typesupport_fastrtps_cpp.hpp.em
// generated code does not contain a copyright notice

#ifndef ULTRASONIC_SENSOR__MSG__ULTRASONIC_MSG__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define ULTRASONIC_SENSOR__MSG__ULTRASONIC_MSG__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_generator_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"

#include "ultrasonic_sensor/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"

#include "ultrasonic_sensor/msg/ultrasonic_msg__struct.hpp"
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace ultrasonic_sensor
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ultrasonic_sensor
cdr_serialize(
  const ultrasonic_sensor::msg::UltrasonicMsg & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ultrasonic_sensor
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ultrasonic_sensor::msg::UltrasonicMsg & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ultrasonic_sensor
get_serialized_size(
  const ultrasonic_sensor::msg::UltrasonicMsg & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ultrasonic_sensor
max_serialized_size_UltrasonicMsg(
  bool & full_bounded,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace ultrasonic_sensor

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ultrasonic_sensor
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ultrasonic_sensor, msg, UltrasonicMsg)();

#ifdef __cplusplus
}
#endif

#endif  // ULTRASONIC_SENSOR__MSG__ULTRASONIC_MSG__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
