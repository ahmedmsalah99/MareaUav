// generated from rosidl_typesupport_fastrtps_cpp/resource/msg__type_support.cpp.em
// generated code does not contain a copyright notice

#include "ultrasonic_sensor/msg/ultrasonic_msg__rosidl_typesupport_fastrtps_cpp.hpp"

#include <limits>
#include <stdexcept>
#include <string>

#include "rosidl_typesupport_cpp/message_type_support.hpp"

#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"

#include "fastcdr/Cdr.h"

// forward declaration of message dependencies and their conversion functions

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
  eprosima::fastcdr::Cdr & cdr)
{
  // field.name z_position
  cdr << ros_message.z_position;
  // field.name z_velocity
  cdr << ros_message.z_velocity;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ultrasonic_sensor
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ultrasonic_sensor::msg::UltrasonicMsg & ros_message)
{
  // field.name z_position
  cdr >> ros_message.z_position;

  // field.name z_velocity
  cdr >> ros_message.z_velocity;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ultrasonic_sensor
get_serialized_size(
  const ultrasonic_sensor::msg::UltrasonicMsg & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  (void)padding;

  // field.name z_position
  {
    size_t item_size = sizeof(ros_message.z_position);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name z_velocity
  {
    size_t item_size = sizeof(ros_message.z_velocity);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ultrasonic_sensor
max_serialized_size_UltrasonicMsg(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  (void)padding;
  (void)full_bounded;

  // field.name z_position
  {
    size_t array_size = 1;


    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // field.name z_velocity
  {
    size_t array_size = 1;


    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  return current_alignment - initial_alignment;
}

static bool __cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const ultrasonic_sensor::msg::UltrasonicMsg *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool __cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<ultrasonic_sensor::msg::UltrasonicMsg *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t __get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const ultrasonic_sensor::msg::UltrasonicMsg *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t __max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_UltrasonicMsg(full_bounded, 0);
}

static message_type_support_callbacks_t callbacks = {
  "ultrasonic_sensor",
  "UltrasonicMsg",
  __cdr_serialize,
  __cdr_deserialize,
  __get_serialized_size,
  __max_serialized_size
};

static rosidl_message_type_support_t handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace ultrasonic_sensor

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_ultrasonic_sensor
const rosidl_message_type_support_t *
get_message_type_support_handle<ultrasonic_sensor::msg::UltrasonicMsg>()
{
  return &ultrasonic_sensor::msg::typesupport_fastrtps_cpp::handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ultrasonic_sensor, msg, UltrasonicMsg)() {
  return &ultrasonic_sensor::msg::typesupport_fastrtps_cpp::handle;
}

#ifdef __cplusplus
}
#endif
