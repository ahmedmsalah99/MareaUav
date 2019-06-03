// generated from rosidl_typesupport_fastrtps_c/resource/msg__type_support_c.cpp.em
// generated code does not contain a copyright notice

#include "ultrasonic_sensor/msg/ultrasonic_msg__rosidl_typesupport_fastrtps_c.h"

#include <cassert>
#include <limits>
#include <string>

// Provides the rosidl_typesupport_fastrtps_c__identifier symbol declaration.
#include "rosidl_typesupport_fastrtps_c/identifier.h"
// Provides the definition of the message_type_support_callbacks_t struct.
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"

#include "ultrasonic_sensor/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "ultrasonic_sensor/msg/ultrasonic_msg__struct.h"
#include "ultrasonic_sensor/msg/ultrasonic_msg__functions.h"

#include "fastcdr/Cdr.h"

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

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using __ros_msg_type = ultrasonic_sensor__msg__UltrasonicMsg;

static bool __cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const __ros_msg_type * ros_message = static_cast<const __ros_msg_type *>(untyped_ros_message);
  // Field name: z_position
  {
    cdr << ros_message->z_position;
  }

  // Field name: z_velocity
  {
    cdr << ros_message->z_velocity;
  }

  return true;
}

static bool __cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  __ros_msg_type * ros_message = static_cast<__ros_msg_type *>(untyped_ros_message);
  // Field name: z_position
  {
    cdr >> ros_message->z_position;
  }

  // Field name: z_velocity
  {
    cdr >> ros_message->z_velocity;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ultrasonic_sensor
size_t get_serialized_size_ultrasonic_sensor__msg__UltrasonicMsg(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const __ros_msg_type * ros_message = static_cast<const __ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  (void)padding;

  // field.name z_position
  {
    size_t item_size = sizeof(ros_message->z_position);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name z_velocity
  {
    size_t item_size = sizeof(ros_message->z_velocity);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t __get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ultrasonic_sensor__msg__UltrasonicMsg(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ultrasonic_sensor
size_t max_serialized_size_ultrasonic_sensor__msg__UltrasonicMsg(
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

static size_t __max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_ultrasonic_sensor__msg__UltrasonicMsg(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks = {
  "ultrasonic_sensor",
  "UltrasonicMsg",
  __cdr_serialize,
  __cdr_deserialize,
  __get_serialized_size,
  __max_serialized_size
};

static rosidl_message_type_support_t __type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ultrasonic_sensor, msg, UltrasonicMsg)() {
  return &__type_support;
}

#if defined(__cplusplus)
}
#endif
