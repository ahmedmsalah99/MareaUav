// generated from rosidl_typesupport_fastrtps_c/resource/msg__type_support_c.cpp.em
// generated code does not contain a copyright notice

#include "apriltag_msgs/msg/april_tag_detection_array__rosidl_typesupport_fastrtps_c.h"

#include <cassert>
#include <limits>
#include <string>

// Provides the rosidl_typesupport_fastrtps_c__identifier symbol declaration.
#include "rosidl_typesupport_fastrtps_c/identifier.h"
// Provides the definition of the message_type_support_callbacks_t struct.
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"

#include "apriltag_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "apriltag_msgs/msg/april_tag_detection_array__struct.h"
#include "apriltag_msgs/msg/april_tag_detection_array__functions.h"

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

#include "apriltag_msgs/msg/april_tag_detection__functions.h"  // detections
#include "std_msgs/msg/header__functions.h"  // header

// forward declare type support functions
size_t get_serialized_size_apriltag_msgs__msg__AprilTagDetection(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_apriltag_msgs__msg__AprilTagDetection(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, apriltag_msgs, msg, AprilTagDetection)();
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_apriltag_msgs
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_apriltag_msgs
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_apriltag_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();


using __ros_msg_type = apriltag_msgs__msg__AprilTagDetectionArray;

static bool __cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const __ros_msg_type * ros_message = static_cast<const __ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * std_msgs__msg__Header__callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!std_msgs__msg__Header__callbacks->cdr_serialize(
        &ros_message->header, cdr))
    {
      return false;
    }
  }

  // Field name: detections
  {
    const message_type_support_callbacks_t * apriltag_msgs__msg__AprilTagDetection__callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, apriltag_msgs, msg, AprilTagDetection
      )()->data);
    size_t size = ros_message->detections.size;
    auto array_ptr = ros_message->detections.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      if (!apriltag_msgs__msg__AprilTagDetection__callbacks->cdr_serialize(
          &array_ptr[i], cdr))
      {
        return false;
      }
    }
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
  // Field name: header
  {
    const message_type_support_callbacks_t * std_msgs__msg__Header__callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!std_msgs__msg__Header__callbacks->cdr_deserialize(
        cdr, &ros_message->header))
    {
      return false;
    }
  }

  // Field name: detections
  {
    const message_type_support_callbacks_t * apriltag_msgs__msg__AprilTagDetection__callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, apriltag_msgs, msg, AprilTagDetection
      )()->data);
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->detections.data) {
      apriltag_msgs__msg__AprilTagDetection__Sequence__fini(&ros_message->detections);
    }
    if (!apriltag_msgs__msg__AprilTagDetection__Sequence__init(&ros_message->detections, size)) {
      return "failed to create array for field 'detections'";
    }
    auto array_ptr = ros_message->detections.data;
    for (size_t i = 0; i < size; ++i) {
      if (!apriltag_msgs__msg__AprilTagDetection__callbacks->cdr_deserialize(
          cdr, &array_ptr[i]))
      {
        return false;
      }
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_apriltag_msgs
size_t get_serialized_size_apriltag_msgs__msg__AprilTagDetectionArray(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const __ros_msg_type * ros_message = static_cast<const __ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  (void)padding;

  // field.name header

  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);
  // field.name detections
  {
    size_t array_size = ros_message->detections.size;
    auto array_ptr = ros_message->detections.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_apriltag_msgs__msg__AprilTagDetection(
        &array_ptr[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static uint32_t __get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_apriltag_msgs__msg__AprilTagDetectionArray(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_apriltag_msgs
size_t max_serialized_size_apriltag_msgs__msg__AprilTagDetectionArray(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  (void)padding;
  (void)full_bounded;

  // field.name header
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_std_msgs__msg__Header(
        full_bounded, current_alignment);
    }
  }
  // field.name detections
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_apriltag_msgs__msg__AprilTagDetection(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static size_t __max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_apriltag_msgs__msg__AprilTagDetectionArray(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks = {
  "apriltag_msgs",
  "AprilTagDetectionArray",
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
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, apriltag_msgs, msg, AprilTagDetectionArray)() {
  return &__type_support;
}

#if defined(__cplusplus)
}
#endif
