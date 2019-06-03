// generated from rosidl_typesupport_fastrtps_c/resource/msg__type_support_c.cpp.em
// generated code does not contain a copyright notice

#include "apriltag_msgs/msg/april_tag_detection__rosidl_typesupport_fastrtps_c.h"

#include <cassert>
#include <limits>
#include <string>

// Provides the rosidl_typesupport_fastrtps_c__identifier symbol declaration.
#include "rosidl_typesupport_fastrtps_c/identifier.h"
// Provides the definition of the message_type_support_callbacks_t struct.
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"

#include "apriltag_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "apriltag_msgs/msg/april_tag_detection__struct.h"
#include "apriltag_msgs/msg/april_tag_detection__functions.h"

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

#include "apriltag_msgs/msg/point__functions.h"  // centre, corners
#include "rosidl_generator_c/primitives_sequence.h"  // homography
#include "rosidl_generator_c/primitives_sequence_functions.h"  // homography
#include "rosidl_generator_c/string.h"  // family
#include "rosidl_generator_c/string_functions.h"  // family

// forward declare type support functions
size_t get_serialized_size_apriltag_msgs__msg__Point(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_apriltag_msgs__msg__Point(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, apriltag_msgs, msg, Point)();


using __ros_msg_type = apriltag_msgs__msg__AprilTagDetection;

static bool __cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const __ros_msg_type * ros_message = static_cast<const __ros_msg_type *>(untyped_ros_message);
  // Field name: family
  {
    const rosidl_generator_c__String * str = &ros_message->family;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: id
  {
    cdr << ros_message->id;
  }

  // Field name: hamming
  {
    cdr << ros_message->hamming;
  }

  // Field name: goodness
  {
    cdr << ros_message->goodness;
  }

  // Field name: decision_margin
  {
    cdr << ros_message->decision_margin;
  }

  // Field name: centre
  {
    const message_type_support_callbacks_t * apriltag_msgs__msg__Point__callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, apriltag_msgs, msg, Point
      )()->data);
    if (!apriltag_msgs__msg__Point__callbacks->cdr_serialize(
        &ros_message->centre, cdr))
    {
      return false;
    }
  }

  // Field name: corners
  {
    const message_type_support_callbacks_t * apriltag_msgs__msg__Point__callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, apriltag_msgs, msg, Point
      )()->data);
    size_t size = 4;
    auto array_ptr = ros_message->corners;
    for (size_t i = 0; i < size; ++i) {
      if (!apriltag_msgs__msg__Point__callbacks->cdr_serialize(
          &array_ptr[i], cdr))
      {
        return false;
      }
    }
  }

  // Field name: homography
  {
    size_t size = 9;
    auto array_ptr = ros_message->homography;
    cdr.serializeArray(array_ptr, size);
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
  // Field name: family
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->family.data) {
      rosidl_generator_c__String__init(&ros_message->family);
    }
    bool succeeded = rosidl_generator_c__String__assign(
      &ros_message->family,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'family'\n");
      return false;
    }
  }

  // Field name: id
  {
    cdr >> ros_message->id;
  }

  // Field name: hamming
  {
    cdr >> ros_message->hamming;
  }

  // Field name: goodness
  {
    cdr >> ros_message->goodness;
  }

  // Field name: decision_margin
  {
    cdr >> ros_message->decision_margin;
  }

  // Field name: centre
  {
    const message_type_support_callbacks_t * apriltag_msgs__msg__Point__callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, apriltag_msgs, msg, Point
      )()->data);
    if (!apriltag_msgs__msg__Point__callbacks->cdr_deserialize(
        cdr, &ros_message->centre))
    {
      return false;
    }
  }

  // Field name: corners
  {
    const message_type_support_callbacks_t * apriltag_msgs__msg__Point__callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, apriltag_msgs, msg, Point
      )()->data);
    size_t size = 4;
    auto array_ptr = ros_message->corners;
    for (size_t i = 0; i < size; ++i) {
      if (!apriltag_msgs__msg__Point__callbacks->cdr_deserialize(
          cdr, &array_ptr[i]))
      {
        return false;
      }
    }
  }

  // Field name: homography
  {
    size_t size = 9;
    auto array_ptr = ros_message->homography;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_apriltag_msgs
size_t get_serialized_size_apriltag_msgs__msg__AprilTagDetection(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const __ros_msg_type * ros_message = static_cast<const __ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  (void)padding;

  // field.name family
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    ros_message->family.size + 1;
  // field.name id
  {
    size_t item_size = sizeof(ros_message->id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name hamming
  {
    size_t item_size = sizeof(ros_message->hamming);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name goodness
  {
    size_t item_size = sizeof(ros_message->goodness);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name decision_margin
  {
    size_t item_size = sizeof(ros_message->decision_margin);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name centre

  current_alignment += get_serialized_size_apriltag_msgs__msg__Point(
    &(ros_message->centre), current_alignment);
  // field.name corners
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->corners;

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_apriltag_msgs__msg__Point(
        &array_ptr[index], current_alignment);
    }
  }
  // field.name homography
  {
    size_t array_size = 9;
    auto array_ptr = ros_message->homography;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t __get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_apriltag_msgs__msg__AprilTagDetection(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_apriltag_msgs
size_t max_serialized_size_apriltag_msgs__msg__AprilTagDetection(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  (void)padding;
  (void)full_bounded;

  // field.name family
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) + 1;
    }
  }
  // field.name id
  {
    size_t array_size = 1;


    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // field.name hamming
  {
    size_t array_size = 1;


    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // field.name goodness
  {
    size_t array_size = 1;


    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // field.name decision_margin
  {
    size_t array_size = 1;


    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // field.name centre
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_apriltag_msgs__msg__Point(
        full_bounded, current_alignment);
    }
  }
  // field.name corners
  {
    size_t array_size = 4;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_apriltag_msgs__msg__Point(
        full_bounded, current_alignment);
    }
  }
  // field.name homography
  {
    size_t array_size = 9;


    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  return current_alignment - initial_alignment;
}

static size_t __max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_apriltag_msgs__msg__AprilTagDetection(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks = {
  "apriltag_msgs",
  "AprilTagDetection",
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
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, apriltag_msgs, msg, AprilTagDetection)() {
  return &__type_support;
}

#if defined(__cplusplus)
}
#endif
