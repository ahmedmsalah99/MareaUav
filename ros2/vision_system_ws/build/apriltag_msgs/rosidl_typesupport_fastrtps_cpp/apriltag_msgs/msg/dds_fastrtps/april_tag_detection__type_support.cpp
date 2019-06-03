// generated from rosidl_typesupport_fastrtps_cpp/resource/msg__type_support.cpp.em
// generated code does not contain a copyright notice

#include "apriltag_msgs/msg/april_tag_detection__rosidl_typesupport_fastrtps_cpp.hpp"

#include <limits>
#include <stdexcept>
#include <string>

#include "rosidl_typesupport_cpp/message_type_support.hpp"

#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"

#include "fastcdr/Cdr.h"

// forward declaration of message dependencies and their conversion functions
namespace apriltag_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const apriltag_msgs::msg::Point &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  apriltag_msgs::msg::Point &);
size_t get_serialized_size(
  const apriltag_msgs::msg::Point &,
  size_t current_alignment);
size_t
max_serialized_size_Point(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace apriltag_msgs

namespace apriltag_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const apriltag_msgs::msg::Point &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  apriltag_msgs::msg::Point &);
size_t get_serialized_size(
  const apriltag_msgs::msg::Point &,
  size_t current_alignment);
size_t
max_serialized_size_Point(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace apriltag_msgs


namespace apriltag_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_apriltag_msgs
cdr_serialize(
  const apriltag_msgs::msg::AprilTagDetection & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // field.name family
  cdr << ros_message.family;
  // field.name id
  cdr << ros_message.id;
  // field.name hamming
  cdr << ros_message.hamming;
  // field.name goodness
  cdr << ros_message.goodness;
  // field.name decision_margin
  cdr << ros_message.decision_margin;
  // field.name centre
  apriltag_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.centre,
    cdr);
  // field.name corners
  {
    for (size_t i = 0; i < 4; i++) {
      apriltag_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
        ros_message.corners[i],
        cdr);
    }
  }
  // field.name homography
  {
    cdr << ros_message.homography;
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_apriltag_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  apriltag_msgs::msg::AprilTagDetection & ros_message)
{
  // field.name family
  cdr >> ros_message.family;

  // field.name id
  cdr >> ros_message.id;

  // field.name hamming
  cdr >> ros_message.hamming;

  // field.name goodness
  cdr >> ros_message.goodness;

  // field.name decision_margin
  cdr >> ros_message.decision_margin;

  // field.name centre
  apriltag_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.centre);

  // field.name corners
  {
    for (size_t i = 0; i < 4; i++) {
      apriltag_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr,
        ros_message.corners[i]);
    }
  }

  // field.name homography
  {
    cdr >> ros_message.homography;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_apriltag_msgs
get_serialized_size(
  const apriltag_msgs::msg::AprilTagDetection & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  (void)padding;

  // field.name family
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    ros_message.family.size() + 1;
  // field.name id
  {
    size_t item_size = sizeof(ros_message.id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name hamming
  {
    size_t item_size = sizeof(ros_message.hamming);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name goodness
  {
    size_t item_size = sizeof(ros_message.goodness);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name decision_margin
  {
    size_t item_size = sizeof(ros_message.decision_margin);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name centre

  current_alignment +=
    apriltag_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.centre, current_alignment);
  // field.name corners
  {
    size_t array_size = 4;

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        apriltag_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
        ros_message.corners[index], current_alignment);
    }
  }
  // field.name homography
  {
    size_t array_size = 9;
    size_t item_size = sizeof(ros_message.homography[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_apriltag_msgs
max_serialized_size_AprilTagDetection(
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
        apriltag_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Point(
        full_bounded, current_alignment);
    }
  }
  // field.name corners
  {
    size_t array_size = 4;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        apriltag_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Point(
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

static bool __cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const apriltag_msgs::msg::AprilTagDetection *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool __cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<apriltag_msgs::msg::AprilTagDetection *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t __get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const apriltag_msgs::msg::AprilTagDetection *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t __max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_AprilTagDetection(full_bounded, 0);
}

static message_type_support_callbacks_t callbacks = {
  "apriltag_msgs",
  "AprilTagDetection",
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

}  // namespace apriltag_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_apriltag_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<apriltag_msgs::msg::AprilTagDetection>()
{
  return &apriltag_msgs::msg::typesupport_fastrtps_cpp::handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, apriltag_msgs, msg, AprilTagDetection)() {
  return &apriltag_msgs::msg::typesupport_fastrtps_cpp::handle;
}

#ifdef __cplusplus
}
#endif
