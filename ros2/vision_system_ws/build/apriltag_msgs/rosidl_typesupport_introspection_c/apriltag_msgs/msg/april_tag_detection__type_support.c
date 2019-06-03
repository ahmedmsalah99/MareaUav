// generated from rosidl_typesupport_introspection_c/resource/msg__type_support.c.em
// generated code does not contain a copyright notice


// providing offsetof()
#include <stddef.h>

#include <apriltag_msgs/msg/april_tag_detection__rosidl_typesupport_introspection_c.h>
#include "apriltag_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"

#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"

#include "apriltag_msgs/msg/april_tag_detection__struct.h"

// include message dependencies
// centre
// corners
#include "apriltag_msgs/msg/point.h"
// centre
// corners
#include "apriltag_msgs/msg/point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

size_t apriltag_msgs__msg__rosidl_typesupport_introspection_c__size_function__AprilTagDetection__corners(
  const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * apriltag_msgs__msg__rosidl_typesupport_introspection_c__get_const_function__AprilTagDetection__corners(
  const void * untyped_member, size_t index)
{
  const apriltag_msgs__msg__Point ** member =
    (const apriltag_msgs__msg__Point **)(untyped_member);
  return &(*member)[index];
}

void * apriltag_msgs__msg__rosidl_typesupport_introspection_c__get_function__AprilTagDetection__corners(
  void * untyped_member, size_t index)
{
  apriltag_msgs__msg__Point ** member =
    (apriltag_msgs__msg__Point **)(untyped_member);
  return &(*member)[index];
}

static rosidl_typesupport_introspection_c__MessageMember apriltag_msgs__msg__rosidl_typesupport_introspection_c__AprilTagDetection_message_member_array[8] = {
  {
    "family",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(apriltag_msgs__msg__AprilTagDetection, family),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(apriltag_msgs__msg__AprilTagDetection, id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "hamming",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(apriltag_msgs__msg__AprilTagDetection, hamming),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "goodness",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(apriltag_msgs__msg__AprilTagDetection, goodness),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "decision_margin",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(apriltag_msgs__msg__AprilTagDetection, decision_margin),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "centre",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(apriltag_msgs__msg__AprilTagDetection, centre),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "corners",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(apriltag_msgs__msg__AprilTagDetection, corners),  // bytes offset in struct
    NULL,  // default value
    apriltag_msgs__msg__rosidl_typesupport_introspection_c__size_function__AprilTagDetection__corners,  // size() function pointer
    apriltag_msgs__msg__rosidl_typesupport_introspection_c__get_const_function__AprilTagDetection__corners,  // get_const(index) function pointer
    apriltag_msgs__msg__rosidl_typesupport_introspection_c__get_function__AprilTagDetection__corners,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "homography",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    9,  // array size
    false,  // is upper bound
    offsetof(apriltag_msgs__msg__AprilTagDetection, homography),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers apriltag_msgs__msg__rosidl_typesupport_introspection_c__AprilTagDetection_message_members = {
  "apriltag_msgs",  // package name
  "AprilTagDetection",  // message name
  8,  // number of fields
  sizeof(apriltag_msgs__msg__AprilTagDetection),
  apriltag_msgs__msg__rosidl_typesupport_introspection_c__AprilTagDetection_message_member_array  // message members
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t apriltag_msgs__msg__rosidl_typesupport_introspection_c__AprilTagDetection_message_type_support_handle = {
  0,
  &apriltag_msgs__msg__rosidl_typesupport_introspection_c__AprilTagDetection_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_apriltag_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, apriltag_msgs, msg, AprilTagDetection)() {
  apriltag_msgs__msg__rosidl_typesupport_introspection_c__AprilTagDetection_message_member_array[5].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, apriltag_msgs, msg, Point)();
  apriltag_msgs__msg__rosidl_typesupport_introspection_c__AprilTagDetection_message_member_array[6].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, apriltag_msgs, msg, Point)();
  if (!apriltag_msgs__msg__rosidl_typesupport_introspection_c__AprilTagDetection_message_type_support_handle.typesupport_identifier) {
    apriltag_msgs__msg__rosidl_typesupport_introspection_c__AprilTagDetection_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &apriltag_msgs__msg__rosidl_typesupport_introspection_c__AprilTagDetection_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
