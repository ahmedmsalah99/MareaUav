// generated from rosidl_typesupport_introspection_c/resource/msg__type_support.c.em
// generated code does not contain a copyright notice


// providing offsetof()
#include <stddef.h>

#include <apriltag_msgs/msg/point__rosidl_typesupport_introspection_c.h>
#include "apriltag_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"

#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"

#include "apriltag_msgs/msg/point__struct.h"

#ifdef __cplusplus
extern "C"
{
#endif

static rosidl_typesupport_introspection_c__MessageMember apriltag_msgs__msg__rosidl_typesupport_introspection_c__Point_message_member_array[2] = {
  {
    "x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(apriltag_msgs__msg__Point, x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(apriltag_msgs__msg__Point, y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers apriltag_msgs__msg__rosidl_typesupport_introspection_c__Point_message_members = {
  "apriltag_msgs",  // package name
  "Point",  // message name
  2,  // number of fields
  sizeof(apriltag_msgs__msg__Point),
  apriltag_msgs__msg__rosidl_typesupport_introspection_c__Point_message_member_array  // message members
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t apriltag_msgs__msg__rosidl_typesupport_introspection_c__Point_message_type_support_handle = {
  0,
  &apriltag_msgs__msg__rosidl_typesupport_introspection_c__Point_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_apriltag_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, apriltag_msgs, msg, Point)() {
  if (!apriltag_msgs__msg__rosidl_typesupport_introspection_c__Point_message_type_support_handle.typesupport_identifier) {
    apriltag_msgs__msg__rosidl_typesupport_introspection_c__Point_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &apriltag_msgs__msg__rosidl_typesupport_introspection_c__Point_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
