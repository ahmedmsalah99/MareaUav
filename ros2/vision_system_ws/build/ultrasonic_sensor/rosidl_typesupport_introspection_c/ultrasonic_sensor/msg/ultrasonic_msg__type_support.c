// generated from rosidl_typesupport_introspection_c/resource/msg__type_support.c.em
// generated code does not contain a copyright notice


// providing offsetof()
#include <stddef.h>

#include <ultrasonic_sensor/msg/ultrasonic_msg__rosidl_typesupport_introspection_c.h>
#include "ultrasonic_sensor/msg/rosidl_typesupport_introspection_c__visibility_control.h"

#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"

#include "ultrasonic_sensor/msg/ultrasonic_msg__struct.h"

#ifdef __cplusplus
extern "C"
{
#endif

static rosidl_typesupport_introspection_c__MessageMember ultrasonic_sensor__msg__rosidl_typesupport_introspection_c__UltrasonicMsg_message_member_array[2] = {
  {
    "z_position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ultrasonic_sensor__msg__UltrasonicMsg, z_position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "z_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ultrasonic_sensor__msg__UltrasonicMsg, z_velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ultrasonic_sensor__msg__rosidl_typesupport_introspection_c__UltrasonicMsg_message_members = {
  "ultrasonic_sensor",  // package name
  "UltrasonicMsg",  // message name
  2,  // number of fields
  sizeof(ultrasonic_sensor__msg__UltrasonicMsg),
  ultrasonic_sensor__msg__rosidl_typesupport_introspection_c__UltrasonicMsg_message_member_array  // message members
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ultrasonic_sensor__msg__rosidl_typesupport_introspection_c__UltrasonicMsg_message_type_support_handle = {
  0,
  &ultrasonic_sensor__msg__rosidl_typesupport_introspection_c__UltrasonicMsg_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ultrasonic_sensor
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ultrasonic_sensor, msg, UltrasonicMsg)() {
  if (!ultrasonic_sensor__msg__rosidl_typesupport_introspection_c__UltrasonicMsg_message_type_support_handle.typesupport_identifier) {
    ultrasonic_sensor__msg__rosidl_typesupport_introspection_c__UltrasonicMsg_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ultrasonic_sensor__msg__rosidl_typesupport_introspection_c__UltrasonicMsg_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
