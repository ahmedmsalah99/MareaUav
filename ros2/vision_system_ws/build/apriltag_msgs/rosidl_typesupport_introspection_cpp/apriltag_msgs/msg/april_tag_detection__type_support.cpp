// generated from rosidl_typesupport_introspection_cpp/resource/msg__type_support.cpp.em
// generated code does not contain a copyright notice

#include <array>
// providing offsetof()
#include <cstddef>
#include <string>
#include <vector>

#include "rosidl_generator_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"

#include "apriltag_msgs/msg/april_tag_detection__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace apriltag_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

size_t size_function__AprilTagDetection__corners(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__AprilTagDetection__corners(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<apriltag_msgs::msg::Point, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__AprilTagDetection__corners(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<apriltag_msgs::msg::Point, 4> *>(untyped_member);
  return &member[index];
}

size_t size_function__AprilTagDetection__homography(const void * untyped_member)
{
  (void)untyped_member;
  return 9;
}

const void * get_const_function__AprilTagDetection__homography(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 9> *>(untyped_member);
  return &member[index];
}

void * get_function__AprilTagDetection__homography(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 9> *>(untyped_member);
  return &member[index];
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember AprilTagDetection_message_member_array[8] = {
  {
    "family",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(apriltag_msgs::msg::AprilTagDetection, family),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(apriltag_msgs::msg::AprilTagDetection, id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "hamming",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(apriltag_msgs::msg::AprilTagDetection, hamming),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "goodness",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(apriltag_msgs::msg::AprilTagDetection, goodness),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "decision_margin",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(apriltag_msgs::msg::AprilTagDetection, decision_margin),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "centre",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<apriltag_msgs::msg::Point>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(apriltag_msgs::msg::AprilTagDetection, centre),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "corners",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<apriltag_msgs::msg::Point>(),  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(apriltag_msgs::msg::AprilTagDetection, corners),  // bytes offset in struct
    nullptr,  // default value
    size_function__AprilTagDetection__corners,  // size() function pointer
    get_const_function__AprilTagDetection__corners,  // get_const(index) function pointer
    get_function__AprilTagDetection__corners,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "homography",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT64,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    9,  // array size
    false,  // is upper bound
    offsetof(apriltag_msgs::msg::AprilTagDetection, homography),  // bytes offset in struct
    nullptr,  // default value
    size_function__AprilTagDetection__homography,  // size() function pointer
    get_const_function__AprilTagDetection__homography,  // get_const(index) function pointer
    get_function__AprilTagDetection__homography,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers AprilTagDetection_message_members = {
  "apriltag_msgs",  // package name
  "AprilTagDetection",  // message name
  8,  // number of fields
  sizeof(apriltag_msgs::msg::AprilTagDetection),
  AprilTagDetection_message_member_array  // message members
};

static const rosidl_message_type_support_t AprilTagDetection_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &AprilTagDetection_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace apriltag_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<apriltag_msgs::msg::AprilTagDetection>()
{
  return &::apriltag_msgs::msg::rosidl_typesupport_introspection_cpp::AprilTagDetection_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, apriltag_msgs, msg, AprilTagDetection)() {
  return &::apriltag_msgs::msg::rosidl_typesupport_introspection_cpp::AprilTagDetection_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
