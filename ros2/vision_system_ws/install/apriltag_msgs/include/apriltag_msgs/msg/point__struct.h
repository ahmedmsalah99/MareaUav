// generated from rosidl_generator_c/resource/msg__struct.h.em
// generated code does not contain a copyright notice

#ifndef APRILTAG_MSGS__MSG__POINT__STRUCT_H_
#define APRILTAG_MSGS__MSG__POINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/// Struct of message apriltag_msgs/Point
typedef struct apriltag_msgs__msg__Point
{
  double x;
  double y;
} apriltag_msgs__msg__Point;

/// Struct for an array of messages
typedef struct apriltag_msgs__msg__Point__Sequence
{
  apriltag_msgs__msg__Point * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} apriltag_msgs__msg__Point__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // APRILTAG_MSGS__MSG__POINT__STRUCT_H_
