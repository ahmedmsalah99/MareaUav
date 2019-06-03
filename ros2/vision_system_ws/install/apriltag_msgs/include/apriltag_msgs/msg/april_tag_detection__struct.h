// generated from rosidl_generator_c/resource/msg__struct.h.em
// generated code does not contain a copyright notice

#ifndef APRILTAG_MSGS__MSG__APRIL_TAG_DETECTION__STRUCT_H_
#define APRILTAG_MSGS__MSG__APRIL_TAG_DETECTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// include message dependencies
// family
#include "rosidl_generator_c/string.h"
// centre
// corners
#include "apriltag_msgs/msg/point__struct.h"

/// Struct of message apriltag_msgs/AprilTagDetection
typedef struct apriltag_msgs__msg__AprilTagDetection
{
  rosidl_generator_c__String family;
  int32_t id;
  int32_t hamming;
  float goodness;
  float decision_margin;
  apriltag_msgs__msg__Point centre;
  apriltag_msgs__msg__Point corners[4];
  double homography[9];
} apriltag_msgs__msg__AprilTagDetection;

/// Struct for an array of messages
typedef struct apriltag_msgs__msg__AprilTagDetection__Sequence
{
  apriltag_msgs__msg__AprilTagDetection * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} apriltag_msgs__msg__AprilTagDetection__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // APRILTAG_MSGS__MSG__APRIL_TAG_DETECTION__STRUCT_H_
