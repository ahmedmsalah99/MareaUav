// generated from rosidl_generator_c/resource/msg__struct.h.em
// generated code does not contain a copyright notice

#ifndef APRILTAG_MSGS__MSG__APRIL_TAG_DETECTION_ARRAY__STRUCT_H_
#define APRILTAG_MSGS__MSG__APRIL_TAG_DETECTION_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// include message dependencies
// header
#include "std_msgs/msg/header__struct.h"
// detections
#include "apriltag_msgs/msg/april_tag_detection__struct.h"

/// Struct of message apriltag_msgs/AprilTagDetectionArray
typedef struct apriltag_msgs__msg__AprilTagDetectionArray
{
  std_msgs__msg__Header header;
  apriltag_msgs__msg__AprilTagDetection__Sequence detections;
} apriltag_msgs__msg__AprilTagDetectionArray;

/// Struct for an array of messages
typedef struct apriltag_msgs__msg__AprilTagDetectionArray__Sequence
{
  apriltag_msgs__msg__AprilTagDetectionArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} apriltag_msgs__msg__AprilTagDetectionArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // APRILTAG_MSGS__MSG__APRIL_TAG_DETECTION_ARRAY__STRUCT_H_
