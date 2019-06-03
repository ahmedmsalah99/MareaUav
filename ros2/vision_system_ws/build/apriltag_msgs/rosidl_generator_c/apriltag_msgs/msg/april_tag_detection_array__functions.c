// generated from rosidl_generator_c/resource/msg__functions.c.em
// generated code does not contain a copyright notice

#include "apriltag_msgs/msg/april_tag_detection_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

// include message dependencies
// header
#include "std_msgs/msg/header__functions.h"
// detections
#include "apriltag_msgs/msg/april_tag_detection__functions.h"

bool
apriltag_msgs__msg__AprilTagDetectionArray__init(apriltag_msgs__msg__AprilTagDetectionArray * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    apriltag_msgs__msg__AprilTagDetectionArray__fini(msg);
    return false;
  }
  // detections
  if (!apriltag_msgs__msg__AprilTagDetection__Sequence__init(&msg->detections, 0)) {
    apriltag_msgs__msg__AprilTagDetectionArray__fini(msg);
    return false;
  }
  return true;
}

void
apriltag_msgs__msg__AprilTagDetectionArray__fini(apriltag_msgs__msg__AprilTagDetectionArray * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // detections
  apriltag_msgs__msg__AprilTagDetection__Sequence__fini(&msg->detections);
}

apriltag_msgs__msg__AprilTagDetectionArray *
apriltag_msgs__msg__AprilTagDetectionArray__create()
{
  apriltag_msgs__msg__AprilTagDetectionArray * msg = (apriltag_msgs__msg__AprilTagDetectionArray *)malloc(sizeof(apriltag_msgs__msg__AprilTagDetectionArray));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(apriltag_msgs__msg__AprilTagDetectionArray));
  bool success = apriltag_msgs__msg__AprilTagDetectionArray__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
apriltag_msgs__msg__AprilTagDetectionArray__destroy(apriltag_msgs__msg__AprilTagDetectionArray * msg)
{
  if (msg) {
    apriltag_msgs__msg__AprilTagDetectionArray__fini(msg);
  }
  free(msg);
}


bool
apriltag_msgs__msg__AprilTagDetectionArray__Sequence__init(apriltag_msgs__msg__AprilTagDetectionArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  apriltag_msgs__msg__AprilTagDetectionArray * data = NULL;
  if (size) {
    data = (apriltag_msgs__msg__AprilTagDetectionArray *)calloc(size, sizeof(apriltag_msgs__msg__AprilTagDetectionArray));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = apriltag_msgs__msg__AprilTagDetectionArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        apriltag_msgs__msg__AprilTagDetectionArray__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
apriltag_msgs__msg__AprilTagDetectionArray__Sequence__fini(apriltag_msgs__msg__AprilTagDetectionArray__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      apriltag_msgs__msg__AprilTagDetectionArray__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

apriltag_msgs__msg__AprilTagDetectionArray__Sequence *
apriltag_msgs__msg__AprilTagDetectionArray__Sequence__create(size_t size)
{
  apriltag_msgs__msg__AprilTagDetectionArray__Sequence * array = (apriltag_msgs__msg__AprilTagDetectionArray__Sequence *)malloc(sizeof(apriltag_msgs__msg__AprilTagDetectionArray__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = apriltag_msgs__msg__AprilTagDetectionArray__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
apriltag_msgs__msg__AprilTagDetectionArray__Sequence__destroy(apriltag_msgs__msg__AprilTagDetectionArray__Sequence * array)
{
  if (array) {
    apriltag_msgs__msg__AprilTagDetectionArray__Sequence__fini(array);
  }
  free(array);
}
