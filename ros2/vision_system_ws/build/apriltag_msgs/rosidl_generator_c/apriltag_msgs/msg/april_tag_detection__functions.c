// generated from rosidl_generator_c/resource/msg__functions.c.em
// generated code does not contain a copyright notice

#include "apriltag_msgs/msg/april_tag_detection__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

// include message dependencies
// family
#include "rosidl_generator_c/string_functions.h"
// centre
// corners
#include "apriltag_msgs/msg/point__functions.h"

bool
apriltag_msgs__msg__AprilTagDetection__init(apriltag_msgs__msg__AprilTagDetection * msg)
{
  if (!msg) {
    return false;
  }
  // family
  if (!rosidl_generator_c__String__init(&msg->family)) {
    apriltag_msgs__msg__AprilTagDetection__fini(msg);
    return false;
  }
  // id
  // hamming
  // goodness
  // decision_margin
  // centre
  if (!apriltag_msgs__msg__Point__init(&msg->centre)) {
    apriltag_msgs__msg__AprilTagDetection__fini(msg);
    return false;
  }
  // corners
  for (size_t i = 0; i < 4; ++i) {
    if (!apriltag_msgs__msg__Point__init(&msg->corners[i])) {
      apriltag_msgs__msg__AprilTagDetection__fini(msg);
      return false;
    }
  }
  // homography
  return true;
}

void
apriltag_msgs__msg__AprilTagDetection__fini(apriltag_msgs__msg__AprilTagDetection * msg)
{
  if (!msg) {
    return;
  }
  // family
  rosidl_generator_c__String__fini(&msg->family);
  // id
  // hamming
  // goodness
  // decision_margin
  // centre
  apriltag_msgs__msg__Point__fini(&msg->centre);
  // corners
  for (size_t i = 0; i < 4; ++i) {
    apriltag_msgs__msg__Point__fini(&msg->corners[i]);
  }
  // homography
}

apriltag_msgs__msg__AprilTagDetection *
apriltag_msgs__msg__AprilTagDetection__create()
{
  apriltag_msgs__msg__AprilTagDetection * msg = (apriltag_msgs__msg__AprilTagDetection *)malloc(sizeof(apriltag_msgs__msg__AprilTagDetection));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(apriltag_msgs__msg__AprilTagDetection));
  bool success = apriltag_msgs__msg__AprilTagDetection__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
apriltag_msgs__msg__AprilTagDetection__destroy(apriltag_msgs__msg__AprilTagDetection * msg)
{
  if (msg) {
    apriltag_msgs__msg__AprilTagDetection__fini(msg);
  }
  free(msg);
}


bool
apriltag_msgs__msg__AprilTagDetection__Sequence__init(apriltag_msgs__msg__AprilTagDetection__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  apriltag_msgs__msg__AprilTagDetection * data = NULL;
  if (size) {
    data = (apriltag_msgs__msg__AprilTagDetection *)calloc(size, sizeof(apriltag_msgs__msg__AprilTagDetection));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = apriltag_msgs__msg__AprilTagDetection__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        apriltag_msgs__msg__AprilTagDetection__fini(&data[i - 1]);
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
apriltag_msgs__msg__AprilTagDetection__Sequence__fini(apriltag_msgs__msg__AprilTagDetection__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      apriltag_msgs__msg__AprilTagDetection__fini(&array->data[i]);
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

apriltag_msgs__msg__AprilTagDetection__Sequence *
apriltag_msgs__msg__AprilTagDetection__Sequence__create(size_t size)
{
  apriltag_msgs__msg__AprilTagDetection__Sequence * array = (apriltag_msgs__msg__AprilTagDetection__Sequence *)malloc(sizeof(apriltag_msgs__msg__AprilTagDetection__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = apriltag_msgs__msg__AprilTagDetection__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
apriltag_msgs__msg__AprilTagDetection__Sequence__destroy(apriltag_msgs__msg__AprilTagDetection__Sequence * array)
{
  if (array) {
    apriltag_msgs__msg__AprilTagDetection__Sequence__fini(array);
  }
  free(array);
}
