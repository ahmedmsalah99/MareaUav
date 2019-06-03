// generated from rosidl_generator_c/resource/msg__functions.c.em
// generated code does not contain a copyright notice

#include "ultrasonic_sensor/msg/ultrasonic_msg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

bool
ultrasonic_sensor__msg__UltrasonicMsg__init(ultrasonic_sensor__msg__UltrasonicMsg * msg)
{
  if (!msg) {
    return false;
  }
  // z_position
  // z_velocity
  return true;
}

void
ultrasonic_sensor__msg__UltrasonicMsg__fini(ultrasonic_sensor__msg__UltrasonicMsg * msg)
{
  if (!msg) {
    return;
  }
  // z_position
  // z_velocity
}

ultrasonic_sensor__msg__UltrasonicMsg *
ultrasonic_sensor__msg__UltrasonicMsg__create()
{
  ultrasonic_sensor__msg__UltrasonicMsg * msg = (ultrasonic_sensor__msg__UltrasonicMsg *)malloc(sizeof(ultrasonic_sensor__msg__UltrasonicMsg));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ultrasonic_sensor__msg__UltrasonicMsg));
  bool success = ultrasonic_sensor__msg__UltrasonicMsg__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
ultrasonic_sensor__msg__UltrasonicMsg__destroy(ultrasonic_sensor__msg__UltrasonicMsg * msg)
{
  if (msg) {
    ultrasonic_sensor__msg__UltrasonicMsg__fini(msg);
  }
  free(msg);
}


bool
ultrasonic_sensor__msg__UltrasonicMsg__Sequence__init(ultrasonic_sensor__msg__UltrasonicMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  ultrasonic_sensor__msg__UltrasonicMsg * data = NULL;
  if (size) {
    data = (ultrasonic_sensor__msg__UltrasonicMsg *)calloc(size, sizeof(ultrasonic_sensor__msg__UltrasonicMsg));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ultrasonic_sensor__msg__UltrasonicMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ultrasonic_sensor__msg__UltrasonicMsg__fini(&data[i - 1]);
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
ultrasonic_sensor__msg__UltrasonicMsg__Sequence__fini(ultrasonic_sensor__msg__UltrasonicMsg__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      ultrasonic_sensor__msg__UltrasonicMsg__fini(&array->data[i]);
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

ultrasonic_sensor__msg__UltrasonicMsg__Sequence *
ultrasonic_sensor__msg__UltrasonicMsg__Sequence__create(size_t size)
{
  ultrasonic_sensor__msg__UltrasonicMsg__Sequence * array = (ultrasonic_sensor__msg__UltrasonicMsg__Sequence *)malloc(sizeof(ultrasonic_sensor__msg__UltrasonicMsg__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = ultrasonic_sensor__msg__UltrasonicMsg__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
ultrasonic_sensor__msg__UltrasonicMsg__Sequence__destroy(ultrasonic_sensor__msg__UltrasonicMsg__Sequence * array)
{
  if (array) {
    ultrasonic_sensor__msg__UltrasonicMsg__Sequence__fini(array);
  }
  free(array);
}
