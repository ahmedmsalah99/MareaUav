// generated from rosidl_generator_c/resource/msg__struct.h.em
// generated code does not contain a copyright notice

#ifndef ULTRASONIC_SENSOR__MSG__ULTRASONIC_MSG__STRUCT_H_
#define ULTRASONIC_SENSOR__MSG__ULTRASONIC_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/// Struct of message ultrasonic_sensor/UltrasonicMsg
typedef struct ultrasonic_sensor__msg__UltrasonicMsg
{
  double z_position;
  double z_velocity;
} ultrasonic_sensor__msg__UltrasonicMsg;

/// Struct for an array of messages
typedef struct ultrasonic_sensor__msg__UltrasonicMsg__Sequence
{
  ultrasonic_sensor__msg__UltrasonicMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ultrasonic_sensor__msg__UltrasonicMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ULTRASONIC_SENSOR__MSG__ULTRASONIC_MSG__STRUCT_H_
