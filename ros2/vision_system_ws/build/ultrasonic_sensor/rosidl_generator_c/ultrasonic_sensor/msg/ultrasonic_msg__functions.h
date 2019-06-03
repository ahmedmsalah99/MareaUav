// generated from rosidl_generator_c/resource/msg__functions.h.em
// generated code does not contain a copyright notice

#ifndef ULTRASONIC_SENSOR__MSG__ULTRASONIC_MSG__FUNCTIONS_H_
#define ULTRASONIC_SENSOR__MSG__ULTRASONIC_MSG__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_generator_c/visibility_control.h"
#include "ultrasonic_sensor/msg/rosidl_generator_c__visibility_control.h"

#include "ultrasonic_sensor/msg/ultrasonic_msg__struct.h"

/// Initialize ultrasonic_sensor/UltrasonicMsg message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(ultrasonic_sensor__msg__UltrasonicMsg)) before
 * or use ultrasonic_sensor__msg__UltrasonicMsg__create() to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_ultrasonic_sensor
bool
ultrasonic_sensor__msg__UltrasonicMsg__init(ultrasonic_sensor__msg__UltrasonicMsg * msg);

/// Finalize ultrasonic_sensor/UltrasonicMsg message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ultrasonic_sensor
void
ultrasonic_sensor__msg__UltrasonicMsg__fini(ultrasonic_sensor__msg__UltrasonicMsg * msg);

/// Create ultrasonic_sensor/UltrasonicMsg message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls ultrasonic_sensor__msg__UltrasonicMsg__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ultrasonic_sensor
ultrasonic_sensor__msg__UltrasonicMsg *
ultrasonic_sensor__msg__UltrasonicMsg__create();

/// Destroy ultrasonic_sensor/UltrasonicMsg message.
/**
 * It calls ultrasonic_sensor__msg__UltrasonicMsg__fini() and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ultrasonic_sensor
void
ultrasonic_sensor__msg__UltrasonicMsg__destroy(ultrasonic_sensor__msg__UltrasonicMsg * msg);


/// Initialize array of ultrasonic_sensor/UltrasonicMsg messages.
/**
 * It allocates the memory for the number of elements and
 * calls ultrasonic_sensor__msg__UltrasonicMsg__init() for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_ultrasonic_sensor
bool
ultrasonic_sensor__msg__UltrasonicMsg__Sequence__init(ultrasonic_sensor__msg__UltrasonicMsg__Sequence * array, size_t size);

/// Finalize array of ultrasonic_sensor/UltrasonicMsg messages.
/**
 * It calls ultrasonic_sensor__msg__UltrasonicMsg__fini() for each element of the array and
 * frees the memory for the number of elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ultrasonic_sensor
void
ultrasonic_sensor__msg__UltrasonicMsg__Sequence__fini(ultrasonic_sensor__msg__UltrasonicMsg__Sequence * array);

/// Create array of ultrasonic_sensor/UltrasonicMsg messages.
/**
 * It allocates the memory for the array and
 * calls ultrasonic_sensor__msg__UltrasonicMsg__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_ultrasonic_sensor
ultrasonic_sensor__msg__UltrasonicMsg__Sequence *
ultrasonic_sensor__msg__UltrasonicMsg__Sequence__create(size_t size);

/// Destroy array of ultrasonic_sensor/UltrasonicMsg messages.
/**
 * It calls ultrasonic_sensor__msg__UltrasonicMsg__Sequence__fini() on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_ultrasonic_sensor
void
ultrasonic_sensor__msg__UltrasonicMsg__Sequence__destroy(ultrasonic_sensor__msg__UltrasonicMsg__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // ULTRASONIC_SENSOR__MSG__ULTRASONIC_MSG__FUNCTIONS_H_
