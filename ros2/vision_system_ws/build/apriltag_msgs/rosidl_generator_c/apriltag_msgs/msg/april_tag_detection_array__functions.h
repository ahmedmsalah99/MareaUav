// generated from rosidl_generator_c/resource/msg__functions.h.em
// generated code does not contain a copyright notice

#ifndef APRILTAG_MSGS__MSG__APRIL_TAG_DETECTION_ARRAY__FUNCTIONS_H_
#define APRILTAG_MSGS__MSG__APRIL_TAG_DETECTION_ARRAY__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_generator_c/visibility_control.h"
#include "apriltag_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "apriltag_msgs/msg/april_tag_detection_array__struct.h"

/// Initialize apriltag_msgs/AprilTagDetectionArray message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(apriltag_msgs__msg__AprilTagDetectionArray)) before
 * or use apriltag_msgs__msg__AprilTagDetectionArray__create() to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_apriltag_msgs
bool
apriltag_msgs__msg__AprilTagDetectionArray__init(apriltag_msgs__msg__AprilTagDetectionArray * msg);

/// Finalize apriltag_msgs/AprilTagDetectionArray message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_apriltag_msgs
void
apriltag_msgs__msg__AprilTagDetectionArray__fini(apriltag_msgs__msg__AprilTagDetectionArray * msg);

/// Create apriltag_msgs/AprilTagDetectionArray message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls apriltag_msgs__msg__AprilTagDetectionArray__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_apriltag_msgs
apriltag_msgs__msg__AprilTagDetectionArray *
apriltag_msgs__msg__AprilTagDetectionArray__create();

/// Destroy apriltag_msgs/AprilTagDetectionArray message.
/**
 * It calls apriltag_msgs__msg__AprilTagDetectionArray__fini() and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_apriltag_msgs
void
apriltag_msgs__msg__AprilTagDetectionArray__destroy(apriltag_msgs__msg__AprilTagDetectionArray * msg);


/// Initialize array of apriltag_msgs/AprilTagDetectionArray messages.
/**
 * It allocates the memory for the number of elements and
 * calls apriltag_msgs__msg__AprilTagDetectionArray__init() for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_apriltag_msgs
bool
apriltag_msgs__msg__AprilTagDetectionArray__Sequence__init(apriltag_msgs__msg__AprilTagDetectionArray__Sequence * array, size_t size);

/// Finalize array of apriltag_msgs/AprilTagDetectionArray messages.
/**
 * It calls apriltag_msgs__msg__AprilTagDetectionArray__fini() for each element of the array and
 * frees the memory for the number of elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_apriltag_msgs
void
apriltag_msgs__msg__AprilTagDetectionArray__Sequence__fini(apriltag_msgs__msg__AprilTagDetectionArray__Sequence * array);

/// Create array of apriltag_msgs/AprilTagDetectionArray messages.
/**
 * It allocates the memory for the array and
 * calls apriltag_msgs__msg__AprilTagDetectionArray__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_apriltag_msgs
apriltag_msgs__msg__AprilTagDetectionArray__Sequence *
apriltag_msgs__msg__AprilTagDetectionArray__Sequence__create(size_t size);

/// Destroy array of apriltag_msgs/AprilTagDetectionArray messages.
/**
 * It calls apriltag_msgs__msg__AprilTagDetectionArray__Sequence__fini() on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_apriltag_msgs
void
apriltag_msgs__msg__AprilTagDetectionArray__Sequence__destroy(apriltag_msgs__msg__AprilTagDetectionArray__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // APRILTAG_MSGS__MSG__APRIL_TAG_DETECTION_ARRAY__FUNCTIONS_H_
