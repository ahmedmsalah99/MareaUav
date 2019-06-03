// generated from rosidl_generator_cpp/resource/msg__struct.hpp.em
// generated code does not contain a copyright notice

#ifndef APRILTAG_MSGS__MSG__APRIL_TAG_DETECTION_ARRAY__STRUCT_HPP_
#define APRILTAG_MSGS__MSG__APRIL_TAG_DETECTION_ARRAY__STRUCT_HPP_

// Protect against ERROR being predefined on Windows, in case somebody defines a
// constant by that name.
#if defined(_WIN32) && defined(ERROR)
  #undef ERROR
#endif

#include <rosidl_generator_cpp/bounded_vector.hpp>
#include <rosidl_generator_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

// include message dependencies
#include "apriltag_msgs/msg/april_tag_detection.hpp"  // detections
#include "std_msgs/msg/header.hpp"  // header

#ifndef _WIN32
# define DEPRECATED_apriltag_msgs_msg_AprilTagDetectionArray __attribute__((deprecated))
#else
# define DEPRECATED_apriltag_msgs_msg_AprilTagDetectionArray __declspec(deprecated)
#endif

namespace apriltag_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AprilTagDetectionArray_
{
  using Type = AprilTagDetectionArray_<ContainerAllocator>;

  explicit AprilTagDetectionArray_(rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit AprilTagDetectionArray_(const ContainerAllocator & _alloc, rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _detections_type =
    std::vector<apriltag_msgs::msg::AprilTagDetection_<ContainerAllocator>, typename ContainerAllocator::template rebind<apriltag_msgs::msg::AprilTagDetection_<ContainerAllocator>>::other>;
  _detections_type detections;

  // setters for named parameter idiom
  Type * set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return this;
  }
  Type * set__detections(
    const std::vector<apriltag_msgs::msg::AprilTagDetection_<ContainerAllocator>, typename ContainerAllocator::template rebind<apriltag_msgs::msg::AprilTagDetection_<ContainerAllocator>>::other> & _arg)
  {
    this->detections = _arg;
    return this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    apriltag_msgs::msg::AprilTagDetectionArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const apriltag_msgs::msg::AprilTagDetectionArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<apriltag_msgs::msg::AprilTagDetectionArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<apriltag_msgs::msg::AprilTagDetectionArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      apriltag_msgs::msg::AprilTagDetectionArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<apriltag_msgs::msg::AprilTagDetectionArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      apriltag_msgs::msg::AprilTagDetectionArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<apriltag_msgs::msg::AprilTagDetectionArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<apriltag_msgs::msg::AprilTagDetectionArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<apriltag_msgs::msg::AprilTagDetectionArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED_apriltag_msgs_msg_AprilTagDetectionArray
    std::shared_ptr<apriltag_msgs::msg::AprilTagDetectionArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED_apriltag_msgs_msg_AprilTagDetectionArray
    std::shared_ptr<apriltag_msgs::msg::AprilTagDetectionArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AprilTagDetectionArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->detections != other.detections) {
      return false;
    }
    return true;
  }
  bool operator!=(const AprilTagDetectionArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AprilTagDetectionArray_

// alias to use template instance with default allocator
using AprilTagDetectionArray =
  apriltag_msgs::msg::AprilTagDetectionArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace apriltag_msgs

#endif  // APRILTAG_MSGS__MSG__APRIL_TAG_DETECTION_ARRAY__STRUCT_HPP_
