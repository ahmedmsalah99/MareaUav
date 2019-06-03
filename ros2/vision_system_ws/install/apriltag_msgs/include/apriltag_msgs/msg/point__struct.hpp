// generated from rosidl_generator_cpp/resource/msg__struct.hpp.em
// generated code does not contain a copyright notice

#ifndef APRILTAG_MSGS__MSG__POINT__STRUCT_HPP_
#define APRILTAG_MSGS__MSG__POINT__STRUCT_HPP_

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

#ifndef _WIN32
# define DEPRECATED_apriltag_msgs_msg_Point __attribute__((deprecated))
#else
# define DEPRECATED_apriltag_msgs_msg_Point __declspec(deprecated)
#endif

namespace apriltag_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Point_
{
  using Type = Point_<ContainerAllocator>;

  explicit Point_(rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  {
    if (rosidl_generator_cpp::MessageInitialization::ALL == _init ||
      rosidl_generator_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
    }
  }

  explicit Point_(const ContainerAllocator & _alloc, rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_generator_cpp::MessageInitialization::ALL == _init ||
      rosidl_generator_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0;
      this->y = 0.0;
    }
  }

  // field types and members
  using _x_type =
    double;
  _x_type x;
  using _y_type =
    double;
  _y_type y;

  // setters for named parameter idiom
  Type * set__x(
    const double & _arg)
  {
    this->x = _arg;
    return this;
  }
  Type * set__y(
    const double & _arg)
  {
    this->y = _arg;
    return this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    apriltag_msgs::msg::Point_<ContainerAllocator> *;
  using ConstRawPtr =
    const apriltag_msgs::msg::Point_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<apriltag_msgs::msg::Point_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<apriltag_msgs::msg::Point_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      apriltag_msgs::msg::Point_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<apriltag_msgs::msg::Point_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      apriltag_msgs::msg::Point_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<apriltag_msgs::msg::Point_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<apriltag_msgs::msg::Point_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<apriltag_msgs::msg::Point_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED_apriltag_msgs_msg_Point
    std::shared_ptr<apriltag_msgs::msg::Point_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED_apriltag_msgs_msg_Point
    std::shared_ptr<apriltag_msgs::msg::Point_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Point_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    return true;
  }
  bool operator!=(const Point_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Point_

// alias to use template instance with default allocator
using Point =
  apriltag_msgs::msg::Point_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace apriltag_msgs

#endif  // APRILTAG_MSGS__MSG__POINT__STRUCT_HPP_
