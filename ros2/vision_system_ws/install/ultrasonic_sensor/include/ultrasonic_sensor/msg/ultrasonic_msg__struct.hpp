// generated from rosidl_generator_cpp/resource/msg__struct.hpp.em
// generated code does not contain a copyright notice

#ifndef ULTRASONIC_SENSOR__MSG__ULTRASONIC_MSG__STRUCT_HPP_
#define ULTRASONIC_SENSOR__MSG__ULTRASONIC_MSG__STRUCT_HPP_

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
# define DEPRECATED_ultrasonic_sensor_msg_UltrasonicMsg __attribute__((deprecated))
#else
# define DEPRECATED_ultrasonic_sensor_msg_UltrasonicMsg __declspec(deprecated)
#endif

namespace ultrasonic_sensor
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct UltrasonicMsg_
{
  using Type = UltrasonicMsg_<ContainerAllocator>;

  explicit UltrasonicMsg_(rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  {
    if (rosidl_generator_cpp::MessageInitialization::ALL == _init ||
      rosidl_generator_cpp::MessageInitialization::ZERO == _init)
    {
      this->z_position = 0.0;
      this->z_velocity = 0.0;
    }
  }

  explicit UltrasonicMsg_(const ContainerAllocator & _alloc, rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_generator_cpp::MessageInitialization::ALL == _init ||
      rosidl_generator_cpp::MessageInitialization::ZERO == _init)
    {
      this->z_position = 0.0;
      this->z_velocity = 0.0;
    }
  }

  // field types and members
  using _z_position_type =
    double;
  _z_position_type z_position;
  using _z_velocity_type =
    double;
  _z_velocity_type z_velocity;

  // setters for named parameter idiom
  Type * set__z_position(
    const double & _arg)
  {
    this->z_position = _arg;
    return this;
  }
  Type * set__z_velocity(
    const double & _arg)
  {
    this->z_velocity = _arg;
    return this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ultrasonic_sensor::msg::UltrasonicMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const ultrasonic_sensor::msg::UltrasonicMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ultrasonic_sensor::msg::UltrasonicMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ultrasonic_sensor::msg::UltrasonicMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ultrasonic_sensor::msg::UltrasonicMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ultrasonic_sensor::msg::UltrasonicMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ultrasonic_sensor::msg::UltrasonicMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ultrasonic_sensor::msg::UltrasonicMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ultrasonic_sensor::msg::UltrasonicMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ultrasonic_sensor::msg::UltrasonicMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED_ultrasonic_sensor_msg_UltrasonicMsg
    std::shared_ptr<ultrasonic_sensor::msg::UltrasonicMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED_ultrasonic_sensor_msg_UltrasonicMsg
    std::shared_ptr<ultrasonic_sensor::msg::UltrasonicMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UltrasonicMsg_ & other) const
  {
    if (this->z_position != other.z_position) {
      return false;
    }
    if (this->z_velocity != other.z_velocity) {
      return false;
    }
    return true;
  }
  bool operator!=(const UltrasonicMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UltrasonicMsg_

// alias to use template instance with default allocator
using UltrasonicMsg =
  ultrasonic_sensor::msg::UltrasonicMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ultrasonic_sensor

#endif  // ULTRASONIC_SENSOR__MSG__ULTRASONIC_MSG__STRUCT_HPP_
