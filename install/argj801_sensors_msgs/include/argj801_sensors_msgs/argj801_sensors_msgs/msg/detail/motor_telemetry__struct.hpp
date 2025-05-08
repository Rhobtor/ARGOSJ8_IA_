// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from argj801_sensors_msgs:msg/MotorTelemetry.idl
// generated code does not contain a copyright notice

#ifndef ARGJ801_SENSORS_MSGS__MSG__DETAIL__MOTOR_TELEMETRY__STRUCT_HPP_
#define ARGJ801_SENSORS_MSGS__MSG__DETAIL__MOTOR_TELEMETRY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__argj801_sensors_msgs__msg__MotorTelemetry __attribute__((deprecated))
#else
# define DEPRECATED__argj801_sensors_msgs__msg__MotorTelemetry __declspec(deprecated)
#endif

namespace argj801_sensors_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MotorTelemetry_
{
  using Type = MotorTelemetry_<ContainerAllocator>;

  explicit MotorTelemetry_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->endocer = 0ll;
      this->rpm = 0.0f;
    }
  }

  explicit MotorTelemetry_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->endocer = 0ll;
      this->rpm = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _endocer_type =
    int64_t;
  _endocer_type endocer;
  using _rpm_type =
    float;
  _rpm_type rpm;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__endocer(
    const int64_t & _arg)
  {
    this->endocer = _arg;
    return *this;
  }
  Type & set__rpm(
    const float & _arg)
  {
    this->rpm = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    argj801_sensors_msgs::msg::MotorTelemetry_<ContainerAllocator> *;
  using ConstRawPtr =
    const argj801_sensors_msgs::msg::MotorTelemetry_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<argj801_sensors_msgs::msg::MotorTelemetry_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<argj801_sensors_msgs::msg::MotorTelemetry_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      argj801_sensors_msgs::msg::MotorTelemetry_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<argj801_sensors_msgs::msg::MotorTelemetry_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      argj801_sensors_msgs::msg::MotorTelemetry_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<argj801_sensors_msgs::msg::MotorTelemetry_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<argj801_sensors_msgs::msg::MotorTelemetry_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<argj801_sensors_msgs::msg::MotorTelemetry_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__argj801_sensors_msgs__msg__MotorTelemetry
    std::shared_ptr<argj801_sensors_msgs::msg::MotorTelemetry_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__argj801_sensors_msgs__msg__MotorTelemetry
    std::shared_ptr<argj801_sensors_msgs::msg::MotorTelemetry_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MotorTelemetry_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->endocer != other.endocer) {
      return false;
    }
    if (this->rpm != other.rpm) {
      return false;
    }
    return true;
  }
  bool operator!=(const MotorTelemetry_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MotorTelemetry_

// alias to use template instance with default allocator
using MotorTelemetry =
  argj801_sensors_msgs::msg::MotorTelemetry_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace argj801_sensors_msgs

#endif  // ARGJ801_SENSORS_MSGS__MSG__DETAIL__MOTOR_TELEMETRY__STRUCT_HPP_
