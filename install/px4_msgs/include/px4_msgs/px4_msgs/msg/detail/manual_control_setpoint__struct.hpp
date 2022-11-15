// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from px4_msgs:msg/ManualControlSetpoint.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__MANUAL_CONTROL_SETPOINT__STRUCT_HPP_
#define PX4_MSGS__MSG__DETAIL__MANUAL_CONTROL_SETPOINT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__px4_msgs__msg__ManualControlSetpoint __attribute__((deprecated))
#else
# define DEPRECATED__px4_msgs__msg__ManualControlSetpoint __declspec(deprecated)
#endif

namespace px4_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ManualControlSetpoint_
{
  using Type = ManualControlSetpoint_<ContainerAllocator>;

  explicit ManualControlSetpoint_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestamp = 0ull;
      this->timestamp_sample = 0ull;
      this->valid = false;
      this->data_source = 0;
      this->x = 0.0f;
      this->y = 0.0f;
      this->z = 0.0f;
      this->r = 0.0f;
      this->flaps = 0.0f;
      this->aux1 = 0.0f;
      this->aux2 = 0.0f;
      this->aux3 = 0.0f;
      this->aux4 = 0.0f;
      this->aux5 = 0.0f;
      this->aux6 = 0.0f;
      this->sticks_moving = false;
    }
  }

  explicit ManualControlSetpoint_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestamp = 0ull;
      this->timestamp_sample = 0ull;
      this->valid = false;
      this->data_source = 0;
      this->x = 0.0f;
      this->y = 0.0f;
      this->z = 0.0f;
      this->r = 0.0f;
      this->flaps = 0.0f;
      this->aux1 = 0.0f;
      this->aux2 = 0.0f;
      this->aux3 = 0.0f;
      this->aux4 = 0.0f;
      this->aux5 = 0.0f;
      this->aux6 = 0.0f;
      this->sticks_moving = false;
    }
  }

  // field types and members
  using _timestamp_type =
    uint64_t;
  _timestamp_type timestamp;
  using _timestamp_sample_type =
    uint64_t;
  _timestamp_sample_type timestamp_sample;
  using _valid_type =
    bool;
  _valid_type valid;
  using _data_source_type =
    uint8_t;
  _data_source_type data_source;
  using _x_type =
    float;
  _x_type x;
  using _y_type =
    float;
  _y_type y;
  using _z_type =
    float;
  _z_type z;
  using _r_type =
    float;
  _r_type r;
  using _flaps_type =
    float;
  _flaps_type flaps;
  using _aux1_type =
    float;
  _aux1_type aux1;
  using _aux2_type =
    float;
  _aux2_type aux2;
  using _aux3_type =
    float;
  _aux3_type aux3;
  using _aux4_type =
    float;
  _aux4_type aux4;
  using _aux5_type =
    float;
  _aux5_type aux5;
  using _aux6_type =
    float;
  _aux6_type aux6;
  using _sticks_moving_type =
    bool;
  _sticks_moving_type sticks_moving;

  // setters for named parameter idiom
  Type & set__timestamp(
    const uint64_t & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }
  Type & set__timestamp_sample(
    const uint64_t & _arg)
  {
    this->timestamp_sample = _arg;
    return *this;
  }
  Type & set__valid(
    const bool & _arg)
  {
    this->valid = _arg;
    return *this;
  }
  Type & set__data_source(
    const uint8_t & _arg)
  {
    this->data_source = _arg;
    return *this;
  }
  Type & set__x(
    const float & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const float & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const float & _arg)
  {
    this->z = _arg;
    return *this;
  }
  Type & set__r(
    const float & _arg)
  {
    this->r = _arg;
    return *this;
  }
  Type & set__flaps(
    const float & _arg)
  {
    this->flaps = _arg;
    return *this;
  }
  Type & set__aux1(
    const float & _arg)
  {
    this->aux1 = _arg;
    return *this;
  }
  Type & set__aux2(
    const float & _arg)
  {
    this->aux2 = _arg;
    return *this;
  }
  Type & set__aux3(
    const float & _arg)
  {
    this->aux3 = _arg;
    return *this;
  }
  Type & set__aux4(
    const float & _arg)
  {
    this->aux4 = _arg;
    return *this;
  }
  Type & set__aux5(
    const float & _arg)
  {
    this->aux5 = _arg;
    return *this;
  }
  Type & set__aux6(
    const float & _arg)
  {
    this->aux6 = _arg;
    return *this;
  }
  Type & set__sticks_moving(
    const bool & _arg)
  {
    this->sticks_moving = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t SOURCE_UNKNOWN =
    0u;
  static constexpr uint8_t SOURCE_RC =
    1u;
  static constexpr uint8_t SOURCE_MAVLINK_0 =
    2u;
  static constexpr uint8_t SOURCE_MAVLINK_1 =
    3u;
  static constexpr uint8_t SOURCE_MAVLINK_2 =
    4u;
  static constexpr uint8_t SOURCE_MAVLINK_3 =
    5u;
  static constexpr uint8_t SOURCE_MAVLINK_4 =
    6u;
  static constexpr uint8_t SOURCE_MAVLINK_5 =
    7u;

  // pointer types
  using RawPtr =
    px4_msgs::msg::ManualControlSetpoint_<ContainerAllocator> *;
  using ConstRawPtr =
    const px4_msgs::msg::ManualControlSetpoint_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<px4_msgs::msg::ManualControlSetpoint_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<px4_msgs::msg::ManualControlSetpoint_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      px4_msgs::msg::ManualControlSetpoint_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<px4_msgs::msg::ManualControlSetpoint_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      px4_msgs::msg::ManualControlSetpoint_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<px4_msgs::msg::ManualControlSetpoint_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<px4_msgs::msg::ManualControlSetpoint_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<px4_msgs::msg::ManualControlSetpoint_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__px4_msgs__msg__ManualControlSetpoint
    std::shared_ptr<px4_msgs::msg::ManualControlSetpoint_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__px4_msgs__msg__ManualControlSetpoint
    std::shared_ptr<px4_msgs::msg::ManualControlSetpoint_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ManualControlSetpoint_ & other) const
  {
    if (this->timestamp != other.timestamp) {
      return false;
    }
    if (this->timestamp_sample != other.timestamp_sample) {
      return false;
    }
    if (this->valid != other.valid) {
      return false;
    }
    if (this->data_source != other.data_source) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    if (this->r != other.r) {
      return false;
    }
    if (this->flaps != other.flaps) {
      return false;
    }
    if (this->aux1 != other.aux1) {
      return false;
    }
    if (this->aux2 != other.aux2) {
      return false;
    }
    if (this->aux3 != other.aux3) {
      return false;
    }
    if (this->aux4 != other.aux4) {
      return false;
    }
    if (this->aux5 != other.aux5) {
      return false;
    }
    if (this->aux6 != other.aux6) {
      return false;
    }
    if (this->sticks_moving != other.sticks_moving) {
      return false;
    }
    return true;
  }
  bool operator!=(const ManualControlSetpoint_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ManualControlSetpoint_

// alias to use template instance with default allocator
using ManualControlSetpoint =
  px4_msgs::msg::ManualControlSetpoint_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr uint8_t ManualControlSetpoint_<ContainerAllocator>::SOURCE_UNKNOWN;
template<typename ContainerAllocator>
constexpr uint8_t ManualControlSetpoint_<ContainerAllocator>::SOURCE_RC;
template<typename ContainerAllocator>
constexpr uint8_t ManualControlSetpoint_<ContainerAllocator>::SOURCE_MAVLINK_0;
template<typename ContainerAllocator>
constexpr uint8_t ManualControlSetpoint_<ContainerAllocator>::SOURCE_MAVLINK_1;
template<typename ContainerAllocator>
constexpr uint8_t ManualControlSetpoint_<ContainerAllocator>::SOURCE_MAVLINK_2;
template<typename ContainerAllocator>
constexpr uint8_t ManualControlSetpoint_<ContainerAllocator>::SOURCE_MAVLINK_3;
template<typename ContainerAllocator>
constexpr uint8_t ManualControlSetpoint_<ContainerAllocator>::SOURCE_MAVLINK_4;
template<typename ContainerAllocator>
constexpr uint8_t ManualControlSetpoint_<ContainerAllocator>::SOURCE_MAVLINK_5;

}  // namespace msg

}  // namespace px4_msgs

#endif  // PX4_MSGS__MSG__DETAIL__MANUAL_CONTROL_SETPOINT__STRUCT_HPP_
