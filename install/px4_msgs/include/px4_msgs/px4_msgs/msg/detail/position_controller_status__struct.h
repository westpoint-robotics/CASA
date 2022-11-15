// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4_msgs:msg/PositionControllerStatus.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__POSITION_CONTROLLER_STATUS__STRUCT_H_
#define PX4_MSGS__MSG__DETAIL__POSITION_CONTROLLER_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/PositionControllerStatus in the package px4_msgs.
typedef struct px4_msgs__msg__PositionControllerStatus
{
  /// time since system start (microseconds)
  uint64_t timestamp;
  float nav_roll;
  float nav_pitch;
  /// set to NAN if not valid
  float nav_bearing;
  /// set to NAN if not valid
  float target_bearing;
  /// set to NAN if not valid
  float xtrack_error;
  float wp_dist;
  /// the optimal distance to a waypoint to switch to the next
  float acceptance_radius;
  /// NaN if not set
  float yaw_acceptance;
  /// the optimal vertical distance to a waypoint to switch to the next
  float altitude_acceptance;
  uint8_t type;
} px4_msgs__msg__PositionControllerStatus;

// Struct for a sequence of px4_msgs__msg__PositionControllerStatus.
typedef struct px4_msgs__msg__PositionControllerStatus__Sequence
{
  px4_msgs__msg__PositionControllerStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4_msgs__msg__PositionControllerStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4_MSGS__MSG__DETAIL__POSITION_CONTROLLER_STATUS__STRUCT_H_
