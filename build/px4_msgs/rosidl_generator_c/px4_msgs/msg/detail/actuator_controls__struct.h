// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4_msgs:msg/ActuatorControls.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__ACTUATOR_CONTROLS__STRUCT_H_
#define PX4_MSGS__MSG__DETAIL__ACTUATOR_CONTROLS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'NUM_ACTUATOR_CONTROLS'.
enum
{
  px4_msgs__msg__ActuatorControls__NUM_ACTUATOR_CONTROLS = 9
};

/// Constant 'NUM_ACTUATOR_CONTROL_GROUPS'.
enum
{
  px4_msgs__msg__ActuatorControls__NUM_ACTUATOR_CONTROL_GROUPS = 4
};

/// Constant 'INDEX_ROLL'.
enum
{
  px4_msgs__msg__ActuatorControls__INDEX_ROLL = 0
};

/// Constant 'INDEX_PITCH'.
enum
{
  px4_msgs__msg__ActuatorControls__INDEX_PITCH = 1
};

/// Constant 'INDEX_YAW'.
enum
{
  px4_msgs__msg__ActuatorControls__INDEX_YAW = 2
};

/// Constant 'INDEX_THROTTLE'.
enum
{
  px4_msgs__msg__ActuatorControls__INDEX_THROTTLE = 3
};

/// Constant 'INDEX_FLAPS'.
enum
{
  px4_msgs__msg__ActuatorControls__INDEX_FLAPS = 4
};

/// Constant 'INDEX_SPOILERS'.
enum
{
  px4_msgs__msg__ActuatorControls__INDEX_SPOILERS = 5
};

/// Constant 'INDEX_AIRBRAKES'.
enum
{
  px4_msgs__msg__ActuatorControls__INDEX_AIRBRAKES = 6
};

/// Constant 'INDEX_LANDING_GEAR'.
enum
{
  px4_msgs__msg__ActuatorControls__INDEX_LANDING_GEAR = 7
};

/// Constant 'INDEX_GIMBAL_SHUTTER'.
enum
{
  px4_msgs__msg__ActuatorControls__INDEX_GIMBAL_SHUTTER = 3
};

/// Constant 'INDEX_CAMERA_ZOOM'.
enum
{
  px4_msgs__msg__ActuatorControls__INDEX_CAMERA_ZOOM = 4
};

/// Constant 'INDEX_COLLECTIVE_TILT'.
enum
{
  px4_msgs__msg__ActuatorControls__INDEX_COLLECTIVE_TILT = 8
};

/// Constant 'GROUP_INDEX_ATTITUDE'.
enum
{
  px4_msgs__msg__ActuatorControls__GROUP_INDEX_ATTITUDE = 0
};

/// Constant 'GROUP_INDEX_ATTITUDE_ALTERNATE'.
enum
{
  px4_msgs__msg__ActuatorControls__GROUP_INDEX_ATTITUDE_ALTERNATE = 1
};

/// Constant 'GROUP_INDEX_GIMBAL'.
enum
{
  px4_msgs__msg__ActuatorControls__GROUP_INDEX_GIMBAL = 2
};

/// Struct defined in msg/ActuatorControls in the package px4_msgs.
typedef struct px4_msgs__msg__ActuatorControls
{
  /// time since system start (microseconds)
  uint64_t timestamp;
  /// the timestamp the data this control response is based on was sampled
  uint64_t timestamp_sample;
  float control[9];
} px4_msgs__msg__ActuatorControls;

// Struct for a sequence of px4_msgs__msg__ActuatorControls.
typedef struct px4_msgs__msg__ActuatorControls__Sequence
{
  px4_msgs__msg__ActuatorControls * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4_msgs__msg__ActuatorControls__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4_MSGS__MSG__DETAIL__ACTUATOR_CONTROLS__STRUCT_H_
