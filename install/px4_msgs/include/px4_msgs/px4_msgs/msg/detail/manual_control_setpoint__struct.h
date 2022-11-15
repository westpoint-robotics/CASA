// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4_msgs:msg/ManualControlSetpoint.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__MANUAL_CONTROL_SETPOINT__STRUCT_H_
#define PX4_MSGS__MSG__DETAIL__MANUAL_CONTROL_SETPOINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'SOURCE_UNKNOWN'.
enum
{
  px4_msgs__msg__ManualControlSetpoint__SOURCE_UNKNOWN = 0
};

/// Constant 'SOURCE_RC'.
/**
  * radio control (input_rc)
 */
enum
{
  px4_msgs__msg__ManualControlSetpoint__SOURCE_RC = 1
};

/// Constant 'SOURCE_MAVLINK_0'.
/**
  * mavlink instance 0
 */
enum
{
  px4_msgs__msg__ManualControlSetpoint__SOURCE_MAVLINK_0 = 2
};

/// Constant 'SOURCE_MAVLINK_1'.
/**
  * mavlink instance 1
 */
enum
{
  px4_msgs__msg__ManualControlSetpoint__SOURCE_MAVLINK_1 = 3
};

/// Constant 'SOURCE_MAVLINK_2'.
/**
  * mavlink instance 2
 */
enum
{
  px4_msgs__msg__ManualControlSetpoint__SOURCE_MAVLINK_2 = 4
};

/// Constant 'SOURCE_MAVLINK_3'.
/**
  * mavlink instance 3
 */
enum
{
  px4_msgs__msg__ManualControlSetpoint__SOURCE_MAVLINK_3 = 5
};

/// Constant 'SOURCE_MAVLINK_4'.
/**
  * mavlink instance 4
 */
enum
{
  px4_msgs__msg__ManualControlSetpoint__SOURCE_MAVLINK_4 = 6
};

/// Constant 'SOURCE_MAVLINK_5'.
/**
  * mavlink instance 5
 */
enum
{
  px4_msgs__msg__ManualControlSetpoint__SOURCE_MAVLINK_5 = 7
};

/// Struct defined in msg/ManualControlSetpoint in the package px4_msgs.
typedef struct px4_msgs__msg__ManualControlSetpoint
{
  /// time since system start (microseconds)
  uint64_t timestamp;
  /// the timestamp of the raw data (microseconds)
  uint64_t timestamp_sample;
  bool valid;
  uint8_t data_source;
  /// Any of the channels may not be available and be set to NaN
  /// to indicate that it does not contain valid data.
  /// The variable names follow the definition of the
  /// MANUAL_CONTROL mavlink message.
  /// The default range is from -1 to 1 (mavlink message -1000 to 1000)
  /// The range for the z variable is defined from 0 to 1. (The z field of
  /// the MANUAL_CONTROL mavlink message is defined from -1000 to 1000)
  /// stick position in x direction -1..1
  /// in general corresponds to forward/back motion or pitch of vehicle,
  /// in general a positive value means forward or negative pitch and
  /// a negative value means backward or positive pitch
  float x;
  /// stick position in y direction -1..1
  /// in general corresponds to right/left motion or roll of vehicle,
  /// in general a positive value means right or positive roll and
  /// a negative value means left or negative roll
  float y;
  /// throttle stick position 0..1
  /// in general corresponds to up/down motion or thrust of vehicle,
  /// in general the value corresponds to the demanded throttle by the user,
  /// if the input is used for setting the setpoint of a vertical position
  /// controller any value > 0.5 means up and any value < 0.5 means down
  float z;
  /// yaw stick/twist position, -1..1
  /// in general corresponds to the righthand rotation around the vertical
  /// (downwards) axis of the vehicle
  float r;
  /// flap position
  float flaps;
  float aux1;
  float aux2;
  float aux3;
  float aux4;
  float aux5;
  float aux6;
  bool sticks_moving;
} px4_msgs__msg__ManualControlSetpoint;

// Struct for a sequence of px4_msgs__msg__ManualControlSetpoint.
typedef struct px4_msgs__msg__ManualControlSetpoint__Sequence
{
  px4_msgs__msg__ManualControlSetpoint * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4_msgs__msg__ManualControlSetpoint__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4_MSGS__MSG__DETAIL__MANUAL_CONTROL_SETPOINT__STRUCT_H_
