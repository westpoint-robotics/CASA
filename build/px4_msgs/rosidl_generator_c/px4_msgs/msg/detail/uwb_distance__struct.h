// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4_msgs:msg/UwbDistance.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__UWB_DISTANCE__STRUCT_H_
#define PX4_MSGS__MSG__DETAIL__UWB_DISTANCE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/UwbDistance in the package px4_msgs.
/**
  * UWB distance contains the distance information measured by an ultra-wideband positioning system,
  * such as Pozyx or NXP Rddrone.
 */
typedef struct px4_msgs__msg__UwbDistance
{
  /// time since system start (microseconds)
  uint64_t timestamp;
  /// Time of UWB device in ms
  uint32_t time_uwb_ms;
  /// Number of Ranges since last Start of Ranging
  uint32_t counter;
  /// UWB SessionID
  uint32_t sessionid;
  /// Time between Ranging Rounds in ms
  uint32_t time_offset;
  /// status feedback #
  uint16_t status;
  /// distance in cm to each UWB Anchor (UWB Device which takes part in Ranging)
  uint16_t anchor_distance[12];
  /// Visual line of sight yes/no
  bool nlos[12];
  /// Angle of arrival of first incoming RX msg
  float aoafirst[12];
  /// Position of the Landing point in relation to the Drone (x,y,z in Meters NED)
  float position[3];
} px4_msgs__msg__UwbDistance;

// Struct for a sequence of px4_msgs__msg__UwbDistance.
typedef struct px4_msgs__msg__UwbDistance__Sequence
{
  px4_msgs__msg__UwbDistance * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4_msgs__msg__UwbDistance__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4_MSGS__MSG__DETAIL__UWB_DISTANCE__STRUCT_H_
