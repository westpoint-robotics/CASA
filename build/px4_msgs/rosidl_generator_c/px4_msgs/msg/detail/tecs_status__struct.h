// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4_msgs:msg/TecsStatus.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__TECS_STATUS__STRUCT_H_
#define PX4_MSGS__MSG__DETAIL__TECS_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'TECS_MODE_NORMAL'.
enum
{
  px4_msgs__msg__TecsStatus__TECS_MODE_NORMAL = 0
};

/// Constant 'TECS_MODE_UNDERSPEED'.
enum
{
  px4_msgs__msg__TecsStatus__TECS_MODE_UNDERSPEED = 1
};

/// Constant 'TECS_MODE_TAKEOFF'.
enum
{
  px4_msgs__msg__TecsStatus__TECS_MODE_TAKEOFF = 2
};

/// Constant 'TECS_MODE_LAND'.
enum
{
  px4_msgs__msg__TecsStatus__TECS_MODE_LAND = 3
};

/// Constant 'TECS_MODE_LAND_THROTTLELIM'.
enum
{
  px4_msgs__msg__TecsStatus__TECS_MODE_LAND_THROTTLELIM = 4
};

/// Constant 'TECS_MODE_BAD_DESCENT'.
enum
{
  px4_msgs__msg__TecsStatus__TECS_MODE_BAD_DESCENT = 5
};

/// Constant 'TECS_MODE_CLIMBOUT'.
enum
{
  px4_msgs__msg__TecsStatus__TECS_MODE_CLIMBOUT = 6
};

/// Struct defined in msg/TecsStatus in the package px4_msgs.
typedef struct px4_msgs__msg__TecsStatus
{
  /// time since system start (microseconds)
  uint64_t timestamp;
  float altitude_sp;
  float altitude_filtered;
  float height_rate_setpoint;
  float height_rate;
  float equivalent_airspeed_sp;
  float true_airspeed_sp;
  float true_airspeed_filtered;
  float true_airspeed_derivative_sp;
  float true_airspeed_derivative;
  float true_airspeed_derivative_raw;
  float true_airspeed_innovation;
  float total_energy_error;
  float energy_distribution_error;
  float total_energy_rate_error;
  float energy_distribution_rate_error;
  float total_energy;
  float total_energy_rate;
  float total_energy_balance;
  float total_energy_balance_rate;
  float total_energy_sp;
  float total_energy_rate_sp;
  float total_energy_balance_sp;
  float total_energy_balance_rate_sp;
  float throttle_integ;
  float pitch_integ;
  float throttle_sp;
  float pitch_sp_rad;
  /// estimated throttle value [0,1] required to fly level at equivalent_airspeed_sp in the current atmospheric conditions
  float throttle_trim;
  uint8_t mode;
} px4_msgs__msg__TecsStatus;

// Struct for a sequence of px4_msgs__msg__TecsStatus.
typedef struct px4_msgs__msg__TecsStatus__Sequence
{
  px4_msgs__msg__TecsStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4_msgs__msg__TecsStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4_MSGS__MSG__DETAIL__TECS_STATUS__STRUCT_H_
