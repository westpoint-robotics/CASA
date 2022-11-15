// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from px4_msgs:msg/VehicleOpticalFlowVel.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/vehicle_optical_flow_vel__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "px4_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "px4_msgs/msg/detail/vehicle_optical_flow_vel__struct.h"
#include "px4_msgs/msg/detail/vehicle_optical_flow_vel__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _VehicleOpticalFlowVel__ros_msg_type = px4_msgs__msg__VehicleOpticalFlowVel;

static bool _VehicleOpticalFlowVel__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _VehicleOpticalFlowVel__ros_msg_type * ros_message = static_cast<const _VehicleOpticalFlowVel__ros_msg_type *>(untyped_ros_message);
  // Field name: timestamp
  {
    cdr << ros_message->timestamp;
  }

  // Field name: timestamp_sample
  {
    cdr << ros_message->timestamp_sample;
  }

  // Field name: vel_body
  {
    size_t size = 2;
    auto array_ptr = ros_message->vel_body;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: vel_ne
  {
    size_t size = 2;
    auto array_ptr = ros_message->vel_ne;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: flow_uncompensated_integral
  {
    size_t size = 2;
    auto array_ptr = ros_message->flow_uncompensated_integral;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: flow_compensated_integral
  {
    size_t size = 2;
    auto array_ptr = ros_message->flow_compensated_integral;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: gyro_rate
  {
    size_t size = 3;
    auto array_ptr = ros_message->gyro_rate;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: gyro_rate_integral
  {
    size_t size = 3;
    auto array_ptr = ros_message->gyro_rate_integral;
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _VehicleOpticalFlowVel__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _VehicleOpticalFlowVel__ros_msg_type * ros_message = static_cast<_VehicleOpticalFlowVel__ros_msg_type *>(untyped_ros_message);
  // Field name: timestamp
  {
    cdr >> ros_message->timestamp;
  }

  // Field name: timestamp_sample
  {
    cdr >> ros_message->timestamp_sample;
  }

  // Field name: vel_body
  {
    size_t size = 2;
    auto array_ptr = ros_message->vel_body;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: vel_ne
  {
    size_t size = 2;
    auto array_ptr = ros_message->vel_ne;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: flow_uncompensated_integral
  {
    size_t size = 2;
    auto array_ptr = ros_message->flow_uncompensated_integral;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: flow_compensated_integral
  {
    size_t size = 2;
    auto array_ptr = ros_message->flow_compensated_integral;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: gyro_rate
  {
    size_t size = 3;
    auto array_ptr = ros_message->gyro_rate;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: gyro_rate_integral
  {
    size_t size = 3;
    auto array_ptr = ros_message->gyro_rate_integral;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t get_serialized_size_px4_msgs__msg__VehicleOpticalFlowVel(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _VehicleOpticalFlowVel__ros_msg_type * ros_message = static_cast<const _VehicleOpticalFlowVel__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name timestamp
  {
    size_t item_size = sizeof(ros_message->timestamp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name timestamp_sample
  {
    size_t item_size = sizeof(ros_message->timestamp_sample);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name vel_body
  {
    size_t array_size = 2;
    auto array_ptr = ros_message->vel_body;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name vel_ne
  {
    size_t array_size = 2;
    auto array_ptr = ros_message->vel_ne;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name flow_uncompensated_integral
  {
    size_t array_size = 2;
    auto array_ptr = ros_message->flow_uncompensated_integral;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name flow_compensated_integral
  {
    size_t array_size = 2;
    auto array_ptr = ros_message->flow_compensated_integral;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name gyro_rate
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->gyro_rate;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name gyro_rate_integral
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->gyro_rate_integral;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _VehicleOpticalFlowVel__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_px4_msgs__msg__VehicleOpticalFlowVel(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t max_serialized_size_px4_msgs__msg__VehicleOpticalFlowVel(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: timestamp
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: timestamp_sample
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: vel_body
  {
    size_t array_size = 2;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: vel_ne
  {
    size_t array_size = 2;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: flow_uncompensated_integral
  {
    size_t array_size = 2;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: flow_compensated_integral
  {
    size_t array_size = 2;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: gyro_rate
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: gyro_rate_integral
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _VehicleOpticalFlowVel__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_px4_msgs__msg__VehicleOpticalFlowVel(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_VehicleOpticalFlowVel = {
  "px4_msgs::msg",
  "VehicleOpticalFlowVel",
  _VehicleOpticalFlowVel__cdr_serialize,
  _VehicleOpticalFlowVel__cdr_deserialize,
  _VehicleOpticalFlowVel__get_serialized_size,
  _VehicleOpticalFlowVel__max_serialized_size
};

static rosidl_message_type_support_t _VehicleOpticalFlowVel__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_VehicleOpticalFlowVel,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, px4_msgs, msg, VehicleOpticalFlowVel)() {
  return &_VehicleOpticalFlowVel__type_support;
}

#if defined(__cplusplus)
}
#endif
