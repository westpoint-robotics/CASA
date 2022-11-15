// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from px4_msgs:msg/TecsStatus.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/tecs_status__rosidl_typesupport_fastrtps_cpp.hpp"
#include "px4_msgs/msg/detail/tecs_status__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace px4_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_px4_msgs
cdr_serialize(
  const px4_msgs::msg::TecsStatus & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: timestamp
  cdr << ros_message.timestamp;
  // Member: altitude_sp
  cdr << ros_message.altitude_sp;
  // Member: altitude_filtered
  cdr << ros_message.altitude_filtered;
  // Member: height_rate_setpoint
  cdr << ros_message.height_rate_setpoint;
  // Member: height_rate
  cdr << ros_message.height_rate;
  // Member: equivalent_airspeed_sp
  cdr << ros_message.equivalent_airspeed_sp;
  // Member: true_airspeed_sp
  cdr << ros_message.true_airspeed_sp;
  // Member: true_airspeed_filtered
  cdr << ros_message.true_airspeed_filtered;
  // Member: true_airspeed_derivative_sp
  cdr << ros_message.true_airspeed_derivative_sp;
  // Member: true_airspeed_derivative
  cdr << ros_message.true_airspeed_derivative;
  // Member: true_airspeed_derivative_raw
  cdr << ros_message.true_airspeed_derivative_raw;
  // Member: true_airspeed_innovation
  cdr << ros_message.true_airspeed_innovation;
  // Member: total_energy_error
  cdr << ros_message.total_energy_error;
  // Member: energy_distribution_error
  cdr << ros_message.energy_distribution_error;
  // Member: total_energy_rate_error
  cdr << ros_message.total_energy_rate_error;
  // Member: energy_distribution_rate_error
  cdr << ros_message.energy_distribution_rate_error;
  // Member: total_energy
  cdr << ros_message.total_energy;
  // Member: total_energy_rate
  cdr << ros_message.total_energy_rate;
  // Member: total_energy_balance
  cdr << ros_message.total_energy_balance;
  // Member: total_energy_balance_rate
  cdr << ros_message.total_energy_balance_rate;
  // Member: total_energy_sp
  cdr << ros_message.total_energy_sp;
  // Member: total_energy_rate_sp
  cdr << ros_message.total_energy_rate_sp;
  // Member: total_energy_balance_sp
  cdr << ros_message.total_energy_balance_sp;
  // Member: total_energy_balance_rate_sp
  cdr << ros_message.total_energy_balance_rate_sp;
  // Member: throttle_integ
  cdr << ros_message.throttle_integ;
  // Member: pitch_integ
  cdr << ros_message.pitch_integ;
  // Member: throttle_sp
  cdr << ros_message.throttle_sp;
  // Member: pitch_sp_rad
  cdr << ros_message.pitch_sp_rad;
  // Member: throttle_trim
  cdr << ros_message.throttle_trim;
  // Member: mode
  cdr << ros_message.mode;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_px4_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  px4_msgs::msg::TecsStatus & ros_message)
{
  // Member: timestamp
  cdr >> ros_message.timestamp;

  // Member: altitude_sp
  cdr >> ros_message.altitude_sp;

  // Member: altitude_filtered
  cdr >> ros_message.altitude_filtered;

  // Member: height_rate_setpoint
  cdr >> ros_message.height_rate_setpoint;

  // Member: height_rate
  cdr >> ros_message.height_rate;

  // Member: equivalent_airspeed_sp
  cdr >> ros_message.equivalent_airspeed_sp;

  // Member: true_airspeed_sp
  cdr >> ros_message.true_airspeed_sp;

  // Member: true_airspeed_filtered
  cdr >> ros_message.true_airspeed_filtered;

  // Member: true_airspeed_derivative_sp
  cdr >> ros_message.true_airspeed_derivative_sp;

  // Member: true_airspeed_derivative
  cdr >> ros_message.true_airspeed_derivative;

  // Member: true_airspeed_derivative_raw
  cdr >> ros_message.true_airspeed_derivative_raw;

  // Member: true_airspeed_innovation
  cdr >> ros_message.true_airspeed_innovation;

  // Member: total_energy_error
  cdr >> ros_message.total_energy_error;

  // Member: energy_distribution_error
  cdr >> ros_message.energy_distribution_error;

  // Member: total_energy_rate_error
  cdr >> ros_message.total_energy_rate_error;

  // Member: energy_distribution_rate_error
  cdr >> ros_message.energy_distribution_rate_error;

  // Member: total_energy
  cdr >> ros_message.total_energy;

  // Member: total_energy_rate
  cdr >> ros_message.total_energy_rate;

  // Member: total_energy_balance
  cdr >> ros_message.total_energy_balance;

  // Member: total_energy_balance_rate
  cdr >> ros_message.total_energy_balance_rate;

  // Member: total_energy_sp
  cdr >> ros_message.total_energy_sp;

  // Member: total_energy_rate_sp
  cdr >> ros_message.total_energy_rate_sp;

  // Member: total_energy_balance_sp
  cdr >> ros_message.total_energy_balance_sp;

  // Member: total_energy_balance_rate_sp
  cdr >> ros_message.total_energy_balance_rate_sp;

  // Member: throttle_integ
  cdr >> ros_message.throttle_integ;

  // Member: pitch_integ
  cdr >> ros_message.pitch_integ;

  // Member: throttle_sp
  cdr >> ros_message.throttle_sp;

  // Member: pitch_sp_rad
  cdr >> ros_message.pitch_sp_rad;

  // Member: throttle_trim
  cdr >> ros_message.throttle_trim;

  // Member: mode
  cdr >> ros_message.mode;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_px4_msgs
get_serialized_size(
  const px4_msgs::msg::TecsStatus & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: timestamp
  {
    size_t item_size = sizeof(ros_message.timestamp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: altitude_sp
  {
    size_t item_size = sizeof(ros_message.altitude_sp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: altitude_filtered
  {
    size_t item_size = sizeof(ros_message.altitude_filtered);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: height_rate_setpoint
  {
    size_t item_size = sizeof(ros_message.height_rate_setpoint);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: height_rate
  {
    size_t item_size = sizeof(ros_message.height_rate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: equivalent_airspeed_sp
  {
    size_t item_size = sizeof(ros_message.equivalent_airspeed_sp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: true_airspeed_sp
  {
    size_t item_size = sizeof(ros_message.true_airspeed_sp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: true_airspeed_filtered
  {
    size_t item_size = sizeof(ros_message.true_airspeed_filtered);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: true_airspeed_derivative_sp
  {
    size_t item_size = sizeof(ros_message.true_airspeed_derivative_sp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: true_airspeed_derivative
  {
    size_t item_size = sizeof(ros_message.true_airspeed_derivative);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: true_airspeed_derivative_raw
  {
    size_t item_size = sizeof(ros_message.true_airspeed_derivative_raw);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: true_airspeed_innovation
  {
    size_t item_size = sizeof(ros_message.true_airspeed_innovation);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: total_energy_error
  {
    size_t item_size = sizeof(ros_message.total_energy_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: energy_distribution_error
  {
    size_t item_size = sizeof(ros_message.energy_distribution_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: total_energy_rate_error
  {
    size_t item_size = sizeof(ros_message.total_energy_rate_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: energy_distribution_rate_error
  {
    size_t item_size = sizeof(ros_message.energy_distribution_rate_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: total_energy
  {
    size_t item_size = sizeof(ros_message.total_energy);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: total_energy_rate
  {
    size_t item_size = sizeof(ros_message.total_energy_rate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: total_energy_balance
  {
    size_t item_size = sizeof(ros_message.total_energy_balance);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: total_energy_balance_rate
  {
    size_t item_size = sizeof(ros_message.total_energy_balance_rate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: total_energy_sp
  {
    size_t item_size = sizeof(ros_message.total_energy_sp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: total_energy_rate_sp
  {
    size_t item_size = sizeof(ros_message.total_energy_rate_sp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: total_energy_balance_sp
  {
    size_t item_size = sizeof(ros_message.total_energy_balance_sp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: total_energy_balance_rate_sp
  {
    size_t item_size = sizeof(ros_message.total_energy_balance_rate_sp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: throttle_integ
  {
    size_t item_size = sizeof(ros_message.throttle_integ);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: pitch_integ
  {
    size_t item_size = sizeof(ros_message.pitch_integ);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: throttle_sp
  {
    size_t item_size = sizeof(ros_message.throttle_sp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: pitch_sp_rad
  {
    size_t item_size = sizeof(ros_message.pitch_sp_rad);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: throttle_trim
  {
    size_t item_size = sizeof(ros_message.throttle_trim);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: mode
  {
    size_t item_size = sizeof(ros_message.mode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_px4_msgs
max_serialized_size_TecsStatus(
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


  // Member: timestamp
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: altitude_sp
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: altitude_filtered
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: height_rate_setpoint
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: height_rate
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: equivalent_airspeed_sp
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: true_airspeed_sp
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: true_airspeed_filtered
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: true_airspeed_derivative_sp
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: true_airspeed_derivative
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: true_airspeed_derivative_raw
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: true_airspeed_innovation
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: total_energy_error
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: energy_distribution_error
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: total_energy_rate_error
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: energy_distribution_rate_error
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: total_energy
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: total_energy_rate
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: total_energy_balance
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: total_energy_balance_rate
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: total_energy_sp
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: total_energy_rate_sp
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: total_energy_balance_sp
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: total_energy_balance_rate_sp
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: throttle_integ
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: pitch_integ
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: throttle_sp
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: pitch_sp_rad
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: throttle_trim
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: mode
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static bool _TecsStatus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const px4_msgs::msg::TecsStatus *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _TecsStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<px4_msgs::msg::TecsStatus *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _TecsStatus__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const px4_msgs::msg::TecsStatus *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _TecsStatus__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_TecsStatus(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _TecsStatus__callbacks = {
  "px4_msgs::msg",
  "TecsStatus",
  _TecsStatus__cdr_serialize,
  _TecsStatus__cdr_deserialize,
  _TecsStatus__get_serialized_size,
  _TecsStatus__max_serialized_size
};

static rosidl_message_type_support_t _TecsStatus__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_TecsStatus__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace px4_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_px4_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<px4_msgs::msg::TecsStatus>()
{
  return &px4_msgs::msg::typesupport_fastrtps_cpp::_TecsStatus__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, px4_msgs, msg, TecsStatus)() {
  return &px4_msgs::msg::typesupport_fastrtps_cpp::_TecsStatus__handle;
}

#ifdef __cplusplus
}
#endif
