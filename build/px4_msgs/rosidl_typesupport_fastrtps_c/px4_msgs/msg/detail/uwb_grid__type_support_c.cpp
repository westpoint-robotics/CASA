// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from px4_msgs:msg/UwbGrid.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/uwb_grid__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "px4_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "px4_msgs/msg/detail/uwb_grid__struct.h"
#include "px4_msgs/msg/detail/uwb_grid__functions.h"
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


using _UwbGrid__ros_msg_type = px4_msgs__msg__UwbGrid;

static bool _UwbGrid__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _UwbGrid__ros_msg_type * ros_message = static_cast<const _UwbGrid__ros_msg_type *>(untyped_ros_message);
  // Field name: timestamp
  {
    cdr << ros_message->timestamp;
  }

  // Field name: initator_time
  {
    cdr << ros_message->initator_time;
  }

  // Field name: num_anchors
  {
    cdr << ros_message->num_anchors;
  }

  // Field name: target_gps
  {
    size_t size = 4;
    auto array_ptr = ros_message->target_gps;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: target_pos
  {
    size_t size = 3;
    auto array_ptr = ros_message->target_pos;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: anchor_pos_0
  {
    size_t size = 3;
    auto array_ptr = ros_message->anchor_pos_0;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: anchor_pos_1
  {
    size_t size = 3;
    auto array_ptr = ros_message->anchor_pos_1;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: anchor_pos_2
  {
    size_t size = 3;
    auto array_ptr = ros_message->anchor_pos_2;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: anchor_pos_3
  {
    size_t size = 3;
    auto array_ptr = ros_message->anchor_pos_3;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: anchor_pos_4
  {
    size_t size = 3;
    auto array_ptr = ros_message->anchor_pos_4;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: anchor_pos_5
  {
    size_t size = 3;
    auto array_ptr = ros_message->anchor_pos_5;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: anchor_pos_6
  {
    size_t size = 3;
    auto array_ptr = ros_message->anchor_pos_6;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: anchor_pos_7
  {
    size_t size = 3;
    auto array_ptr = ros_message->anchor_pos_7;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: anchor_pos_8
  {
    size_t size = 3;
    auto array_ptr = ros_message->anchor_pos_8;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: anchor_pos_9
  {
    size_t size = 3;
    auto array_ptr = ros_message->anchor_pos_9;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: anchor_pos_10
  {
    size_t size = 3;
    auto array_ptr = ros_message->anchor_pos_10;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: anchor_pos_11
  {
    size_t size = 3;
    auto array_ptr = ros_message->anchor_pos_11;
    cdr.serializeArray(array_ptr, size);
  }

  return true;
}

static bool _UwbGrid__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _UwbGrid__ros_msg_type * ros_message = static_cast<_UwbGrid__ros_msg_type *>(untyped_ros_message);
  // Field name: timestamp
  {
    cdr >> ros_message->timestamp;
  }

  // Field name: initator_time
  {
    cdr >> ros_message->initator_time;
  }

  // Field name: num_anchors
  {
    cdr >> ros_message->num_anchors;
  }

  // Field name: target_gps
  {
    size_t size = 4;
    auto array_ptr = ros_message->target_gps;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: target_pos
  {
    size_t size = 3;
    auto array_ptr = ros_message->target_pos;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: anchor_pos_0
  {
    size_t size = 3;
    auto array_ptr = ros_message->anchor_pos_0;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: anchor_pos_1
  {
    size_t size = 3;
    auto array_ptr = ros_message->anchor_pos_1;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: anchor_pos_2
  {
    size_t size = 3;
    auto array_ptr = ros_message->anchor_pos_2;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: anchor_pos_3
  {
    size_t size = 3;
    auto array_ptr = ros_message->anchor_pos_3;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: anchor_pos_4
  {
    size_t size = 3;
    auto array_ptr = ros_message->anchor_pos_4;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: anchor_pos_5
  {
    size_t size = 3;
    auto array_ptr = ros_message->anchor_pos_5;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: anchor_pos_6
  {
    size_t size = 3;
    auto array_ptr = ros_message->anchor_pos_6;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: anchor_pos_7
  {
    size_t size = 3;
    auto array_ptr = ros_message->anchor_pos_7;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: anchor_pos_8
  {
    size_t size = 3;
    auto array_ptr = ros_message->anchor_pos_8;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: anchor_pos_9
  {
    size_t size = 3;
    auto array_ptr = ros_message->anchor_pos_9;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: anchor_pos_10
  {
    size_t size = 3;
    auto array_ptr = ros_message->anchor_pos_10;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: anchor_pos_11
  {
    size_t size = 3;
    auto array_ptr = ros_message->anchor_pos_11;
    cdr.deserializeArray(array_ptr, size);
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t get_serialized_size_px4_msgs__msg__UwbGrid(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _UwbGrid__ros_msg_type * ros_message = static_cast<const _UwbGrid__ros_msg_type *>(untyped_ros_message);
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
  // field.name initator_time
  {
    size_t item_size = sizeof(ros_message->initator_time);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name num_anchors
  {
    size_t item_size = sizeof(ros_message->num_anchors);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name target_gps
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->target_gps;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name target_pos
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->target_pos;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name anchor_pos_0
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->anchor_pos_0;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name anchor_pos_1
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->anchor_pos_1;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name anchor_pos_2
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->anchor_pos_2;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name anchor_pos_3
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->anchor_pos_3;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name anchor_pos_4
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->anchor_pos_4;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name anchor_pos_5
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->anchor_pos_5;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name anchor_pos_6
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->anchor_pos_6;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name anchor_pos_7
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->anchor_pos_7;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name anchor_pos_8
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->anchor_pos_8;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name anchor_pos_9
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->anchor_pos_9;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name anchor_pos_10
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->anchor_pos_10;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name anchor_pos_11
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->anchor_pos_11;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _UwbGrid__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_px4_msgs__msg__UwbGrid(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t max_serialized_size_px4_msgs__msg__UwbGrid(
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
  // member: initator_time
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: num_anchors
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: target_gps
  {
    size_t array_size = 4;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: target_pos
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: anchor_pos_0
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: anchor_pos_1
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: anchor_pos_2
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: anchor_pos_3
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: anchor_pos_4
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: anchor_pos_5
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: anchor_pos_6
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: anchor_pos_7
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: anchor_pos_8
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: anchor_pos_9
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: anchor_pos_10
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: anchor_pos_11
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _UwbGrid__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_px4_msgs__msg__UwbGrid(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_UwbGrid = {
  "px4_msgs::msg",
  "UwbGrid",
  _UwbGrid__cdr_serialize,
  _UwbGrid__cdr_deserialize,
  _UwbGrid__get_serialized_size,
  _UwbGrid__max_serialized_size
};

static rosidl_message_type_support_t _UwbGrid__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_UwbGrid,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, px4_msgs, msg, UwbGrid)() {
  return &_UwbGrid__type_support;
}

#if defined(__cplusplus)
}
#endif
