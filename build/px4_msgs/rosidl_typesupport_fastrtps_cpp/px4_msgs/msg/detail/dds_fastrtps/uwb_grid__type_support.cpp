// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from px4_msgs:msg/UwbGrid.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/uwb_grid__rosidl_typesupport_fastrtps_cpp.hpp"
#include "px4_msgs/msg/detail/uwb_grid__struct.hpp"

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
  const px4_msgs::msg::UwbGrid & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: timestamp
  cdr << ros_message.timestamp;
  // Member: initator_time
  cdr << ros_message.initator_time;
  // Member: num_anchors
  cdr << ros_message.num_anchors;
  // Member: target_gps
  {
    cdr << ros_message.target_gps;
  }
  // Member: target_pos
  {
    cdr << ros_message.target_pos;
  }
  // Member: anchor_pos_0
  {
    cdr << ros_message.anchor_pos_0;
  }
  // Member: anchor_pos_1
  {
    cdr << ros_message.anchor_pos_1;
  }
  // Member: anchor_pos_2
  {
    cdr << ros_message.anchor_pos_2;
  }
  // Member: anchor_pos_3
  {
    cdr << ros_message.anchor_pos_3;
  }
  // Member: anchor_pos_4
  {
    cdr << ros_message.anchor_pos_4;
  }
  // Member: anchor_pos_5
  {
    cdr << ros_message.anchor_pos_5;
  }
  // Member: anchor_pos_6
  {
    cdr << ros_message.anchor_pos_6;
  }
  // Member: anchor_pos_7
  {
    cdr << ros_message.anchor_pos_7;
  }
  // Member: anchor_pos_8
  {
    cdr << ros_message.anchor_pos_8;
  }
  // Member: anchor_pos_9
  {
    cdr << ros_message.anchor_pos_9;
  }
  // Member: anchor_pos_10
  {
    cdr << ros_message.anchor_pos_10;
  }
  // Member: anchor_pos_11
  {
    cdr << ros_message.anchor_pos_11;
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_px4_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  px4_msgs::msg::UwbGrid & ros_message)
{
  // Member: timestamp
  cdr >> ros_message.timestamp;

  // Member: initator_time
  cdr >> ros_message.initator_time;

  // Member: num_anchors
  cdr >> ros_message.num_anchors;

  // Member: target_gps
  {
    cdr >> ros_message.target_gps;
  }

  // Member: target_pos
  {
    cdr >> ros_message.target_pos;
  }

  // Member: anchor_pos_0
  {
    cdr >> ros_message.anchor_pos_0;
  }

  // Member: anchor_pos_1
  {
    cdr >> ros_message.anchor_pos_1;
  }

  // Member: anchor_pos_2
  {
    cdr >> ros_message.anchor_pos_2;
  }

  // Member: anchor_pos_3
  {
    cdr >> ros_message.anchor_pos_3;
  }

  // Member: anchor_pos_4
  {
    cdr >> ros_message.anchor_pos_4;
  }

  // Member: anchor_pos_5
  {
    cdr >> ros_message.anchor_pos_5;
  }

  // Member: anchor_pos_6
  {
    cdr >> ros_message.anchor_pos_6;
  }

  // Member: anchor_pos_7
  {
    cdr >> ros_message.anchor_pos_7;
  }

  // Member: anchor_pos_8
  {
    cdr >> ros_message.anchor_pos_8;
  }

  // Member: anchor_pos_9
  {
    cdr >> ros_message.anchor_pos_9;
  }

  // Member: anchor_pos_10
  {
    cdr >> ros_message.anchor_pos_10;
  }

  // Member: anchor_pos_11
  {
    cdr >> ros_message.anchor_pos_11;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_px4_msgs
get_serialized_size(
  const px4_msgs::msg::UwbGrid & ros_message,
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
  // Member: initator_time
  {
    size_t item_size = sizeof(ros_message.initator_time);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: num_anchors
  {
    size_t item_size = sizeof(ros_message.num_anchors);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: target_gps
  {
    size_t array_size = 4;
    size_t item_size = sizeof(ros_message.target_gps[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: target_pos
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.target_pos[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: anchor_pos_0
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.anchor_pos_0[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: anchor_pos_1
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.anchor_pos_1[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: anchor_pos_2
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.anchor_pos_2[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: anchor_pos_3
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.anchor_pos_3[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: anchor_pos_4
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.anchor_pos_4[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: anchor_pos_5
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.anchor_pos_5[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: anchor_pos_6
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.anchor_pos_6[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: anchor_pos_7
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.anchor_pos_7[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: anchor_pos_8
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.anchor_pos_8[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: anchor_pos_9
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.anchor_pos_9[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: anchor_pos_10
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.anchor_pos_10[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: anchor_pos_11
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.anchor_pos_11[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_px4_msgs
max_serialized_size_UwbGrid(
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

  // Member: initator_time
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: num_anchors
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: target_gps
  {
    size_t array_size = 4;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: target_pos
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: anchor_pos_0
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: anchor_pos_1
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: anchor_pos_2
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: anchor_pos_3
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: anchor_pos_4
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: anchor_pos_5
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: anchor_pos_6
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: anchor_pos_7
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: anchor_pos_8
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: anchor_pos_9
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: anchor_pos_10
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  // Member: anchor_pos_11
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }

  return current_alignment - initial_alignment;
}

static bool _UwbGrid__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const px4_msgs::msg::UwbGrid *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _UwbGrid__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<px4_msgs::msg::UwbGrid *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _UwbGrid__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const px4_msgs::msg::UwbGrid *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _UwbGrid__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_UwbGrid(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _UwbGrid__callbacks = {
  "px4_msgs::msg",
  "UwbGrid",
  _UwbGrid__cdr_serialize,
  _UwbGrid__cdr_deserialize,
  _UwbGrid__get_serialized_size,
  _UwbGrid__max_serialized_size
};

static rosidl_message_type_support_t _UwbGrid__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_UwbGrid__callbacks,
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
get_message_type_support_handle<px4_msgs::msg::UwbGrid>()
{
  return &px4_msgs::msg::typesupport_fastrtps_cpp::_UwbGrid__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, px4_msgs, msg, UwbGrid)() {
  return &px4_msgs::msg::typesupport_fastrtps_cpp::_UwbGrid__handle;
}

#ifdef __cplusplus
}
#endif
