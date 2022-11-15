// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from px4_msgs:msg/UwbDistance.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "px4_msgs/msg/detail/uwb_distance__rosidl_typesupport_introspection_c.h"
#include "px4_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "px4_msgs/msg/detail/uwb_distance__functions.h"
#include "px4_msgs/msg/detail/uwb_distance__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__UwbDistance_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  px4_msgs__msg__UwbDistance__init(message_memory);
}

void px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__UwbDistance_fini_function(void * message_memory)
{
  px4_msgs__msg__UwbDistance__fini(message_memory);
}

size_t px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__size_function__UwbDistance__anchor_distance(
  const void * untyped_member)
{
  (void)untyped_member;
  return 12;
}

const void * px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__get_const_function__UwbDistance__anchor_distance(
  const void * untyped_member, size_t index)
{
  const uint16_t * member =
    (const uint16_t *)(untyped_member);
  return &member[index];
}

void * px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__get_function__UwbDistance__anchor_distance(
  void * untyped_member, size_t index)
{
  uint16_t * member =
    (uint16_t *)(untyped_member);
  return &member[index];
}

void px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__fetch_function__UwbDistance__anchor_distance(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint16_t * item =
    ((const uint16_t *)
    px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__get_const_function__UwbDistance__anchor_distance(untyped_member, index));
  uint16_t * value =
    (uint16_t *)(untyped_value);
  *value = *item;
}

void px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__assign_function__UwbDistance__anchor_distance(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint16_t * item =
    ((uint16_t *)
    px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__get_function__UwbDistance__anchor_distance(untyped_member, index));
  const uint16_t * value =
    (const uint16_t *)(untyped_value);
  *item = *value;
}

size_t px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__size_function__UwbDistance__nlos(
  const void * untyped_member)
{
  (void)untyped_member;
  return 12;
}

const void * px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__get_const_function__UwbDistance__nlos(
  const void * untyped_member, size_t index)
{
  const bool * member =
    (const bool *)(untyped_member);
  return &member[index];
}

void * px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__get_function__UwbDistance__nlos(
  void * untyped_member, size_t index)
{
  bool * member =
    (bool *)(untyped_member);
  return &member[index];
}

void px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__fetch_function__UwbDistance__nlos(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const bool * item =
    ((const bool *)
    px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__get_const_function__UwbDistance__nlos(untyped_member, index));
  bool * value =
    (bool *)(untyped_value);
  *value = *item;
}

void px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__assign_function__UwbDistance__nlos(
  void * untyped_member, size_t index, const void * untyped_value)
{
  bool * item =
    ((bool *)
    px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__get_function__UwbDistance__nlos(untyped_member, index));
  const bool * value =
    (const bool *)(untyped_value);
  *item = *value;
}

size_t px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__size_function__UwbDistance__aoafirst(
  const void * untyped_member)
{
  (void)untyped_member;
  return 12;
}

const void * px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__get_const_function__UwbDistance__aoafirst(
  const void * untyped_member, size_t index)
{
  const float * member =
    (const float *)(untyped_member);
  return &member[index];
}

void * px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__get_function__UwbDistance__aoafirst(
  void * untyped_member, size_t index)
{
  float * member =
    (float *)(untyped_member);
  return &member[index];
}

void px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__fetch_function__UwbDistance__aoafirst(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__get_const_function__UwbDistance__aoafirst(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__assign_function__UwbDistance__aoafirst(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__get_function__UwbDistance__aoafirst(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

size_t px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__size_function__UwbDistance__position(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__get_const_function__UwbDistance__position(
  const void * untyped_member, size_t index)
{
  const float * member =
    (const float *)(untyped_member);
  return &member[index];
}

void * px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__get_function__UwbDistance__position(
  void * untyped_member, size_t index)
{
  float * member =
    (float *)(untyped_member);
  return &member[index];
}

void px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__fetch_function__UwbDistance__position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__get_const_function__UwbDistance__position(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__assign_function__UwbDistance__position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__get_function__UwbDistance__position(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__UwbDistance_message_member_array[10] = {
  {
    "timestamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(px4_msgs__msg__UwbDistance, timestamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "time_uwb_ms",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(px4_msgs__msg__UwbDistance, time_uwb_ms),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "counter",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(px4_msgs__msg__UwbDistance, counter),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "sessionid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(px4_msgs__msg__UwbDistance, sessionid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "time_offset",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(px4_msgs__msg__UwbDistance, time_offset),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(px4_msgs__msg__UwbDistance, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "anchor_distance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    12,  // array size
    false,  // is upper bound
    offsetof(px4_msgs__msg__UwbDistance, anchor_distance),  // bytes offset in struct
    NULL,  // default value
    px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__size_function__UwbDistance__anchor_distance,  // size() function pointer
    px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__get_const_function__UwbDistance__anchor_distance,  // get_const(index) function pointer
    px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__get_function__UwbDistance__anchor_distance,  // get(index) function pointer
    px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__fetch_function__UwbDistance__anchor_distance,  // fetch(index, &value) function pointer
    px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__assign_function__UwbDistance__anchor_distance,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "nlos",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    12,  // array size
    false,  // is upper bound
    offsetof(px4_msgs__msg__UwbDistance, nlos),  // bytes offset in struct
    NULL,  // default value
    px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__size_function__UwbDistance__nlos,  // size() function pointer
    px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__get_const_function__UwbDistance__nlos,  // get_const(index) function pointer
    px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__get_function__UwbDistance__nlos,  // get(index) function pointer
    px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__fetch_function__UwbDistance__nlos,  // fetch(index, &value) function pointer
    px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__assign_function__UwbDistance__nlos,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "aoafirst",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    12,  // array size
    false,  // is upper bound
    offsetof(px4_msgs__msg__UwbDistance, aoafirst),  // bytes offset in struct
    NULL,  // default value
    px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__size_function__UwbDistance__aoafirst,  // size() function pointer
    px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__get_const_function__UwbDistance__aoafirst,  // get_const(index) function pointer
    px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__get_function__UwbDistance__aoafirst,  // get(index) function pointer
    px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__fetch_function__UwbDistance__aoafirst,  // fetch(index, &value) function pointer
    px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__assign_function__UwbDistance__aoafirst,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(px4_msgs__msg__UwbDistance, position),  // bytes offset in struct
    NULL,  // default value
    px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__size_function__UwbDistance__position,  // size() function pointer
    px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__get_const_function__UwbDistance__position,  // get_const(index) function pointer
    px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__get_function__UwbDistance__position,  // get(index) function pointer
    px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__fetch_function__UwbDistance__position,  // fetch(index, &value) function pointer
    px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__assign_function__UwbDistance__position,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__UwbDistance_message_members = {
  "px4_msgs__msg",  // message namespace
  "UwbDistance",  // message name
  10,  // number of fields
  sizeof(px4_msgs__msg__UwbDistance),
  px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__UwbDistance_message_member_array,  // message members
  px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__UwbDistance_init_function,  // function to initialize message memory (memory has to be allocated)
  px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__UwbDistance_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__UwbDistance_message_type_support_handle = {
  0,
  &px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__UwbDistance_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_px4_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, px4_msgs, msg, UwbDistance)() {
  if (!px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__UwbDistance_message_type_support_handle.typesupport_identifier) {
    px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__UwbDistance_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &px4_msgs__msg__UwbDistance__rosidl_typesupport_introspection_c__UwbDistance_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
