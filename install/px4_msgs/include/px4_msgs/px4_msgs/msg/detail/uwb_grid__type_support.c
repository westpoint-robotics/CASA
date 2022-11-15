// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from px4_msgs:msg/UwbGrid.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "px4_msgs/msg/detail/uwb_grid__rosidl_typesupport_introspection_c.h"
#include "px4_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "px4_msgs/msg/detail/uwb_grid__functions.h"
#include "px4_msgs/msg/detail/uwb_grid__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__UwbGrid_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  px4_msgs__msg__UwbGrid__init(message_memory);
}

void px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__UwbGrid_fini_function(void * message_memory)
{
  px4_msgs__msg__UwbGrid__fini(message_memory);
}

size_t px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__size_function__UwbGrid__target_gps(
  const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__target_gps(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__target_gps(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__fetch_function__UwbGrid__target_gps(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__target_gps(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__assign_function__UwbGrid__target_gps(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__target_gps(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__size_function__UwbGrid__target_pos(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__target_pos(
  const void * untyped_member, size_t index)
{
  const int16_t * member =
    (const int16_t *)(untyped_member);
  return &member[index];
}

void * px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__target_pos(
  void * untyped_member, size_t index)
{
  int16_t * member =
    (int16_t *)(untyped_member);
  return &member[index];
}

void px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__fetch_function__UwbGrid__target_pos(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int16_t * item =
    ((const int16_t *)
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__target_pos(untyped_member, index));
  int16_t * value =
    (int16_t *)(untyped_value);
  *value = *item;
}

void px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__assign_function__UwbGrid__target_pos(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int16_t * item =
    ((int16_t *)
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__target_pos(untyped_member, index));
  const int16_t * value =
    (const int16_t *)(untyped_value);
  *item = *value;
}

size_t px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__size_function__UwbGrid__anchor_pos_0(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__anchor_pos_0(
  const void * untyped_member, size_t index)
{
  const int16_t * member =
    (const int16_t *)(untyped_member);
  return &member[index];
}

void * px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__anchor_pos_0(
  void * untyped_member, size_t index)
{
  int16_t * member =
    (int16_t *)(untyped_member);
  return &member[index];
}

void px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__fetch_function__UwbGrid__anchor_pos_0(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int16_t * item =
    ((const int16_t *)
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__anchor_pos_0(untyped_member, index));
  int16_t * value =
    (int16_t *)(untyped_value);
  *value = *item;
}

void px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__assign_function__UwbGrid__anchor_pos_0(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int16_t * item =
    ((int16_t *)
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__anchor_pos_0(untyped_member, index));
  const int16_t * value =
    (const int16_t *)(untyped_value);
  *item = *value;
}

size_t px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__size_function__UwbGrid__anchor_pos_1(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__anchor_pos_1(
  const void * untyped_member, size_t index)
{
  const int16_t * member =
    (const int16_t *)(untyped_member);
  return &member[index];
}

void * px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__anchor_pos_1(
  void * untyped_member, size_t index)
{
  int16_t * member =
    (int16_t *)(untyped_member);
  return &member[index];
}

void px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__fetch_function__UwbGrid__anchor_pos_1(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int16_t * item =
    ((const int16_t *)
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__anchor_pos_1(untyped_member, index));
  int16_t * value =
    (int16_t *)(untyped_value);
  *value = *item;
}

void px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__assign_function__UwbGrid__anchor_pos_1(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int16_t * item =
    ((int16_t *)
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__anchor_pos_1(untyped_member, index));
  const int16_t * value =
    (const int16_t *)(untyped_value);
  *item = *value;
}

size_t px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__size_function__UwbGrid__anchor_pos_2(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__anchor_pos_2(
  const void * untyped_member, size_t index)
{
  const int16_t * member =
    (const int16_t *)(untyped_member);
  return &member[index];
}

void * px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__anchor_pos_2(
  void * untyped_member, size_t index)
{
  int16_t * member =
    (int16_t *)(untyped_member);
  return &member[index];
}

void px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__fetch_function__UwbGrid__anchor_pos_2(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int16_t * item =
    ((const int16_t *)
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__anchor_pos_2(untyped_member, index));
  int16_t * value =
    (int16_t *)(untyped_value);
  *value = *item;
}

void px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__assign_function__UwbGrid__anchor_pos_2(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int16_t * item =
    ((int16_t *)
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__anchor_pos_2(untyped_member, index));
  const int16_t * value =
    (const int16_t *)(untyped_value);
  *item = *value;
}

size_t px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__size_function__UwbGrid__anchor_pos_3(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__anchor_pos_3(
  const void * untyped_member, size_t index)
{
  const int16_t * member =
    (const int16_t *)(untyped_member);
  return &member[index];
}

void * px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__anchor_pos_3(
  void * untyped_member, size_t index)
{
  int16_t * member =
    (int16_t *)(untyped_member);
  return &member[index];
}

void px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__fetch_function__UwbGrid__anchor_pos_3(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int16_t * item =
    ((const int16_t *)
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__anchor_pos_3(untyped_member, index));
  int16_t * value =
    (int16_t *)(untyped_value);
  *value = *item;
}

void px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__assign_function__UwbGrid__anchor_pos_3(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int16_t * item =
    ((int16_t *)
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__anchor_pos_3(untyped_member, index));
  const int16_t * value =
    (const int16_t *)(untyped_value);
  *item = *value;
}

size_t px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__size_function__UwbGrid__anchor_pos_4(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__anchor_pos_4(
  const void * untyped_member, size_t index)
{
  const int16_t * member =
    (const int16_t *)(untyped_member);
  return &member[index];
}

void * px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__anchor_pos_4(
  void * untyped_member, size_t index)
{
  int16_t * member =
    (int16_t *)(untyped_member);
  return &member[index];
}

void px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__fetch_function__UwbGrid__anchor_pos_4(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int16_t * item =
    ((const int16_t *)
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__anchor_pos_4(untyped_member, index));
  int16_t * value =
    (int16_t *)(untyped_value);
  *value = *item;
}

void px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__assign_function__UwbGrid__anchor_pos_4(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int16_t * item =
    ((int16_t *)
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__anchor_pos_4(untyped_member, index));
  const int16_t * value =
    (const int16_t *)(untyped_value);
  *item = *value;
}

size_t px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__size_function__UwbGrid__anchor_pos_5(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__anchor_pos_5(
  const void * untyped_member, size_t index)
{
  const int16_t * member =
    (const int16_t *)(untyped_member);
  return &member[index];
}

void * px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__anchor_pos_5(
  void * untyped_member, size_t index)
{
  int16_t * member =
    (int16_t *)(untyped_member);
  return &member[index];
}

void px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__fetch_function__UwbGrid__anchor_pos_5(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int16_t * item =
    ((const int16_t *)
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__anchor_pos_5(untyped_member, index));
  int16_t * value =
    (int16_t *)(untyped_value);
  *value = *item;
}

void px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__assign_function__UwbGrid__anchor_pos_5(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int16_t * item =
    ((int16_t *)
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__anchor_pos_5(untyped_member, index));
  const int16_t * value =
    (const int16_t *)(untyped_value);
  *item = *value;
}

size_t px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__size_function__UwbGrid__anchor_pos_6(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__anchor_pos_6(
  const void * untyped_member, size_t index)
{
  const int16_t * member =
    (const int16_t *)(untyped_member);
  return &member[index];
}

void * px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__anchor_pos_6(
  void * untyped_member, size_t index)
{
  int16_t * member =
    (int16_t *)(untyped_member);
  return &member[index];
}

void px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__fetch_function__UwbGrid__anchor_pos_6(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int16_t * item =
    ((const int16_t *)
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__anchor_pos_6(untyped_member, index));
  int16_t * value =
    (int16_t *)(untyped_value);
  *value = *item;
}

void px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__assign_function__UwbGrid__anchor_pos_6(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int16_t * item =
    ((int16_t *)
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__anchor_pos_6(untyped_member, index));
  const int16_t * value =
    (const int16_t *)(untyped_value);
  *item = *value;
}

size_t px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__size_function__UwbGrid__anchor_pos_7(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__anchor_pos_7(
  const void * untyped_member, size_t index)
{
  const int16_t * member =
    (const int16_t *)(untyped_member);
  return &member[index];
}

void * px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__anchor_pos_7(
  void * untyped_member, size_t index)
{
  int16_t * member =
    (int16_t *)(untyped_member);
  return &member[index];
}

void px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__fetch_function__UwbGrid__anchor_pos_7(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int16_t * item =
    ((const int16_t *)
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__anchor_pos_7(untyped_member, index));
  int16_t * value =
    (int16_t *)(untyped_value);
  *value = *item;
}

void px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__assign_function__UwbGrid__anchor_pos_7(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int16_t * item =
    ((int16_t *)
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__anchor_pos_7(untyped_member, index));
  const int16_t * value =
    (const int16_t *)(untyped_value);
  *item = *value;
}

size_t px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__size_function__UwbGrid__anchor_pos_8(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__anchor_pos_8(
  const void * untyped_member, size_t index)
{
  const int16_t * member =
    (const int16_t *)(untyped_member);
  return &member[index];
}

void * px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__anchor_pos_8(
  void * untyped_member, size_t index)
{
  int16_t * member =
    (int16_t *)(untyped_member);
  return &member[index];
}

void px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__fetch_function__UwbGrid__anchor_pos_8(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int16_t * item =
    ((const int16_t *)
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__anchor_pos_8(untyped_member, index));
  int16_t * value =
    (int16_t *)(untyped_value);
  *value = *item;
}

void px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__assign_function__UwbGrid__anchor_pos_8(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int16_t * item =
    ((int16_t *)
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__anchor_pos_8(untyped_member, index));
  const int16_t * value =
    (const int16_t *)(untyped_value);
  *item = *value;
}

size_t px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__size_function__UwbGrid__anchor_pos_9(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__anchor_pos_9(
  const void * untyped_member, size_t index)
{
  const int16_t * member =
    (const int16_t *)(untyped_member);
  return &member[index];
}

void * px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__anchor_pos_9(
  void * untyped_member, size_t index)
{
  int16_t * member =
    (int16_t *)(untyped_member);
  return &member[index];
}

void px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__fetch_function__UwbGrid__anchor_pos_9(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int16_t * item =
    ((const int16_t *)
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__anchor_pos_9(untyped_member, index));
  int16_t * value =
    (int16_t *)(untyped_value);
  *value = *item;
}

void px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__assign_function__UwbGrid__anchor_pos_9(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int16_t * item =
    ((int16_t *)
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__anchor_pos_9(untyped_member, index));
  const int16_t * value =
    (const int16_t *)(untyped_value);
  *item = *value;
}

size_t px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__size_function__UwbGrid__anchor_pos_10(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__anchor_pos_10(
  const void * untyped_member, size_t index)
{
  const int16_t * member =
    (const int16_t *)(untyped_member);
  return &member[index];
}

void * px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__anchor_pos_10(
  void * untyped_member, size_t index)
{
  int16_t * member =
    (int16_t *)(untyped_member);
  return &member[index];
}

void px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__fetch_function__UwbGrid__anchor_pos_10(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int16_t * item =
    ((const int16_t *)
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__anchor_pos_10(untyped_member, index));
  int16_t * value =
    (int16_t *)(untyped_value);
  *value = *item;
}

void px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__assign_function__UwbGrid__anchor_pos_10(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int16_t * item =
    ((int16_t *)
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__anchor_pos_10(untyped_member, index));
  const int16_t * value =
    (const int16_t *)(untyped_value);
  *item = *value;
}

size_t px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__size_function__UwbGrid__anchor_pos_11(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__anchor_pos_11(
  const void * untyped_member, size_t index)
{
  const int16_t * member =
    (const int16_t *)(untyped_member);
  return &member[index];
}

void * px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__anchor_pos_11(
  void * untyped_member, size_t index)
{
  int16_t * member =
    (int16_t *)(untyped_member);
  return &member[index];
}

void px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__fetch_function__UwbGrid__anchor_pos_11(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int16_t * item =
    ((const int16_t *)
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__anchor_pos_11(untyped_member, index));
  int16_t * value =
    (int16_t *)(untyped_value);
  *value = *item;
}

void px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__assign_function__UwbGrid__anchor_pos_11(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int16_t * item =
    ((int16_t *)
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__anchor_pos_11(untyped_member, index));
  const int16_t * value =
    (const int16_t *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__UwbGrid_message_member_array[17] = {
  {
    "timestamp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(px4_msgs__msg__UwbGrid, timestamp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "initator_time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(px4_msgs__msg__UwbGrid, initator_time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "num_anchors",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(px4_msgs__msg__UwbGrid, num_anchors),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "target_gps",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(px4_msgs__msg__UwbGrid, target_gps),  // bytes offset in struct
    NULL,  // default value
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__size_function__UwbGrid__target_gps,  // size() function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__target_gps,  // get_const(index) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__target_gps,  // get(index) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__fetch_function__UwbGrid__target_gps,  // fetch(index, &value) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__assign_function__UwbGrid__target_gps,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "target_pos",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(px4_msgs__msg__UwbGrid, target_pos),  // bytes offset in struct
    NULL,  // default value
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__size_function__UwbGrid__target_pos,  // size() function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__target_pos,  // get_const(index) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__target_pos,  // get(index) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__fetch_function__UwbGrid__target_pos,  // fetch(index, &value) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__assign_function__UwbGrid__target_pos,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "anchor_pos_0",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(px4_msgs__msg__UwbGrid, anchor_pos_0),  // bytes offset in struct
    NULL,  // default value
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__size_function__UwbGrid__anchor_pos_0,  // size() function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__anchor_pos_0,  // get_const(index) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__anchor_pos_0,  // get(index) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__fetch_function__UwbGrid__anchor_pos_0,  // fetch(index, &value) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__assign_function__UwbGrid__anchor_pos_0,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "anchor_pos_1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(px4_msgs__msg__UwbGrid, anchor_pos_1),  // bytes offset in struct
    NULL,  // default value
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__size_function__UwbGrid__anchor_pos_1,  // size() function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__anchor_pos_1,  // get_const(index) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__anchor_pos_1,  // get(index) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__fetch_function__UwbGrid__anchor_pos_1,  // fetch(index, &value) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__assign_function__UwbGrid__anchor_pos_1,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "anchor_pos_2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(px4_msgs__msg__UwbGrid, anchor_pos_2),  // bytes offset in struct
    NULL,  // default value
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__size_function__UwbGrid__anchor_pos_2,  // size() function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__anchor_pos_2,  // get_const(index) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__anchor_pos_2,  // get(index) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__fetch_function__UwbGrid__anchor_pos_2,  // fetch(index, &value) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__assign_function__UwbGrid__anchor_pos_2,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "anchor_pos_3",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(px4_msgs__msg__UwbGrid, anchor_pos_3),  // bytes offset in struct
    NULL,  // default value
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__size_function__UwbGrid__anchor_pos_3,  // size() function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__anchor_pos_3,  // get_const(index) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__anchor_pos_3,  // get(index) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__fetch_function__UwbGrid__anchor_pos_3,  // fetch(index, &value) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__assign_function__UwbGrid__anchor_pos_3,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "anchor_pos_4",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(px4_msgs__msg__UwbGrid, anchor_pos_4),  // bytes offset in struct
    NULL,  // default value
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__size_function__UwbGrid__anchor_pos_4,  // size() function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__anchor_pos_4,  // get_const(index) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__anchor_pos_4,  // get(index) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__fetch_function__UwbGrid__anchor_pos_4,  // fetch(index, &value) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__assign_function__UwbGrid__anchor_pos_4,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "anchor_pos_5",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(px4_msgs__msg__UwbGrid, anchor_pos_5),  // bytes offset in struct
    NULL,  // default value
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__size_function__UwbGrid__anchor_pos_5,  // size() function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__anchor_pos_5,  // get_const(index) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__anchor_pos_5,  // get(index) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__fetch_function__UwbGrid__anchor_pos_5,  // fetch(index, &value) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__assign_function__UwbGrid__anchor_pos_5,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "anchor_pos_6",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(px4_msgs__msg__UwbGrid, anchor_pos_6),  // bytes offset in struct
    NULL,  // default value
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__size_function__UwbGrid__anchor_pos_6,  // size() function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__anchor_pos_6,  // get_const(index) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__anchor_pos_6,  // get(index) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__fetch_function__UwbGrid__anchor_pos_6,  // fetch(index, &value) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__assign_function__UwbGrid__anchor_pos_6,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "anchor_pos_7",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(px4_msgs__msg__UwbGrid, anchor_pos_7),  // bytes offset in struct
    NULL,  // default value
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__size_function__UwbGrid__anchor_pos_7,  // size() function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__anchor_pos_7,  // get_const(index) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__anchor_pos_7,  // get(index) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__fetch_function__UwbGrid__anchor_pos_7,  // fetch(index, &value) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__assign_function__UwbGrid__anchor_pos_7,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "anchor_pos_8",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(px4_msgs__msg__UwbGrid, anchor_pos_8),  // bytes offset in struct
    NULL,  // default value
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__size_function__UwbGrid__anchor_pos_8,  // size() function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__anchor_pos_8,  // get_const(index) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__anchor_pos_8,  // get(index) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__fetch_function__UwbGrid__anchor_pos_8,  // fetch(index, &value) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__assign_function__UwbGrid__anchor_pos_8,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "anchor_pos_9",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(px4_msgs__msg__UwbGrid, anchor_pos_9),  // bytes offset in struct
    NULL,  // default value
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__size_function__UwbGrid__anchor_pos_9,  // size() function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__anchor_pos_9,  // get_const(index) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__anchor_pos_9,  // get(index) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__fetch_function__UwbGrid__anchor_pos_9,  // fetch(index, &value) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__assign_function__UwbGrid__anchor_pos_9,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "anchor_pos_10",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(px4_msgs__msg__UwbGrid, anchor_pos_10),  // bytes offset in struct
    NULL,  // default value
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__size_function__UwbGrid__anchor_pos_10,  // size() function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__anchor_pos_10,  // get_const(index) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__anchor_pos_10,  // get(index) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__fetch_function__UwbGrid__anchor_pos_10,  // fetch(index, &value) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__assign_function__UwbGrid__anchor_pos_10,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "anchor_pos_11",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(px4_msgs__msg__UwbGrid, anchor_pos_11),  // bytes offset in struct
    NULL,  // default value
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__size_function__UwbGrid__anchor_pos_11,  // size() function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_const_function__UwbGrid__anchor_pos_11,  // get_const(index) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__get_function__UwbGrid__anchor_pos_11,  // get(index) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__fetch_function__UwbGrid__anchor_pos_11,  // fetch(index, &value) function pointer
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__assign_function__UwbGrid__anchor_pos_11,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__UwbGrid_message_members = {
  "px4_msgs__msg",  // message namespace
  "UwbGrid",  // message name
  17,  // number of fields
  sizeof(px4_msgs__msg__UwbGrid),
  px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__UwbGrid_message_member_array,  // message members
  px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__UwbGrid_init_function,  // function to initialize message memory (memory has to be allocated)
  px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__UwbGrid_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__UwbGrid_message_type_support_handle = {
  0,
  &px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__UwbGrid_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_px4_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, px4_msgs, msg, UwbGrid)() {
  if (!px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__UwbGrid_message_type_support_handle.typesupport_identifier) {
    px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__UwbGrid_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &px4_msgs__msg__UwbGrid__rosidl_typesupport_introspection_c__UwbGrid_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
