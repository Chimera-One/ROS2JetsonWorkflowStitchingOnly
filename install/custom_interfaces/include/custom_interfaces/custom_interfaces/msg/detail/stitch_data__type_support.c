// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from custom_interfaces:msg/StitchData.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "custom_interfaces/msg/detail/stitch_data__rosidl_typesupport_introspection_c.h"
#include "custom_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "custom_interfaces/msg/detail/stitch_data__functions.h"
#include "custom_interfaces/msg/detail/stitch_data__struct.h"


// Include directives for member types
// Member `data`
// Member `names`
#include "rosidl_runtime_c/string_functions.h"
// Member `gps_latitude`
// Member `gps_longitude`
// Member `gps_altitude`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__StitchData_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  custom_interfaces__msg__StitchData__init(message_memory);
}

void custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__StitchData_fini_function(void * message_memory)
{
  custom_interfaces__msg__StitchData__fini(message_memory);
}

size_t custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__size_function__StitchData__names(
  const void * untyped_member)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return member->size;
}

const void * custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__get_const_function__StitchData__names(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String__Sequence * member =
    (const rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void * custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__get_function__StitchData__names(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  return &member->data[index];
}

void custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__fetch_function__StitchData__names(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__get_const_function__StitchData__names(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__assign_function__StitchData__names(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__get_function__StitchData__names(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

bool custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__resize_function__StitchData__names(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__String__Sequence * member =
    (rosidl_runtime_c__String__Sequence *)(untyped_member);
  rosidl_runtime_c__String__Sequence__fini(member);
  return rosidl_runtime_c__String__Sequence__init(member, size);
}

size_t custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__size_function__StitchData__gps_latitude(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__get_const_function__StitchData__gps_latitude(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__get_function__StitchData__gps_latitude(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__fetch_function__StitchData__gps_latitude(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__get_const_function__StitchData__gps_latitude(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__assign_function__StitchData__gps_latitude(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__get_function__StitchData__gps_latitude(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__resize_function__StitchData__gps_latitude(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__size_function__StitchData__gps_longitude(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__get_const_function__StitchData__gps_longitude(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__get_function__StitchData__gps_longitude(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__fetch_function__StitchData__gps_longitude(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__get_const_function__StitchData__gps_longitude(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__assign_function__StitchData__gps_longitude(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__get_function__StitchData__gps_longitude(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__resize_function__StitchData__gps_longitude(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__size_function__StitchData__gps_altitude(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__get_const_function__StitchData__gps_altitude(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__get_function__StitchData__gps_altitude(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__fetch_function__StitchData__gps_altitude(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__get_const_function__StitchData__gps_altitude(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__assign_function__StitchData__gps_altitude(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__get_function__StitchData__gps_altitude(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__resize_function__StitchData__gps_altitude(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__StitchData_message_member_array[5] = {
  {
    "data",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__msg__StitchData, data),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "names",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__msg__StitchData, names),  // bytes offset in struct
    NULL,  // default value
    custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__size_function__StitchData__names,  // size() function pointer
    custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__get_const_function__StitchData__names,  // get_const(index) function pointer
    custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__get_function__StitchData__names,  // get(index) function pointer
    custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__fetch_function__StitchData__names,  // fetch(index, &value) function pointer
    custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__assign_function__StitchData__names,  // assign(index, value) function pointer
    custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__resize_function__StitchData__names  // resize(index) function pointer
  },
  {
    "gps_latitude",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__msg__StitchData, gps_latitude),  // bytes offset in struct
    NULL,  // default value
    custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__size_function__StitchData__gps_latitude,  // size() function pointer
    custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__get_const_function__StitchData__gps_latitude,  // get_const(index) function pointer
    custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__get_function__StitchData__gps_latitude,  // get(index) function pointer
    custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__fetch_function__StitchData__gps_latitude,  // fetch(index, &value) function pointer
    custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__assign_function__StitchData__gps_latitude,  // assign(index, value) function pointer
    custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__resize_function__StitchData__gps_latitude  // resize(index) function pointer
  },
  {
    "gps_longitude",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__msg__StitchData, gps_longitude),  // bytes offset in struct
    NULL,  // default value
    custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__size_function__StitchData__gps_longitude,  // size() function pointer
    custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__get_const_function__StitchData__gps_longitude,  // get_const(index) function pointer
    custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__get_function__StitchData__gps_longitude,  // get(index) function pointer
    custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__fetch_function__StitchData__gps_longitude,  // fetch(index, &value) function pointer
    custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__assign_function__StitchData__gps_longitude,  // assign(index, value) function pointer
    custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__resize_function__StitchData__gps_longitude  // resize(index) function pointer
  },
  {
    "gps_altitude",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__msg__StitchData, gps_altitude),  // bytes offset in struct
    NULL,  // default value
    custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__size_function__StitchData__gps_altitude,  // size() function pointer
    custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__get_const_function__StitchData__gps_altitude,  // get_const(index) function pointer
    custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__get_function__StitchData__gps_altitude,  // get(index) function pointer
    custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__fetch_function__StitchData__gps_altitude,  // fetch(index, &value) function pointer
    custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__assign_function__StitchData__gps_altitude,  // assign(index, value) function pointer
    custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__resize_function__StitchData__gps_altitude  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__StitchData_message_members = {
  "custom_interfaces__msg",  // message namespace
  "StitchData",  // message name
  5,  // number of fields
  sizeof(custom_interfaces__msg__StitchData),
  custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__StitchData_message_member_array,  // message members
  custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__StitchData_init_function,  // function to initialize message memory (memory has to be allocated)
  custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__StitchData_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__StitchData_message_type_support_handle = {
  0,
  &custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__StitchData_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, msg, StitchData)() {
  if (!custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__StitchData_message_type_support_handle.typesupport_identifier) {
    custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__StitchData_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &custom_interfaces__msg__StitchData__rosidl_typesupport_introspection_c__StitchData_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
