// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_interfaces:msg/StitchData.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__STITCH_DATA__STRUCT_H_
#define CUSTOM_INTERFACES__MSG__DETAIL__STITCH_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'data'
// Member 'name'
#include "rosidl_runtime_c/string.h"
// Member 'gps_latitude'
// Member 'gps_longitude'
// Member 'gps_altitude'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/StitchData in the package custom_interfaces.
typedef struct custom_interfaces__msg__StitchData
{
  rosidl_runtime_c__String data;
  rosidl_runtime_c__String__Sequence name;
  rosidl_runtime_c__double__Sequence gps_latitude;
  rosidl_runtime_c__double__Sequence gps_longitude;
  rosidl_runtime_c__double__Sequence gps_altitude;
} custom_interfaces__msg__StitchData;

// Struct for a sequence of custom_interfaces__msg__StitchData.
typedef struct custom_interfaces__msg__StitchData__Sequence
{
  custom_interfaces__msg__StitchData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__msg__StitchData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__STITCH_DATA__STRUCT_H_
