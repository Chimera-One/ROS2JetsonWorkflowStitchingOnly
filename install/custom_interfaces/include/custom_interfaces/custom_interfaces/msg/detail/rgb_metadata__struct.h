// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_interfaces:msg/RGBMetadata.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__RGB_METADATA__STRUCT_H_
#define CUSTOM_INTERFACES__MSG__DETAIL__RGB_METADATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/RGBMetadata in the package custom_interfaces.
/**
  * primitives
 */
typedef struct custom_interfaces__msg__RGBMetadata
{
  rosidl_runtime_c__String name;
  double gps_latitude;
  double gps_longitude;
  double gps_altitude;
  int32_t gps_signal_level;
  double compass;
  double roll;
  double pitch;
  double yaw;
  double gimbal_roll;
  double gimbal_pitch;
  double gimbal_yaw;
} custom_interfaces__msg__RGBMetadata;

// Struct for a sequence of custom_interfaces__msg__RGBMetadata.
typedef struct custom_interfaces__msg__RGBMetadata__Sequence
{
  custom_interfaces__msg__RGBMetadata * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__msg__RGBMetadata__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__RGB_METADATA__STRUCT_H_
