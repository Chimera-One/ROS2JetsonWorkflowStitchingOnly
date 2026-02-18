// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_interfaces:msg/StitchedData.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__STITCHED_DATA__STRUCT_H_
#define CUSTOM_INTERFACES__MSG__DETAIL__STITCHED_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'image'
#include "sensor_msgs/msg/detail/image__struct.h"

/// Struct defined in msg/StitchedData in the package custom_interfaces.
/**
  * primitives
 */
typedef struct custom_interfaces__msg__StitchedData
{
  double rotation_degree;
  double latitude;
  double longitude;
  double altitude;
  int64_t numberofimages;
  /// existing ROS message
  sensor_msgs__msg__Image image;
} custom_interfaces__msg__StitchedData;

// Struct for a sequence of custom_interfaces__msg__StitchedData.
typedef struct custom_interfaces__msg__StitchedData__Sequence
{
  custom_interfaces__msg__StitchedData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__msg__StitchedData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__STITCHED_DATA__STRUCT_H_
