// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_interfaces:msg/RGB.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__RGB__STRUCT_H_
#define CUSTOM_INTERFACES__MSG__DETAIL__RGB__STRUCT_H_

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
// Member 'image'
#include "sensor_msgs/msg/detail/image__struct.h"

/// Struct defined in msg/RGB in the package custom_interfaces.
/**
  * primitives
 */
typedef struct custom_interfaces__msg__RGB
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
  /// existing ROS message
  sensor_msgs__msg__Image image;
} custom_interfaces__msg__RGB;

// Struct for a sequence of custom_interfaces__msg__RGB.
typedef struct custom_interfaces__msg__RGB__Sequence
{
  custom_interfaces__msg__RGB * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__msg__RGB__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__RGB__STRUCT_H_
