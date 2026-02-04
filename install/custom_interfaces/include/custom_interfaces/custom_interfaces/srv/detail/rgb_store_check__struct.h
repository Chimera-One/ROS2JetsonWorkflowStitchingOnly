// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_interfaces:srv/RGBStoreCheck.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__SRV__DETAIL__RGB_STORE_CHECK__STRUCT_H_
#define CUSTOM_INTERFACES__SRV__DETAIL__RGB_STORE_CHECK__STRUCT_H_

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

/// Struct defined in srv/RGBStoreCheck in the package custom_interfaces.
typedef struct custom_interfaces__srv__RGBStoreCheck_Request
{
  sensor_msgs__msg__Image image;
} custom_interfaces__srv__RGBStoreCheck_Request;

// Struct for a sequence of custom_interfaces__srv__RGBStoreCheck_Request.
typedef struct custom_interfaces__srv__RGBStoreCheck_Request__Sequence
{
  custom_interfaces__srv__RGBStoreCheck_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__srv__RGBStoreCheck_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/RGBStoreCheck in the package custom_interfaces.
typedef struct custom_interfaces__srv__RGBStoreCheck_Response
{
  bool success;
  rosidl_runtime_c__String message;
  int64_t numberofimagesreceived;
} custom_interfaces__srv__RGBStoreCheck_Response;

// Struct for a sequence of custom_interfaces__srv__RGBStoreCheck_Response.
typedef struct custom_interfaces__srv__RGBStoreCheck_Response__Sequence
{
  custom_interfaces__srv__RGBStoreCheck_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__srv__RGBStoreCheck_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACES__SRV__DETAIL__RGB_STORE_CHECK__STRUCT_H_
