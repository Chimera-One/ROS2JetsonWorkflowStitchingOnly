// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom_interfaces:msg/RGB.idl
// generated code does not contain a copyright notice
#include "custom_interfaces/msg/detail/rgb__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `name`
#include "rosidl_runtime_c/string_functions.h"
// Member `image`
#include "sensor_msgs/msg/detail/image__functions.h"

bool
custom_interfaces__msg__RGB__init(custom_interfaces__msg__RGB * msg)
{
  if (!msg) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__init(&msg->name)) {
    custom_interfaces__msg__RGB__fini(msg);
    return false;
  }
  // gps_latitude
  // gps_longitude
  // gps_altitude
  // gps_signal_level
  // compass
  // roll
  // pitch
  // yaw
  // gimbal_roll
  // gimbal_pitch
  // gimbal_yaw
  // image
  if (!sensor_msgs__msg__Image__init(&msg->image)) {
    custom_interfaces__msg__RGB__fini(msg);
    return false;
  }
  return true;
}

void
custom_interfaces__msg__RGB__fini(custom_interfaces__msg__RGB * msg)
{
  if (!msg) {
    return;
  }
  // name
  rosidl_runtime_c__String__fini(&msg->name);
  // gps_latitude
  // gps_longitude
  // gps_altitude
  // gps_signal_level
  // compass
  // roll
  // pitch
  // yaw
  // gimbal_roll
  // gimbal_pitch
  // gimbal_yaw
  // image
  sensor_msgs__msg__Image__fini(&msg->image);
}

bool
custom_interfaces__msg__RGB__are_equal(const custom_interfaces__msg__RGB * lhs, const custom_interfaces__msg__RGB * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->name), &(rhs->name)))
  {
    return false;
  }
  // gps_latitude
  if (lhs->gps_latitude != rhs->gps_latitude) {
    return false;
  }
  // gps_longitude
  if (lhs->gps_longitude != rhs->gps_longitude) {
    return false;
  }
  // gps_altitude
  if (lhs->gps_altitude != rhs->gps_altitude) {
    return false;
  }
  // gps_signal_level
  if (lhs->gps_signal_level != rhs->gps_signal_level) {
    return false;
  }
  // compass
  if (lhs->compass != rhs->compass) {
    return false;
  }
  // roll
  if (lhs->roll != rhs->roll) {
    return false;
  }
  // pitch
  if (lhs->pitch != rhs->pitch) {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  // gimbal_roll
  if (lhs->gimbal_roll != rhs->gimbal_roll) {
    return false;
  }
  // gimbal_pitch
  if (lhs->gimbal_pitch != rhs->gimbal_pitch) {
    return false;
  }
  // gimbal_yaw
  if (lhs->gimbal_yaw != rhs->gimbal_yaw) {
    return false;
  }
  // image
  if (!sensor_msgs__msg__Image__are_equal(
      &(lhs->image), &(rhs->image)))
  {
    return false;
  }
  return true;
}

bool
custom_interfaces__msg__RGB__copy(
  const custom_interfaces__msg__RGB * input,
  custom_interfaces__msg__RGB * output)
{
  if (!input || !output) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__copy(
      &(input->name), &(output->name)))
  {
    return false;
  }
  // gps_latitude
  output->gps_latitude = input->gps_latitude;
  // gps_longitude
  output->gps_longitude = input->gps_longitude;
  // gps_altitude
  output->gps_altitude = input->gps_altitude;
  // gps_signal_level
  output->gps_signal_level = input->gps_signal_level;
  // compass
  output->compass = input->compass;
  // roll
  output->roll = input->roll;
  // pitch
  output->pitch = input->pitch;
  // yaw
  output->yaw = input->yaw;
  // gimbal_roll
  output->gimbal_roll = input->gimbal_roll;
  // gimbal_pitch
  output->gimbal_pitch = input->gimbal_pitch;
  // gimbal_yaw
  output->gimbal_yaw = input->gimbal_yaw;
  // image
  if (!sensor_msgs__msg__Image__copy(
      &(input->image), &(output->image)))
  {
    return false;
  }
  return true;
}

custom_interfaces__msg__RGB *
custom_interfaces__msg__RGB__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interfaces__msg__RGB * msg = (custom_interfaces__msg__RGB *)allocator.allocate(sizeof(custom_interfaces__msg__RGB), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom_interfaces__msg__RGB));
  bool success = custom_interfaces__msg__RGB__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
custom_interfaces__msg__RGB__destroy(custom_interfaces__msg__RGB * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    custom_interfaces__msg__RGB__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
custom_interfaces__msg__RGB__Sequence__init(custom_interfaces__msg__RGB__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interfaces__msg__RGB * data = NULL;

  if (size) {
    data = (custom_interfaces__msg__RGB *)allocator.zero_allocate(size, sizeof(custom_interfaces__msg__RGB), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom_interfaces__msg__RGB__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom_interfaces__msg__RGB__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
custom_interfaces__msg__RGB__Sequence__fini(custom_interfaces__msg__RGB__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      custom_interfaces__msg__RGB__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

custom_interfaces__msg__RGB__Sequence *
custom_interfaces__msg__RGB__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interfaces__msg__RGB__Sequence * array = (custom_interfaces__msg__RGB__Sequence *)allocator.allocate(sizeof(custom_interfaces__msg__RGB__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = custom_interfaces__msg__RGB__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
custom_interfaces__msg__RGB__Sequence__destroy(custom_interfaces__msg__RGB__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    custom_interfaces__msg__RGB__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
custom_interfaces__msg__RGB__Sequence__are_equal(const custom_interfaces__msg__RGB__Sequence * lhs, const custom_interfaces__msg__RGB__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom_interfaces__msg__RGB__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom_interfaces__msg__RGB__Sequence__copy(
  const custom_interfaces__msg__RGB__Sequence * input,
  custom_interfaces__msg__RGB__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom_interfaces__msg__RGB);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    custom_interfaces__msg__RGB * data =
      (custom_interfaces__msg__RGB *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom_interfaces__msg__RGB__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          custom_interfaces__msg__RGB__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!custom_interfaces__msg__RGB__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
