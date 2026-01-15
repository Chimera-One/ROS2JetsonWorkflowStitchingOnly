// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom_interfaces:msg/StitchData.idl
// generated code does not contain a copyright notice
#include "custom_interfaces/msg/detail/stitch_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `data`
// Member `names`
#include "rosidl_runtime_c/string_functions.h"
// Member `gps_latitude`
// Member `gps_longitude`
// Member `gps_altitude`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
custom_interfaces__msg__StitchData__init(custom_interfaces__msg__StitchData * msg)
{
  if (!msg) {
    return false;
  }
  // data
  if (!rosidl_runtime_c__String__init(&msg->data)) {
    custom_interfaces__msg__StitchData__fini(msg);
    return false;
  }
  // names
  if (!rosidl_runtime_c__String__Sequence__init(&msg->names, 0)) {
    custom_interfaces__msg__StitchData__fini(msg);
    return false;
  }
  // gps_latitude
  if (!rosidl_runtime_c__double__Sequence__init(&msg->gps_latitude, 0)) {
    custom_interfaces__msg__StitchData__fini(msg);
    return false;
  }
  // gps_longitude
  if (!rosidl_runtime_c__double__Sequence__init(&msg->gps_longitude, 0)) {
    custom_interfaces__msg__StitchData__fini(msg);
    return false;
  }
  // gps_altitude
  if (!rosidl_runtime_c__double__Sequence__init(&msg->gps_altitude, 0)) {
    custom_interfaces__msg__StitchData__fini(msg);
    return false;
  }
  return true;
}

void
custom_interfaces__msg__StitchData__fini(custom_interfaces__msg__StitchData * msg)
{
  if (!msg) {
    return;
  }
  // data
  rosidl_runtime_c__String__fini(&msg->data);
  // names
  rosidl_runtime_c__String__Sequence__fini(&msg->names);
  // gps_latitude
  rosidl_runtime_c__double__Sequence__fini(&msg->gps_latitude);
  // gps_longitude
  rosidl_runtime_c__double__Sequence__fini(&msg->gps_longitude);
  // gps_altitude
  rosidl_runtime_c__double__Sequence__fini(&msg->gps_altitude);
}

bool
custom_interfaces__msg__StitchData__are_equal(const custom_interfaces__msg__StitchData * lhs, const custom_interfaces__msg__StitchData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // data
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->data), &(rhs->data)))
  {
    return false;
  }
  // names
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->names), &(rhs->names)))
  {
    return false;
  }
  // gps_latitude
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->gps_latitude), &(rhs->gps_latitude)))
  {
    return false;
  }
  // gps_longitude
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->gps_longitude), &(rhs->gps_longitude)))
  {
    return false;
  }
  // gps_altitude
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->gps_altitude), &(rhs->gps_altitude)))
  {
    return false;
  }
  return true;
}

bool
custom_interfaces__msg__StitchData__copy(
  const custom_interfaces__msg__StitchData * input,
  custom_interfaces__msg__StitchData * output)
{
  if (!input || !output) {
    return false;
  }
  // data
  if (!rosidl_runtime_c__String__copy(
      &(input->data), &(output->data)))
  {
    return false;
  }
  // names
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->names), &(output->names)))
  {
    return false;
  }
  // gps_latitude
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->gps_latitude), &(output->gps_latitude)))
  {
    return false;
  }
  // gps_longitude
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->gps_longitude), &(output->gps_longitude)))
  {
    return false;
  }
  // gps_altitude
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->gps_altitude), &(output->gps_altitude)))
  {
    return false;
  }
  return true;
}

custom_interfaces__msg__StitchData *
custom_interfaces__msg__StitchData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interfaces__msg__StitchData * msg = (custom_interfaces__msg__StitchData *)allocator.allocate(sizeof(custom_interfaces__msg__StitchData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom_interfaces__msg__StitchData));
  bool success = custom_interfaces__msg__StitchData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
custom_interfaces__msg__StitchData__destroy(custom_interfaces__msg__StitchData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    custom_interfaces__msg__StitchData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
custom_interfaces__msg__StitchData__Sequence__init(custom_interfaces__msg__StitchData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interfaces__msg__StitchData * data = NULL;

  if (size) {
    data = (custom_interfaces__msg__StitchData *)allocator.zero_allocate(size, sizeof(custom_interfaces__msg__StitchData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom_interfaces__msg__StitchData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom_interfaces__msg__StitchData__fini(&data[i - 1]);
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
custom_interfaces__msg__StitchData__Sequence__fini(custom_interfaces__msg__StitchData__Sequence * array)
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
      custom_interfaces__msg__StitchData__fini(&array->data[i]);
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

custom_interfaces__msg__StitchData__Sequence *
custom_interfaces__msg__StitchData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interfaces__msg__StitchData__Sequence * array = (custom_interfaces__msg__StitchData__Sequence *)allocator.allocate(sizeof(custom_interfaces__msg__StitchData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = custom_interfaces__msg__StitchData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
custom_interfaces__msg__StitchData__Sequence__destroy(custom_interfaces__msg__StitchData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    custom_interfaces__msg__StitchData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
custom_interfaces__msg__StitchData__Sequence__are_equal(const custom_interfaces__msg__StitchData__Sequence * lhs, const custom_interfaces__msg__StitchData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom_interfaces__msg__StitchData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom_interfaces__msg__StitchData__Sequence__copy(
  const custom_interfaces__msg__StitchData__Sequence * input,
  custom_interfaces__msg__StitchData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom_interfaces__msg__StitchData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    custom_interfaces__msg__StitchData * data =
      (custom_interfaces__msg__StitchData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom_interfaces__msg__StitchData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          custom_interfaces__msg__StitchData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!custom_interfaces__msg__StitchData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
