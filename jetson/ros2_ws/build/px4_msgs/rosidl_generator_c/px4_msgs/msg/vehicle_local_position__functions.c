// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/VehicleLocalPosition.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/vehicle_local_position__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
px4_msgs__msg__VehicleLocalPosition__init(px4_msgs__msg__VehicleLocalPosition * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // timestamp_sample
  // xy_valid
  // z_valid
  // v_xy_valid
  // v_z_valid
  // x
  // y
  // z
  // delta_xy
  // xy_reset_counter
  // delta_z
  // z_reset_counter
  // vx
  // vy
  // vz
  // z_deriv
  // delta_vxy
  // vxy_reset_counter
  // delta_vz
  // vz_reset_counter
  // ax
  // ay
  // az
  // heading
  // heading_var
  // unaided_heading
  // delta_heading
  // heading_reset_counter
  // heading_good_for_control
  // tilt_var
  // xy_global
  // z_global
  // ref_timestamp
  // ref_lat
  // ref_lon
  // ref_alt
  // dist_bottom_valid
  // dist_bottom
  // dist_bottom_var
  // delta_dist_bottom
  // dist_bottom_reset_counter
  // dist_bottom_sensor_bitfield
  // eph
  // epv
  // evh
  // evv
  // dead_reckoning
  // vxy_max
  // vz_max
  // hagl_min
  // hagl_max
  return true;
}

void
px4_msgs__msg__VehicleLocalPosition__fini(px4_msgs__msg__VehicleLocalPosition * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // timestamp_sample
  // xy_valid
  // z_valid
  // v_xy_valid
  // v_z_valid
  // x
  // y
  // z
  // delta_xy
  // xy_reset_counter
  // delta_z
  // z_reset_counter
  // vx
  // vy
  // vz
  // z_deriv
  // delta_vxy
  // vxy_reset_counter
  // delta_vz
  // vz_reset_counter
  // ax
  // ay
  // az
  // heading
  // heading_var
  // unaided_heading
  // delta_heading
  // heading_reset_counter
  // heading_good_for_control
  // tilt_var
  // xy_global
  // z_global
  // ref_timestamp
  // ref_lat
  // ref_lon
  // ref_alt
  // dist_bottom_valid
  // dist_bottom
  // dist_bottom_var
  // delta_dist_bottom
  // dist_bottom_reset_counter
  // dist_bottom_sensor_bitfield
  // eph
  // epv
  // evh
  // evv
  // dead_reckoning
  // vxy_max
  // vz_max
  // hagl_min
  // hagl_max
}

px4_msgs__msg__VehicleLocalPosition *
px4_msgs__msg__VehicleLocalPosition__create()
{
  px4_msgs__msg__VehicleLocalPosition * msg = (px4_msgs__msg__VehicleLocalPosition *)malloc(sizeof(px4_msgs__msg__VehicleLocalPosition));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__VehicleLocalPosition));
  bool success = px4_msgs__msg__VehicleLocalPosition__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__VehicleLocalPosition__destroy(px4_msgs__msg__VehicleLocalPosition * msg)
{
  if (msg) {
    px4_msgs__msg__VehicleLocalPosition__fini(msg);
  }
  free(msg);
}


bool
px4_msgs__msg__VehicleLocalPosition__Sequence__init(px4_msgs__msg__VehicleLocalPosition__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  px4_msgs__msg__VehicleLocalPosition * data = NULL;
  if (size) {
    data = (px4_msgs__msg__VehicleLocalPosition *)calloc(size, sizeof(px4_msgs__msg__VehicleLocalPosition));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__VehicleLocalPosition__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__VehicleLocalPosition__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
px4_msgs__msg__VehicleLocalPosition__Sequence__fini(px4_msgs__msg__VehicleLocalPosition__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__VehicleLocalPosition__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

px4_msgs__msg__VehicleLocalPosition__Sequence *
px4_msgs__msg__VehicleLocalPosition__Sequence__create(size_t size)
{
  px4_msgs__msg__VehicleLocalPosition__Sequence * array = (px4_msgs__msg__VehicleLocalPosition__Sequence *)malloc(sizeof(px4_msgs__msg__VehicleLocalPosition__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__VehicleLocalPosition__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__VehicleLocalPosition__Sequence__destroy(px4_msgs__msg__VehicleLocalPosition__Sequence * array)
{
  if (array) {
    px4_msgs__msg__VehicleLocalPosition__Sequence__fini(array);
  }
  free(array);
}
