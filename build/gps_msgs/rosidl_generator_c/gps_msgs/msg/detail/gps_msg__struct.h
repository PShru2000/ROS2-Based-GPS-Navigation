// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from gps_msgs:msg/GpsMsg.idl
// generated code does not contain a copyright notice

#ifndef GPS_MSGS__MSG__DETAIL__GPS_MSG__STRUCT_H_
#define GPS_MSGS__MSG__DETAIL__GPS_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'zone'
// Member 'letter'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/GpsMsg in the package gps_msgs.
typedef struct gps_msgs__msg__GpsMsg
{
  std_msgs__msg__Header header;
  double latitude;
  double longitude;
  double altitude;
  double hdop;
  double utm_easting;
  double utm_northing;
  double utc;
  rosidl_runtime_c__String zone;
  rosidl_runtime_c__String letter;
} gps_msgs__msg__GpsMsg;

// Struct for a sequence of gps_msgs__msg__GpsMsg.
typedef struct gps_msgs__msg__GpsMsg__Sequence
{
  gps_msgs__msg__GpsMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} gps_msgs__msg__GpsMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // GPS_MSGS__MSG__DETAIL__GPS_MSG__STRUCT_H_
