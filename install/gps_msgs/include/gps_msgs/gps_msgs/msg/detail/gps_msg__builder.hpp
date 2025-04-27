// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from gps_msgs:msg/GpsMsg.idl
// generated code does not contain a copyright notice

#ifndef GPS_MSGS__MSG__DETAIL__GPS_MSG__BUILDER_HPP_
#define GPS_MSGS__MSG__DETAIL__GPS_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "gps_msgs/msg/detail/gps_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace gps_msgs
{

namespace msg
{

namespace builder
{

class Init_GpsMsg_letter
{
public:
  explicit Init_GpsMsg_letter(::gps_msgs::msg::GpsMsg & msg)
  : msg_(msg)
  {}
  ::gps_msgs::msg::GpsMsg letter(::gps_msgs::msg::GpsMsg::_letter_type arg)
  {
    msg_.letter = std::move(arg);
    return std::move(msg_);
  }

private:
  ::gps_msgs::msg::GpsMsg msg_;
};

class Init_GpsMsg_zone
{
public:
  explicit Init_GpsMsg_zone(::gps_msgs::msg::GpsMsg & msg)
  : msg_(msg)
  {}
  Init_GpsMsg_letter zone(::gps_msgs::msg::GpsMsg::_zone_type arg)
  {
    msg_.zone = std::move(arg);
    return Init_GpsMsg_letter(msg_);
  }

private:
  ::gps_msgs::msg::GpsMsg msg_;
};

class Init_GpsMsg_utc
{
public:
  explicit Init_GpsMsg_utc(::gps_msgs::msg::GpsMsg & msg)
  : msg_(msg)
  {}
  Init_GpsMsg_zone utc(::gps_msgs::msg::GpsMsg::_utc_type arg)
  {
    msg_.utc = std::move(arg);
    return Init_GpsMsg_zone(msg_);
  }

private:
  ::gps_msgs::msg::GpsMsg msg_;
};

class Init_GpsMsg_utm_northing
{
public:
  explicit Init_GpsMsg_utm_northing(::gps_msgs::msg::GpsMsg & msg)
  : msg_(msg)
  {}
  Init_GpsMsg_utc utm_northing(::gps_msgs::msg::GpsMsg::_utm_northing_type arg)
  {
    msg_.utm_northing = std::move(arg);
    return Init_GpsMsg_utc(msg_);
  }

private:
  ::gps_msgs::msg::GpsMsg msg_;
};

class Init_GpsMsg_utm_easting
{
public:
  explicit Init_GpsMsg_utm_easting(::gps_msgs::msg::GpsMsg & msg)
  : msg_(msg)
  {}
  Init_GpsMsg_utm_northing utm_easting(::gps_msgs::msg::GpsMsg::_utm_easting_type arg)
  {
    msg_.utm_easting = std::move(arg);
    return Init_GpsMsg_utm_northing(msg_);
  }

private:
  ::gps_msgs::msg::GpsMsg msg_;
};

class Init_GpsMsg_hdop
{
public:
  explicit Init_GpsMsg_hdop(::gps_msgs::msg::GpsMsg & msg)
  : msg_(msg)
  {}
  Init_GpsMsg_utm_easting hdop(::gps_msgs::msg::GpsMsg::_hdop_type arg)
  {
    msg_.hdop = std::move(arg);
    return Init_GpsMsg_utm_easting(msg_);
  }

private:
  ::gps_msgs::msg::GpsMsg msg_;
};

class Init_GpsMsg_altitude
{
public:
  explicit Init_GpsMsg_altitude(::gps_msgs::msg::GpsMsg & msg)
  : msg_(msg)
  {}
  Init_GpsMsg_hdop altitude(::gps_msgs::msg::GpsMsg::_altitude_type arg)
  {
    msg_.altitude = std::move(arg);
    return Init_GpsMsg_hdop(msg_);
  }

private:
  ::gps_msgs::msg::GpsMsg msg_;
};

class Init_GpsMsg_longitude
{
public:
  explicit Init_GpsMsg_longitude(::gps_msgs::msg::GpsMsg & msg)
  : msg_(msg)
  {}
  Init_GpsMsg_altitude longitude(::gps_msgs::msg::GpsMsg::_longitude_type arg)
  {
    msg_.longitude = std::move(arg);
    return Init_GpsMsg_altitude(msg_);
  }

private:
  ::gps_msgs::msg::GpsMsg msg_;
};

class Init_GpsMsg_latitude
{
public:
  explicit Init_GpsMsg_latitude(::gps_msgs::msg::GpsMsg & msg)
  : msg_(msg)
  {}
  Init_GpsMsg_longitude latitude(::gps_msgs::msg::GpsMsg::_latitude_type arg)
  {
    msg_.latitude = std::move(arg);
    return Init_GpsMsg_longitude(msg_);
  }

private:
  ::gps_msgs::msg::GpsMsg msg_;
};

class Init_GpsMsg_header
{
public:
  Init_GpsMsg_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GpsMsg_latitude header(::gps_msgs::msg::GpsMsg::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_GpsMsg_latitude(msg_);
  }

private:
  ::gps_msgs::msg::GpsMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::gps_msgs::msg::GpsMsg>()
{
  return gps_msgs::msg::builder::Init_GpsMsg_header();
}

}  // namespace gps_msgs

#endif  // GPS_MSGS__MSG__DETAIL__GPS_MSG__BUILDER_HPP_
