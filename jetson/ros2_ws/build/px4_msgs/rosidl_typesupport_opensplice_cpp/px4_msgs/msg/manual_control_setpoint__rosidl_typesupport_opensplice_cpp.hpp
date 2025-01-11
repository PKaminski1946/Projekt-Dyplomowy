// generated from rosidl_typesupport_opensplice_cpp/resource/idl__rosidl_typesupport_cpp.hpp.em
// generated code does not contain a copyright notice
#ifndef PX4_MSGS__MSG__MANUAL_CONTROL_SETPOINT__ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_HPP_
#define PX4_MSGS__MSG__MANUAL_CONTROL_SETPOINT__ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_HPP_
// generated from
// rosidl_typesupport_opensplice_cpp/resource/msg__rosidl_typesupport_opensplice_cpp.hpp.em
// generated code does not contain a copyright notice

#include "px4_msgs/msg/manual_control_setpoint__struct.hpp"
#include "px4_msgs/msg/dds_opensplice/ccpp_ManualControlSetpoint_.h"
#include "rosidl_generator_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "px4_msgs/msg/rosidl_typesupport_opensplice_cpp__visibility_control.h"

namespace DDS
{
class DomainParticipant;
class DataReader;
class DataWriter;
}  // namespace DDS

namespace px4_msgs
{
namespace msg
{
namespace typesupport_opensplice_cpp
{

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_px4_msgs
extern void register_type__ManualControlSetpoint(
  DDS::DomainParticipant * participant,
  const char * type_name);

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_px4_msgs
extern void convert_ros_message_to_dds(
  const px4_msgs::msg::ManualControlSetpoint & ros_message,
  px4_msgs::msg::dds_::ManualControlSetpoint_ & dds_message);

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_px4_msgs
extern void publish__ManualControlSetpoint(
  DDS::DataWriter * topic_writer,
  const void * untyped_ros_message);

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_px4_msgs
extern void convert_dds_message_to_ros(
  const px4_msgs::msg::dds_::ManualControlSetpoint_ & dds_message,
  px4_msgs::msg::ManualControlSetpoint & ros_message);

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_px4_msgs
extern bool take__ManualControlSetpoint(
  DDS::DataReader * topic_reader,
  bool ignore_local_publications,
  void * untyped_ros_message,
  bool * taken);

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_EXPORT_px4_msgs
const char *
serialize__ManualControlSetpoint(
  const void * untyped_ros_message,
  void * serialized_data);

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_EXPORT_px4_msgs
const char *
deserialize__ManualControlSetpoint(
  const uint8_t * buffer,
  unsigned length,
  void * untyped_ros_message);

}  // namespace typesupport_opensplice_cpp

}  // namespace msg
}  // namespace px4_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_PUBLIC_px4_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_opensplice_cpp,
  px4_msgs, msg,
  ManualControlSetpoint)();

#ifdef __cplusplus
}
#endif
#endif  // PX4_MSGS__MSG__MANUAL_CONTROL_SETPOINT__ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_HPP_
