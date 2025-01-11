// generated from rosidl_typesupport_opensplice_cpp/resource/idl__dds_opensplice_type_support.cpp.em
// generated code does not contain a copyright notice

#include <codecvt>
#include <cstring>
#include <iostream>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>

#include <u_instanceHandle.h>
#include <CdrTypeSupport.h>

// generated from rosidl_typesupport_opensplice_cpp/resource/msg__type_support.cpp.em
// generated code does not contain a copyright notice

#include "px4_msgs/msg/vehicle_local_position__rosidl_typesupport_opensplice_cpp.hpp"
// already included above
// #include "px4_msgs/msg/vehicle_local_position__rosidl_typesupport_opensplice_cpp.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "px4_msgs/msg/vehicle_local_position__struct.hpp"
#include "px4_msgs/msg/dds_opensplice/ccpp_VehicleLocalPosition_.h"
#include "rosidl_typesupport_opensplice_cpp/identifier.hpp"
#include "rosidl_typesupport_opensplice_cpp/message_type_support.h"
#include "rosidl_typesupport_opensplice_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_opensplice_cpp/u__instanceHandle.h"
#include "rmw/rmw.h"


// forward declaration of message dependencies and their conversion functions

namespace px4_msgs
{
namespace msg
{

namespace typesupport_opensplice_cpp
{

using __dds_msg_type_VehicleLocalPosition = px4_msgs::msg::dds_::VehicleLocalPosition_;
using __ros_msg_type_VehicleLocalPosition = px4_msgs::msg::VehicleLocalPosition;

static px4_msgs::msg::dds_::VehicleLocalPosition_TypeSupport __type_support_VehicleLocalPosition;

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_EXPORT_px4_msgs
const char *
register_type__VehicleLocalPosition(
  void * untyped_participant,
  const char * type_name)
{
  if (!untyped_participant) {
    return "untyped participant handle is null";
  }
  if (!type_name) {
    return "type name handle is null";
  }
  DDS::DomainParticipant * participant =
    static_cast<DDS::DomainParticipant *>(untyped_participant);

  DDS::ReturnCode_t status = __type_support_VehicleLocalPosition.register_type(participant, type_name);
  switch (status) {
    case DDS::RETCODE_ERROR:
      return "px4_msgs::msg::dds_::VehicleLocalPosition_TypeSupport.register_type: "
             "an internal error has occurred";
    case DDS::RETCODE_BAD_PARAMETER:
      return "px4_msgs::msg::dds_::VehicleLocalPosition_TypeSupport.register_type: "
             "bad domain participant or type name parameter";
    case DDS::RETCODE_OUT_OF_RESOURCES:
      return "px4_msgs::msg::dds_::VehicleLocalPosition_TypeSupport.register_type: "
             "out of resources";
    case DDS::RETCODE_PRECONDITION_NOT_MET:
      return "px4_msgs::msg::dds_::VehicleLocalPosition_TypeSupport.register_type: "
             "already registered with a different TypeSupport class";
    case DDS::RETCODE_OK:
      return nullptr;
    default:
      return "px4_msgs::msg::dds_::VehicleLocalPosition_TypeSupport.register_type: unknown return code";
  }
}

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_EXPORT_px4_msgs
void
convert_ros_message_to_dds(
  const __ros_msg_type_VehicleLocalPosition & ros_message,
  __dds_msg_type_VehicleLocalPosition & dds_message)
{
  // member.name timestamp
  dds_message.timestamp_ = ros_message.timestamp;
  // member.name timestamp_sample
  dds_message.timestamp_sample_ = ros_message.timestamp_sample;
  // member.name xy_valid
  dds_message.xy_valid_ = ros_message.xy_valid;
  // member.name z_valid
  dds_message.z_valid_ = ros_message.z_valid;
  // member.name v_xy_valid
  dds_message.v_xy_valid_ = ros_message.v_xy_valid;
  // member.name v_z_valid
  dds_message.v_z_valid_ = ros_message.v_z_valid;
  // member.name x
  dds_message.x_ = ros_message.x;
  // member.name y
  dds_message.y_ = ros_message.y;
  // member.name z
  dds_message.z_ = ros_message.z;
  // member.name delta_xy
  {
    size_t size = 2;
    for (DDS::ULong i = 0; i < size; i++) {
      dds_message.delta_xy_[i] = ros_message.delta_xy[i];
    }
  }
  // member.name xy_reset_counter
  dds_message.xy_reset_counter_ = ros_message.xy_reset_counter;
  // member.name delta_z
  dds_message.delta_z_ = ros_message.delta_z;
  // member.name z_reset_counter
  dds_message.z_reset_counter_ = ros_message.z_reset_counter;
  // member.name vx
  dds_message.vx_ = ros_message.vx;
  // member.name vy
  dds_message.vy_ = ros_message.vy;
  // member.name vz
  dds_message.vz_ = ros_message.vz;
  // member.name z_deriv
  dds_message.z_deriv_ = ros_message.z_deriv;
  // member.name delta_vxy
  {
    size_t size = 2;
    for (DDS::ULong i = 0; i < size; i++) {
      dds_message.delta_vxy_[i] = ros_message.delta_vxy[i];
    }
  }
  // member.name vxy_reset_counter
  dds_message.vxy_reset_counter_ = ros_message.vxy_reset_counter;
  // member.name delta_vz
  dds_message.delta_vz_ = ros_message.delta_vz;
  // member.name vz_reset_counter
  dds_message.vz_reset_counter_ = ros_message.vz_reset_counter;
  // member.name ax
  dds_message.ax_ = ros_message.ax;
  // member.name ay
  dds_message.ay_ = ros_message.ay;
  // member.name az
  dds_message.az_ = ros_message.az;
  // member.name heading
  dds_message.heading_ = ros_message.heading;
  // member.name heading_var
  dds_message.heading_var_ = ros_message.heading_var;
  // member.name unaided_heading
  dds_message.unaided_heading_ = ros_message.unaided_heading;
  // member.name delta_heading
  dds_message.delta_heading_ = ros_message.delta_heading;
  // member.name heading_reset_counter
  dds_message.heading_reset_counter_ = ros_message.heading_reset_counter;
  // member.name heading_good_for_control
  dds_message.heading_good_for_control_ = ros_message.heading_good_for_control;
  // member.name tilt_var
  dds_message.tilt_var_ = ros_message.tilt_var;
  // member.name xy_global
  dds_message.xy_global_ = ros_message.xy_global;
  // member.name z_global
  dds_message.z_global_ = ros_message.z_global;
  // member.name ref_timestamp
  dds_message.ref_timestamp_ = ros_message.ref_timestamp;
  // member.name ref_lat
  dds_message.ref_lat_ = ros_message.ref_lat;
  // member.name ref_lon
  dds_message.ref_lon_ = ros_message.ref_lon;
  // member.name ref_alt
  dds_message.ref_alt_ = ros_message.ref_alt;
  // member.name dist_bottom_valid
  dds_message.dist_bottom_valid_ = ros_message.dist_bottom_valid;
  // member.name dist_bottom
  dds_message.dist_bottom_ = ros_message.dist_bottom;
  // member.name dist_bottom_var
  dds_message.dist_bottom_var_ = ros_message.dist_bottom_var;
  // member.name delta_dist_bottom
  dds_message.delta_dist_bottom_ = ros_message.delta_dist_bottom;
  // member.name dist_bottom_reset_counter
  dds_message.dist_bottom_reset_counter_ = ros_message.dist_bottom_reset_counter;
  // member.name dist_bottom_sensor_bitfield
  dds_message.dist_bottom_sensor_bitfield_ = ros_message.dist_bottom_sensor_bitfield;
  // member.name eph
  dds_message.eph_ = ros_message.eph;
  // member.name epv
  dds_message.epv_ = ros_message.epv;
  // member.name evh
  dds_message.evh_ = ros_message.evh;
  // member.name evv
  dds_message.evv_ = ros_message.evv;
  // member.name dead_reckoning
  dds_message.dead_reckoning_ = ros_message.dead_reckoning;
  // member.name vxy_max
  dds_message.vxy_max_ = ros_message.vxy_max;
  // member.name vz_max
  dds_message.vz_max_ = ros_message.vz_max;
  // member.name hagl_min
  dds_message.hagl_min_ = ros_message.hagl_min;
  // member.name hagl_max
  dds_message.hagl_max_ = ros_message.hagl_max;
}

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_EXPORT_px4_msgs
const char *
publish__VehicleLocalPosition(
  void * untyped_topic_writer,
  const void * untyped_ros_message)
{
  DDS::DataWriter * topic_writer = static_cast<DDS::DataWriter *>(untyped_topic_writer);

  const __ros_msg_type_VehicleLocalPosition & ros_message = *static_cast<const __ros_msg_type_VehicleLocalPosition *>(untyped_ros_message);
  __dds_msg_type_VehicleLocalPosition dds_message;
  convert_ros_message_to_dds(ros_message, dds_message);

  px4_msgs::msg::dds_::VehicleLocalPosition_DataWriter * data_writer =
    px4_msgs::msg::dds_::VehicleLocalPosition_DataWriter::_narrow(topic_writer);
  DDS::ReturnCode_t status = data_writer->write(dds_message, DDS::HANDLE_NIL);
  switch (status) {
    case DDS::RETCODE_ERROR:
      return "px4_msgs::msg::dds_::VehicleLocalPosition_DataWriter.write: "
             "an internal error has occurred";
    case DDS::RETCODE_BAD_PARAMETER:
      return "px4_msgs::msg::dds_::VehicleLocalPosition_DataWriter.write: "
             "bad handle or instance_data parameter";
    case DDS::RETCODE_ALREADY_DELETED:
      return "px4_msgs::msg::dds_::VehicleLocalPosition_DataWriter.write: "
             "this px4_msgs::msg::dds_::VehicleLocalPosition_DataWriter has already been deleted";
    case DDS::RETCODE_OUT_OF_RESOURCES:
      return "px4_msgs::msg::dds_::VehicleLocalPosition_DataWriter.write: "
             "out of resources";
    case DDS::RETCODE_NOT_ENABLED:
      return "px4_msgs::msg::dds_::VehicleLocalPosition_DataWriter.write: "
             "this px4_msgs::msg::dds_::VehicleLocalPosition_DataWriter is not enabled";
    case DDS::RETCODE_PRECONDITION_NOT_MET:
      return "px4_msgs::msg::dds_::VehicleLocalPosition_DataWriter.write: "
             "the handle has not been registered with this px4_msgs::msg::dds_::VehicleLocalPosition_DataWriter";
    case DDS::RETCODE_TIMEOUT:
      return "px4_msgs::msg::dds_::VehicleLocalPosition_DataWriter.write: "
             "writing resulted in blocking and then exceeded the timeout set by the "
             "max_blocking_time of the ReliabilityQosPolicy";
    case DDS::RETCODE_OK:
      return nullptr;
    default:
      return "px4_msgs::msg::dds_::VehicleLocalPosition_DataWriter.write: unknown return code";
  }
}

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_EXPORT_px4_msgs
void
convert_dds_message_to_ros(
  const __dds_msg_type_VehicleLocalPosition & dds_message,
  __ros_msg_type_VehicleLocalPosition & ros_message)
{
  // member.name timestamp
  ros_message.timestamp =
    dds_message.timestamp_;
  // member.name timestamp_sample
  ros_message.timestamp_sample =
    dds_message.timestamp_sample_;
  // member.name xy_valid
  ros_message.xy_valid =
    (dds_message.xy_valid_ != 0);
  // member.name z_valid
  ros_message.z_valid =
    (dds_message.z_valid_ != 0);
  // member.name v_xy_valid
  ros_message.v_xy_valid =
    (dds_message.v_xy_valid_ != 0);
  // member.name v_z_valid
  ros_message.v_z_valid =
    (dds_message.v_z_valid_ != 0);
  // member.name x
  ros_message.x =
    dds_message.x_;
  // member.name y
  ros_message.y =
    dds_message.y_;
  // member.name z
  ros_message.z =
    dds_message.z_;
  // member.name delta_xy
  {
    size_t size = 2;
    for (DDS::ULong i = 0; i < size; i++) {
      ros_message.delta_xy[i] = dds_message.delta_xy_[i];
    }
  }
  // member.name xy_reset_counter
  ros_message.xy_reset_counter =
    dds_message.xy_reset_counter_;
  // member.name delta_z
  ros_message.delta_z =
    dds_message.delta_z_;
  // member.name z_reset_counter
  ros_message.z_reset_counter =
    dds_message.z_reset_counter_;
  // member.name vx
  ros_message.vx =
    dds_message.vx_;
  // member.name vy
  ros_message.vy =
    dds_message.vy_;
  // member.name vz
  ros_message.vz =
    dds_message.vz_;
  // member.name z_deriv
  ros_message.z_deriv =
    dds_message.z_deriv_;
  // member.name delta_vxy
  {
    size_t size = 2;
    for (DDS::ULong i = 0; i < size; i++) {
      ros_message.delta_vxy[i] = dds_message.delta_vxy_[i];
    }
  }
  // member.name vxy_reset_counter
  ros_message.vxy_reset_counter =
    dds_message.vxy_reset_counter_;
  // member.name delta_vz
  ros_message.delta_vz =
    dds_message.delta_vz_;
  // member.name vz_reset_counter
  ros_message.vz_reset_counter =
    dds_message.vz_reset_counter_;
  // member.name ax
  ros_message.ax =
    dds_message.ax_;
  // member.name ay
  ros_message.ay =
    dds_message.ay_;
  // member.name az
  ros_message.az =
    dds_message.az_;
  // member.name heading
  ros_message.heading =
    dds_message.heading_;
  // member.name heading_var
  ros_message.heading_var =
    dds_message.heading_var_;
  // member.name unaided_heading
  ros_message.unaided_heading =
    dds_message.unaided_heading_;
  // member.name delta_heading
  ros_message.delta_heading =
    dds_message.delta_heading_;
  // member.name heading_reset_counter
  ros_message.heading_reset_counter =
    dds_message.heading_reset_counter_;
  // member.name heading_good_for_control
  ros_message.heading_good_for_control =
    (dds_message.heading_good_for_control_ != 0);
  // member.name tilt_var
  ros_message.tilt_var =
    dds_message.tilt_var_;
  // member.name xy_global
  ros_message.xy_global =
    (dds_message.xy_global_ != 0);
  // member.name z_global
  ros_message.z_global =
    (dds_message.z_global_ != 0);
  // member.name ref_timestamp
  ros_message.ref_timestamp =
    dds_message.ref_timestamp_;
  // member.name ref_lat
  ros_message.ref_lat =
    dds_message.ref_lat_;
  // member.name ref_lon
  ros_message.ref_lon =
    dds_message.ref_lon_;
  // member.name ref_alt
  ros_message.ref_alt =
    dds_message.ref_alt_;
  // member.name dist_bottom_valid
  ros_message.dist_bottom_valid =
    (dds_message.dist_bottom_valid_ != 0);
  // member.name dist_bottom
  ros_message.dist_bottom =
    dds_message.dist_bottom_;
  // member.name dist_bottom_var
  ros_message.dist_bottom_var =
    dds_message.dist_bottom_var_;
  // member.name delta_dist_bottom
  ros_message.delta_dist_bottom =
    dds_message.delta_dist_bottom_;
  // member.name dist_bottom_reset_counter
  ros_message.dist_bottom_reset_counter =
    dds_message.dist_bottom_reset_counter_;
  // member.name dist_bottom_sensor_bitfield
  ros_message.dist_bottom_sensor_bitfield =
    dds_message.dist_bottom_sensor_bitfield_;
  // member.name eph
  ros_message.eph =
    dds_message.eph_;
  // member.name epv
  ros_message.epv =
    dds_message.epv_;
  // member.name evh
  ros_message.evh =
    dds_message.evh_;
  // member.name evv
  ros_message.evv =
    dds_message.evv_;
  // member.name dead_reckoning
  ros_message.dead_reckoning =
    (dds_message.dead_reckoning_ != 0);
  // member.name vxy_max
  ros_message.vxy_max =
    dds_message.vxy_max_;
  // member.name vz_max
  ros_message.vz_max =
    dds_message.vz_max_;
  // member.name hagl_min
  ros_message.hagl_min =
    dds_message.hagl_min_;
  // member.name hagl_max
  ros_message.hagl_max =
    dds_message.hagl_max_;
}

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_EXPORT_px4_msgs
const char *
take__VehicleLocalPosition(
  void * untyped_topic_reader,
  bool ignore_local_publications,
  void * untyped_ros_message,
  bool * taken,
  void * sending_publication_handle)
{
  if (untyped_ros_message == 0) {
    return "invalid ros message pointer";
  }

  DDS::DataReader * topic_reader = static_cast<DDS::DataReader *>(untyped_topic_reader);

  px4_msgs::msg::dds_::VehicleLocalPosition_DataReader * data_reader =
    px4_msgs::msg::dds_::VehicleLocalPosition_DataReader::_narrow(topic_reader);

  px4_msgs::msg::dds_::VehicleLocalPosition_Seq dds_messages;
  DDS::SampleInfoSeq sample_infos;
  DDS::ReturnCode_t status = data_reader->take(
    dds_messages,
    sample_infos,
    1,
    DDS::ANY_SAMPLE_STATE,
    DDS::ANY_VIEW_STATE,
    DDS::ANY_INSTANCE_STATE);

  const char * errs = nullptr;
  bool ignore_sample = false;

  switch (status) {
    case DDS::RETCODE_ERROR:
      errs = "px4_msgs::msg::dds_::VehicleLocalPosition_DataReader.take: "
        "an internal error has occurred";
      goto finally;
    case DDS::RETCODE_ALREADY_DELETED:
      errs = "px4_msgs::msg::dds_::VehicleLocalPosition_DataReader.take: "
        "this px4_msgs::msg::dds_::VehicleLocalPosition_DataReader has already been deleted";
      goto finally;
    case DDS::RETCODE_OUT_OF_RESOURCES:
      errs = "px4_msgs::msg::dds_::VehicleLocalPosition_DataReader.take: "
        "out of resources";
      goto finally;
    case DDS::RETCODE_NOT_ENABLED:
      errs = "px4_msgs::msg::dds_::VehicleLocalPosition_DataReader.take: "
        "this px4_msgs::msg::dds_::VehicleLocalPosition_DataReader is not enabled";
      goto finally;
    case DDS::RETCODE_PRECONDITION_NOT_MET:
      errs = "px4_msgs::msg::dds_::VehicleLocalPosition_DataReader.take: "
        "a precondition is not met, one of: "
        "max_samples > maximum and max_samples != LENGTH_UNLIMITED, or "
        "the two sequences do not have matching parameters (length, maximum, release), or "
        "maximum > 0 and release is false.";
      goto finally;
    case DDS::RETCODE_NO_DATA:
      *taken = false;
      errs = nullptr;
      goto finally;
    case DDS::RETCODE_OK:
      break;
    default:
      errs = "px4_msgs::msg::dds_::VehicleLocalPosition_DataReader.take: unknown return code";
      goto finally;
  }

  {
    DDS::SampleInfo & sample_info = sample_infos[0];
    if (!sample_info.valid_data) {
      // skip sample without data
      ignore_sample = true;
    } else {
      DDS::InstanceHandle_t sender_handle = sample_info.publication_handle;
      auto sender_gid = u_instanceHandleToGID(sender_handle);
      if (ignore_local_publications) {
        // compare the system id from the sender and this receiver
        // if they are equal the sample has been sent from this process and should be ignored
        DDS::InstanceHandle_t receiver_handle = topic_reader->get_instance_handle();
        auto receiver_gid = u_instanceHandleToGID(receiver_handle);
        ignore_sample = sender_gid.systemId == receiver_gid.systemId;
      }
      // This is nullptr when being used with plain rmw_take, so check first.
      if (sending_publication_handle) {
        *static_cast<DDS::InstanceHandle_t *>(sending_publication_handle) = sender_handle;
      }
    }
  }

  if (!ignore_sample) {
    __ros_msg_type_VehicleLocalPosition & ros_message = *static_cast<__ros_msg_type_VehicleLocalPosition *>(untyped_ros_message);
    convert_dds_message_to_ros(dds_messages[0], ros_message);
    *taken = true;
  } else {
    *taken = false;
  }

finally:
  // Ensure the loan is returned.
  status = data_reader->return_loan(dds_messages, sample_infos);
  switch (status) {
    case DDS::RETCODE_ERROR:
      return "px4_msgs::msg::dds_::VehicleLocalPosition_DataReader.return_loan: "
             "an internal error has occurred";
    case DDS::RETCODE_ALREADY_DELETED:
      return "px4_msgs::msg::dds_::VehicleLocalPosition_DataReader.return_loan: "
             "this px4_msgs::msg::dds_::VehicleLocalPosition_DataReader has already been deleted";
    case DDS::RETCODE_OUT_OF_RESOURCES:
      return "px4_msgs::msg::dds_::VehicleLocalPosition_DataReader.return_loan: "
             "out of resources";
    case DDS::RETCODE_NOT_ENABLED:
      return "px4_msgs::msg::dds_::VehicleLocalPosition_DataReader.return_loan: "
             "this px4_msgs::msg::dds_::VehicleLocalPosition_DataReader is not enabled";
    case DDS::RETCODE_PRECONDITION_NOT_MET:
      return "px4_msgs::msg::dds_::VehicleLocalPosition_DataReader.return_loan: "
             "a precondition is not met, one of: "
             "the data_values and info_seq do not belong to a single related pair, or "
             "the data_values and info_seq were not obtained from this "
             "px4_msgs::msg::dds_::VehicleLocalPosition_DataReader";
    case DDS::RETCODE_OK:
      break;
    default:
      return "px4_msgs::msg::dds_::VehicleLocalPosition_DataReader.return_loan failed with "
             "unknown return code";
  }

  return errs;
}

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_EXPORT_px4_msgs
const char *
serialize__VehicleLocalPosition(
  const void * untyped_ros_message,
  void * untyped_serialized_data)
{
  const __ros_msg_type_VehicleLocalPosition & ros_message = *static_cast<const __ros_msg_type_VehicleLocalPosition *>(untyped_ros_message);
  __dds_msg_type_VehicleLocalPosition dds_message;

  convert_ros_message_to_dds(ros_message, dds_message);

  DDS::OpenSplice::CdrTypeSupport cdr_ts(__type_support_VehicleLocalPosition);
  DDS::OpenSplice::CdrSerializedData * serdata = nullptr;

  DDS::ReturnCode_t status = cdr_ts.serialize(&dds_message, &serdata);
  switch (status) {
    case DDS::RETCODE_ERROR:
      return "px4_msgs::msg::dds_::VehicleLocalPosition_TypeSupport.serialize: "
             "an internal error has occurred";
    case DDS::RETCODE_BAD_PARAMETER:
      return "px4_msgs::msg::dds_::VehicleLocalPosition_TypeSupport.serialize: "
             "bad parameter";
    case DDS::RETCODE_ALREADY_DELETED:
      return "px4_msgs::msg::dds_::VehicleLocalPosition_TypeSupport.serialize: "
             "this px4_msgs::msg::dds_::VehicleLocalPosition_TypeSupport has already been deleted";
    case DDS::RETCODE_OUT_OF_RESOURCES:
      return "px4_msgs::msg::dds_::VehicleLocalPosition_TypeSupport.serialize: "
             "out of resources";
    case DDS::RETCODE_OK:
      break;
    default:
      return "px4_msgs::msg::dds_::VehicleLocalPosition_TypeSupport.serialize failed with "
             "unknown return code";
  }

  rmw_serialized_message_t * serialized_data =
    static_cast<rmw_serialized_message_t *>(untyped_serialized_data);

  auto data_length = serdata->get_size();

  if (serialized_data->buffer_capacity < data_length) {
    if (rmw_serialized_message_resize(serialized_data, data_length) == RMW_RET_OK) {
      serialized_data->buffer_capacity = data_length;
    } else {
      delete serdata;
      return "px4_msgs::msg::dds_::VehicleLocalPosition_TypeSupport.serialize: "
             "unable to dynamically resize serialized message";
    }
  }

  serialized_data->buffer_length = data_length;
  serdata->get_data(serialized_data->buffer);

  delete serdata;

  return nullptr;
}

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_EXPORT_px4_msgs
const char *
deserialize__VehicleLocalPosition(
  const uint8_t * buffer,
  unsigned length,
  void * untyped_ros_message)
{
  __dds_msg_type_VehicleLocalPosition dds_message;

  DDS::OpenSplice::CdrTypeSupport cdr_ts(__type_support_VehicleLocalPosition);

  DDS::ReturnCode_t status = cdr_ts.deserialize(buffer, length, &dds_message);

  switch (status) {
    case DDS::RETCODE_ERROR:
      return "px4_msgs::msg::dds_::VehicleLocalPosition_TypeSupport.deserialize: "
             "an internal error has occurred";
    case DDS::RETCODE_BAD_PARAMETER:
      return "px4_msgs::msg::dds_::VehicleLocalPosition_TypeSupport.deserialize: "
             "bad parameter";
    case DDS::RETCODE_ALREADY_DELETED:
      return "px4_msgs::msg::dds_::VehicleLocalPosition_TypeSupport.deserialize: "
             "this px4_msgs::msg::dds_::VehicleLocalPosition_TypeSupport has already been deleted";
    case DDS::RETCODE_OUT_OF_RESOURCES:
      return "px4_msgs::msg::dds_::VehicleLocalPosition_TypeSupport.deserialize: "
             "out of resources";
    case DDS::RETCODE_OK:
      break;
    default:
      return "px4_msgs::msg::dds_::VehicleLocalPosition_TypeSupport.deserialize failed with "
             "unknown return code";
  }

  __ros_msg_type_VehicleLocalPosition & ros_message = *static_cast<__ros_msg_type_VehicleLocalPosition *>(untyped_ros_message);
  convert_dds_message_to_ros(dds_message, ros_message);

  return nullptr;
}

static message_type_support_callbacks_t VehicleLocalPosition_callbacks = {
  "px4_msgs::msg",
  "VehicleLocalPosition",
  &register_type__VehicleLocalPosition,
  &publish__VehicleLocalPosition,
  &take__VehicleLocalPosition,
  &serialize__VehicleLocalPosition,
  &deserialize__VehicleLocalPosition,
  nullptr,  // convert ros to dds (handled differently for C++)
  nullptr,  // convert dds to ros (handled differently for C++)
};

static rosidl_message_type_support_t VehicleLocalPosition_handle = {
  rosidl_typesupport_opensplice_cpp::typesupport_identifier,
  &VehicleLocalPosition_callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_opensplice_cpp

}  // namespace msg
}  // namespace px4_msgs

namespace rosidl_typesupport_opensplice_cpp
{

template<>
ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_EXPORT_px4_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<px4_msgs::msg::VehicleLocalPosition>()
{
  return &px4_msgs::msg::typesupport_opensplice_cpp::VehicleLocalPosition_handle;
}

}  // namespace rosidl_typesupport_opensplice_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_opensplice_cpp,
  px4_msgs, msg,
  VehicleLocalPosition)()
{
  return &px4_msgs::msg::typesupport_opensplice_cpp::VehicleLocalPosition_handle;
}

#ifdef __cplusplus
}
#endif
