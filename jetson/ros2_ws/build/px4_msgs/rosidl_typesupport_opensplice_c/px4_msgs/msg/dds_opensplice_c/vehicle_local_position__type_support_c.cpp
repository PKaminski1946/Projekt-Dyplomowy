// generated from rosidl_typesupport_opensplice_c/resource/idl__dds_opensplice__type_support.c.em
// generated code does not contain a copyright notice

#include <cassert>
#include <codecvt>
#include <cstring>
#include <iostream>
#include <limits>
#include <locale>
#include <sstream>

#include <CdrTypeSupport.h>
#include <u_instanceHandle.h>

// generated from rosidl_typesupport_opensplice_c/resource/msg__type_support_c.cpp.em
// generated code does not contain a copyright notice

#include "px4_msgs/msg/vehicle_local_position__rosidl_typesupport_opensplice_c.h"
#include "rosidl_typesupport_opensplice_c/identifier.h"
#include "px4_msgs/msg/rosidl_generator_c__visibility_control.h"
#include "rosidl_typesupport_opensplice_cpp/message_type_support.h"
#include "rosidl_typesupport_opensplice_cpp/u__instanceHandle.h"
#include "rmw/rmw.h"
#include "px4_msgs/msg/rosidl_typesupport_opensplice_c__visibility_control.h"
#include "px4_msgs/msg/vehicle_local_position.h"
#include "px4_msgs/msg/dds_opensplice/ccpp_VehicleLocalPosition_.h"

// includes and forward declarations of message dependencies and their conversion functions
#if defined(__cplusplus)
extern "C"
{
#endif

// include message dependencies
#include "rosidl_generator_c/primitives_sequence.h"  // delta_vxy, delta_xy
#include "rosidl_generator_c/primitives_sequence_functions.h"  // delta_vxy, delta_xy

// forward declare type support functions

using __dds_msg_type_px4_msgs__msg__VehicleLocalPosition = px4_msgs::msg::dds_::VehicleLocalPosition_;
using __ros_msg_type_px4_msgs__msg__VehicleLocalPosition = px4_msgs__msg__VehicleLocalPosition;

static px4_msgs::msg::dds_::VehicleLocalPosition_TypeSupport _type_support_px4_msgs__msg__VehicleLocalPosition;

static const char *
register_type_px4_msgs__msg__VehicleLocalPosition(void * untyped_participant, const char * type_name)
{
  if (!untyped_participant) {
    return "untyped participant handle is null";
  }
  if (!type_name) {
    return "type name handle is null";
  }
  using DDS::DomainParticipant;
  DomainParticipant * participant = static_cast<DomainParticipant *>(untyped_participant);

  DDS::ReturnCode_t status = _type_support_px4_msgs__msg__VehicleLocalPosition.register_type(participant, type_name);
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

static const char *
convert_ros_to_dds_px4_msgs__msg__VehicleLocalPosition(const void * untyped_ros_message, void * untyped_dds_message)
{
  if (!untyped_ros_message) {
    return "ros message handle is null";
  }
  if (!untyped_dds_message) {
    return "dds message handle is null";
  }
  const __ros_msg_type_px4_msgs__msg__VehicleLocalPosition * ros_message = static_cast<const __ros_msg_type_px4_msgs__msg__VehicleLocalPosition *>(untyped_ros_message);
  __dds_msg_type_px4_msgs__msg__VehicleLocalPosition * dds_message = static_cast<__dds_msg_type_px4_msgs__msg__VehicleLocalPosition *>(untyped_dds_message);
  // Field name: timestamp
  {
    dds_message->timestamp_ = ros_message->timestamp;
  }

  // Field name: timestamp_sample
  {
    dds_message->timestamp_sample_ = ros_message->timestamp_sample;
  }

  // Field name: xy_valid
  {
    dds_message->xy_valid_ = ros_message->xy_valid;
  }

  // Field name: z_valid
  {
    dds_message->z_valid_ = ros_message->z_valid;
  }

  // Field name: v_xy_valid
  {
    dds_message->v_xy_valid_ = ros_message->v_xy_valid;
  }

  // Field name: v_z_valid
  {
    dds_message->v_z_valid_ = ros_message->v_z_valid;
  }

  // Field name: x
  {
    dds_message->x_ = ros_message->x;
  }

  // Field name: y
  {
    dds_message->y_ = ros_message->y;
  }

  // Field name: z
  {
    dds_message->z_ = ros_message->z;
  }

  // Field name: delta_xy
  {
    size_t size = 2;
    for (DDS::ULong i = 0; i < size; ++i) {
      auto & ros_i = ros_message->delta_xy[i];
      dds_message->delta_xy_[i] = ros_i;
    }
  }

  // Field name: xy_reset_counter
  {
    dds_message->xy_reset_counter_ = ros_message->xy_reset_counter;
  }

  // Field name: delta_z
  {
    dds_message->delta_z_ = ros_message->delta_z;
  }

  // Field name: z_reset_counter
  {
    dds_message->z_reset_counter_ = ros_message->z_reset_counter;
  }

  // Field name: vx
  {
    dds_message->vx_ = ros_message->vx;
  }

  // Field name: vy
  {
    dds_message->vy_ = ros_message->vy;
  }

  // Field name: vz
  {
    dds_message->vz_ = ros_message->vz;
  }

  // Field name: z_deriv
  {
    dds_message->z_deriv_ = ros_message->z_deriv;
  }

  // Field name: delta_vxy
  {
    size_t size = 2;
    for (DDS::ULong i = 0; i < size; ++i) {
      auto & ros_i = ros_message->delta_vxy[i];
      dds_message->delta_vxy_[i] = ros_i;
    }
  }

  // Field name: vxy_reset_counter
  {
    dds_message->vxy_reset_counter_ = ros_message->vxy_reset_counter;
  }

  // Field name: delta_vz
  {
    dds_message->delta_vz_ = ros_message->delta_vz;
  }

  // Field name: vz_reset_counter
  {
    dds_message->vz_reset_counter_ = ros_message->vz_reset_counter;
  }

  // Field name: ax
  {
    dds_message->ax_ = ros_message->ax;
  }

  // Field name: ay
  {
    dds_message->ay_ = ros_message->ay;
  }

  // Field name: az
  {
    dds_message->az_ = ros_message->az;
  }

  // Field name: heading
  {
    dds_message->heading_ = ros_message->heading;
  }

  // Field name: heading_var
  {
    dds_message->heading_var_ = ros_message->heading_var;
  }

  // Field name: unaided_heading
  {
    dds_message->unaided_heading_ = ros_message->unaided_heading;
  }

  // Field name: delta_heading
  {
    dds_message->delta_heading_ = ros_message->delta_heading;
  }

  // Field name: heading_reset_counter
  {
    dds_message->heading_reset_counter_ = ros_message->heading_reset_counter;
  }

  // Field name: heading_good_for_control
  {
    dds_message->heading_good_for_control_ = ros_message->heading_good_for_control;
  }

  // Field name: tilt_var
  {
    dds_message->tilt_var_ = ros_message->tilt_var;
  }

  // Field name: xy_global
  {
    dds_message->xy_global_ = ros_message->xy_global;
  }

  // Field name: z_global
  {
    dds_message->z_global_ = ros_message->z_global;
  }

  // Field name: ref_timestamp
  {
    dds_message->ref_timestamp_ = ros_message->ref_timestamp;
  }

  // Field name: ref_lat
  {
    dds_message->ref_lat_ = ros_message->ref_lat;
  }

  // Field name: ref_lon
  {
    dds_message->ref_lon_ = ros_message->ref_lon;
  }

  // Field name: ref_alt
  {
    dds_message->ref_alt_ = ros_message->ref_alt;
  }

  // Field name: dist_bottom_valid
  {
    dds_message->dist_bottom_valid_ = ros_message->dist_bottom_valid;
  }

  // Field name: dist_bottom
  {
    dds_message->dist_bottom_ = ros_message->dist_bottom;
  }

  // Field name: dist_bottom_var
  {
    dds_message->dist_bottom_var_ = ros_message->dist_bottom_var;
  }

  // Field name: delta_dist_bottom
  {
    dds_message->delta_dist_bottom_ = ros_message->delta_dist_bottom;
  }

  // Field name: dist_bottom_reset_counter
  {
    dds_message->dist_bottom_reset_counter_ = ros_message->dist_bottom_reset_counter;
  }

  // Field name: dist_bottom_sensor_bitfield
  {
    dds_message->dist_bottom_sensor_bitfield_ = ros_message->dist_bottom_sensor_bitfield;
  }

  // Field name: eph
  {
    dds_message->eph_ = ros_message->eph;
  }

  // Field name: epv
  {
    dds_message->epv_ = ros_message->epv;
  }

  // Field name: evh
  {
    dds_message->evh_ = ros_message->evh;
  }

  // Field name: evv
  {
    dds_message->evv_ = ros_message->evv;
  }

  // Field name: dead_reckoning
  {
    dds_message->dead_reckoning_ = ros_message->dead_reckoning;
  }

  // Field name: vxy_max
  {
    dds_message->vxy_max_ = ros_message->vxy_max;
  }

  // Field name: vz_max
  {
    dds_message->vz_max_ = ros_message->vz_max;
  }

  // Field name: hagl_min
  {
    dds_message->hagl_min_ = ros_message->hagl_min;
  }

  // Field name: hagl_max
  {
    dds_message->hagl_max_ = ros_message->hagl_max;
  }

  return 0;
}

static const char *
publish_px4_msgs__msg__VehicleLocalPosition(void * dds_data_writer, const void * ros_message)
{
  if (!dds_data_writer) {
    return "data writer handle is null";
  }
  if (!ros_message) {
    return "ros message handle is null";
  }

  DDS::DataWriter * topic_writer = static_cast<DDS::DataWriter *>(dds_data_writer);

  __dds_msg_type_px4_msgs__msg__VehicleLocalPosition dds_message;
  const char * err_msg = convert_ros_to_dds_px4_msgs__msg__VehicleLocalPosition(ros_message, &dds_message);
  if (err_msg != 0) {
    return err_msg;
  }

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

static const char *
convert_dds_to_ros_px4_msgs__msg__VehicleLocalPosition(const void * untyped_dds_message, void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    return "ros message handle is null";
  }
  if (!untyped_dds_message) {
    return "dds message handle is null";
  }
  const __dds_msg_type_px4_msgs__msg__VehicleLocalPosition * dds_message = static_cast<const __dds_msg_type_px4_msgs__msg__VehicleLocalPosition *>(untyped_dds_message);
  __ros_msg_type_px4_msgs__msg__VehicleLocalPosition * ros_message = static_cast<__ros_msg_type_px4_msgs__msg__VehicleLocalPosition *>(untyped_ros_message);
  // Field name: timestamp
  {
    ros_message->timestamp = dds_message->timestamp_;
  }

  // Field name: timestamp_sample
  {
    ros_message->timestamp_sample = dds_message->timestamp_sample_;
  }

  // Field name: xy_valid
  {
    ros_message->xy_valid = (dds_message->xy_valid_ != 0);
  }

  // Field name: z_valid
  {
    ros_message->z_valid = (dds_message->z_valid_ != 0);
  }

  // Field name: v_xy_valid
  {
    ros_message->v_xy_valid = (dds_message->v_xy_valid_ != 0);
  }

  // Field name: v_z_valid
  {
    ros_message->v_z_valid = (dds_message->v_z_valid_ != 0);
  }

  // Field name: x
  {
    ros_message->x = dds_message->x_;
  }

  // Field name: y
  {
    ros_message->y = dds_message->y_;
  }

  // Field name: z
  {
    ros_message->z = dds_message->z_;
  }

  // Field name: delta_xy
  {
    size_t size = 2;
    for (DDS::ULong i = 0; i < size; i++) {
      auto & ros_i = ros_message->delta_xy[i];
      ros_i = dds_message->delta_xy_[i];
    }
  }

  // Field name: xy_reset_counter
  {
    ros_message->xy_reset_counter = dds_message->xy_reset_counter_;
  }

  // Field name: delta_z
  {
    ros_message->delta_z = dds_message->delta_z_;
  }

  // Field name: z_reset_counter
  {
    ros_message->z_reset_counter = dds_message->z_reset_counter_;
  }

  // Field name: vx
  {
    ros_message->vx = dds_message->vx_;
  }

  // Field name: vy
  {
    ros_message->vy = dds_message->vy_;
  }

  // Field name: vz
  {
    ros_message->vz = dds_message->vz_;
  }

  // Field name: z_deriv
  {
    ros_message->z_deriv = dds_message->z_deriv_;
  }

  // Field name: delta_vxy
  {
    size_t size = 2;
    for (DDS::ULong i = 0; i < size; i++) {
      auto & ros_i = ros_message->delta_vxy[i];
      ros_i = dds_message->delta_vxy_[i];
    }
  }

  // Field name: vxy_reset_counter
  {
    ros_message->vxy_reset_counter = dds_message->vxy_reset_counter_;
  }

  // Field name: delta_vz
  {
    ros_message->delta_vz = dds_message->delta_vz_;
  }

  // Field name: vz_reset_counter
  {
    ros_message->vz_reset_counter = dds_message->vz_reset_counter_;
  }

  // Field name: ax
  {
    ros_message->ax = dds_message->ax_;
  }

  // Field name: ay
  {
    ros_message->ay = dds_message->ay_;
  }

  // Field name: az
  {
    ros_message->az = dds_message->az_;
  }

  // Field name: heading
  {
    ros_message->heading = dds_message->heading_;
  }

  // Field name: heading_var
  {
    ros_message->heading_var = dds_message->heading_var_;
  }

  // Field name: unaided_heading
  {
    ros_message->unaided_heading = dds_message->unaided_heading_;
  }

  // Field name: delta_heading
  {
    ros_message->delta_heading = dds_message->delta_heading_;
  }

  // Field name: heading_reset_counter
  {
    ros_message->heading_reset_counter = dds_message->heading_reset_counter_;
  }

  // Field name: heading_good_for_control
  {
    ros_message->heading_good_for_control = (dds_message->heading_good_for_control_ != 0);
  }

  // Field name: tilt_var
  {
    ros_message->tilt_var = dds_message->tilt_var_;
  }

  // Field name: xy_global
  {
    ros_message->xy_global = (dds_message->xy_global_ != 0);
  }

  // Field name: z_global
  {
    ros_message->z_global = (dds_message->z_global_ != 0);
  }

  // Field name: ref_timestamp
  {
    ros_message->ref_timestamp = dds_message->ref_timestamp_;
  }

  // Field name: ref_lat
  {
    ros_message->ref_lat = dds_message->ref_lat_;
  }

  // Field name: ref_lon
  {
    ros_message->ref_lon = dds_message->ref_lon_;
  }

  // Field name: ref_alt
  {
    ros_message->ref_alt = dds_message->ref_alt_;
  }

  // Field name: dist_bottom_valid
  {
    ros_message->dist_bottom_valid = (dds_message->dist_bottom_valid_ != 0);
  }

  // Field name: dist_bottom
  {
    ros_message->dist_bottom = dds_message->dist_bottom_;
  }

  // Field name: dist_bottom_var
  {
    ros_message->dist_bottom_var = dds_message->dist_bottom_var_;
  }

  // Field name: delta_dist_bottom
  {
    ros_message->delta_dist_bottom = dds_message->delta_dist_bottom_;
  }

  // Field name: dist_bottom_reset_counter
  {
    ros_message->dist_bottom_reset_counter = dds_message->dist_bottom_reset_counter_;
  }

  // Field name: dist_bottom_sensor_bitfield
  {
    ros_message->dist_bottom_sensor_bitfield = dds_message->dist_bottom_sensor_bitfield_;
  }

  // Field name: eph
  {
    ros_message->eph = dds_message->eph_;
  }

  // Field name: epv
  {
    ros_message->epv = dds_message->epv_;
  }

  // Field name: evh
  {
    ros_message->evh = dds_message->evh_;
  }

  // Field name: evv
  {
    ros_message->evv = dds_message->evv_;
  }

  // Field name: dead_reckoning
  {
    ros_message->dead_reckoning = (dds_message->dead_reckoning_ != 0);
  }

  // Field name: vxy_max
  {
    ros_message->vxy_max = dds_message->vxy_max_;
  }

  // Field name: vz_max
  {
    ros_message->vz_max = dds_message->vz_max_;
  }

  // Field name: hagl_min
  {
    ros_message->hagl_min = dds_message->hagl_min_;
  }

  // Field name: hagl_max
  {
    ros_message->hagl_max = dds_message->hagl_max_;
  }

  return 0;
}

static const char *
take_px4_msgs__msg__VehicleLocalPosition(
  void * dds_data_reader,
  bool ignore_local_publications,
  void * untyped_ros_message,
  bool * taken,
  void * sending_publication_handle)
{
  if (untyped_ros_message == 0) {
    return "invalid ros message pointer";
  }

  DDS::DataReader * topic_reader = static_cast<DDS::DataReader *>(dds_data_reader);

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
    errs = convert_dds_to_ros_px4_msgs__msg__VehicleLocalPosition(&dds_messages[0], untyped_ros_message);
    if (errs != 0) {
      goto finally;
    }
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
      return nullptr;
    default:
      return "px4_msgs::msg::dds_::VehicleLocalPosition_DataReader.return_loan failed with "
             "unknown return code";
  }

  return errs;
}

static const char *
serialize_px4_msgs__msg__VehicleLocalPosition(
  const void * untyped_ros_message,
  void * untyped_serialized_data)
{
  if (!untyped_ros_message) {
    return "ros message handle is null";
  }
  if (!untyped_serialized_data) {
    return "serialized_data handle is null";
  }

  __dds_msg_type_px4_msgs__msg__VehicleLocalPosition dds_message;
  const char * err_msg = convert_ros_to_dds_px4_msgs__msg__VehicleLocalPosition(untyped_ros_message, &dds_message);
  if (err_msg != 0) {
    return err_msg;
  }

  DDS::OpenSplice::CdrTypeSupport cdr_ts(_type_support_px4_msgs__msg__VehicleLocalPosition);

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

static const char *
deserialize_px4_msgs__msg__VehicleLocalPosition(
  const uint8_t * buffer,
  unsigned length,
  void * untyped_ros_message)
{
  const char * errs = nullptr;

  if (untyped_ros_message == 0) {
    return "invalid ros message pointer";
  }

  DDS::OpenSplice::CdrTypeSupport cdr_ts(_type_support_px4_msgs__msg__VehicleLocalPosition);

  __dds_msg_type_px4_msgs__msg__VehicleLocalPosition dds_message;
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

  errs = convert_dds_to_ros_px4_msgs__msg__VehicleLocalPosition(&dds_message, untyped_ros_message);

  return errs;
}


static message_type_support_callbacks_t VehicleLocalPosition__callbacks = {
  "px4_msgs::msg",  // message_namespace
  "VehicleLocalPosition",  // message_name
  register_type_px4_msgs__msg__VehicleLocalPosition,  // register_type
  publish_px4_msgs__msg__VehicleLocalPosition,  // publish
  take_px4_msgs__msg__VehicleLocalPosition,  // take
  serialize_px4_msgs__msg__VehicleLocalPosition,  // serialize message
  deserialize_px4_msgs__msg__VehicleLocalPosition,  // deserialize message
  convert_ros_to_dds_px4_msgs__msg__VehicleLocalPosition,  // convert_ros_to_dds
  convert_dds_to_ros_px4_msgs__msg__VehicleLocalPosition,  // convert_dds_to_ros
};

static rosidl_message_type_support_t VehicleLocalPosition__type_support = {
  rosidl_typesupport_opensplice_c__identifier,
  &VehicleLocalPosition__callbacks,  // data
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_opensplice_c,
  px4_msgs, msg,
  VehicleLocalPosition)()
{
  return &VehicleLocalPosition__type_support;
}

#if defined(__cplusplus)
}
#endif
