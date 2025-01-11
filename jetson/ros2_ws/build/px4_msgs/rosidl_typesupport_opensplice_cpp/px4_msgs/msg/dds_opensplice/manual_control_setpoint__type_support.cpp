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

#include "px4_msgs/msg/manual_control_setpoint__rosidl_typesupport_opensplice_cpp.hpp"
// already included above
// #include "px4_msgs/msg/manual_control_setpoint__rosidl_typesupport_opensplice_cpp.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "px4_msgs/msg/manual_control_setpoint__struct.hpp"
#include "px4_msgs/msg/dds_opensplice/ccpp_ManualControlSetpoint_.h"
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

using __dds_msg_type_ManualControlSetpoint = px4_msgs::msg::dds_::ManualControlSetpoint_;
using __ros_msg_type_ManualControlSetpoint = px4_msgs::msg::ManualControlSetpoint;

static px4_msgs::msg::dds_::ManualControlSetpoint_TypeSupport __type_support_ManualControlSetpoint;

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_EXPORT_px4_msgs
const char *
register_type__ManualControlSetpoint(
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

  DDS::ReturnCode_t status = __type_support_ManualControlSetpoint.register_type(participant, type_name);
  switch (status) {
    case DDS::RETCODE_ERROR:
      return "px4_msgs::msg::dds_::ManualControlSetpoint_TypeSupport.register_type: "
             "an internal error has occurred";
    case DDS::RETCODE_BAD_PARAMETER:
      return "px4_msgs::msg::dds_::ManualControlSetpoint_TypeSupport.register_type: "
             "bad domain participant or type name parameter";
    case DDS::RETCODE_OUT_OF_RESOURCES:
      return "px4_msgs::msg::dds_::ManualControlSetpoint_TypeSupport.register_type: "
             "out of resources";
    case DDS::RETCODE_PRECONDITION_NOT_MET:
      return "px4_msgs::msg::dds_::ManualControlSetpoint_TypeSupport.register_type: "
             "already registered with a different TypeSupport class";
    case DDS::RETCODE_OK:
      return nullptr;
    default:
      return "px4_msgs::msg::dds_::ManualControlSetpoint_TypeSupport.register_type: unknown return code";
  }
}

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_EXPORT_px4_msgs
void
convert_ros_message_to_dds(
  const __ros_msg_type_ManualControlSetpoint & ros_message,
  __dds_msg_type_ManualControlSetpoint & dds_message)
{
  // member.name timestamp
  dds_message.timestamp_ = ros_message.timestamp;
  // member.name timestamp_sample
  dds_message.timestamp_sample_ = ros_message.timestamp_sample;
  // member.name valid
  dds_message.valid_ = ros_message.valid;
  // member.name data_source
  dds_message.data_source_ = ros_message.data_source;
  // member.name roll
  dds_message.roll_ = ros_message.roll;
  // member.name pitch
  dds_message.pitch_ = ros_message.pitch;
  // member.name yaw
  dds_message.yaw_ = ros_message.yaw;
  // member.name throttle
  dds_message.throttle_ = ros_message.throttle;
  // member.name flaps
  dds_message.flaps_ = ros_message.flaps;
  // member.name aux1
  dds_message.aux1_ = ros_message.aux1;
  // member.name aux2
  dds_message.aux2_ = ros_message.aux2;
  // member.name aux3
  dds_message.aux3_ = ros_message.aux3;
  // member.name aux4
  dds_message.aux4_ = ros_message.aux4;
  // member.name aux5
  dds_message.aux5_ = ros_message.aux5;
  // member.name aux6
  dds_message.aux6_ = ros_message.aux6;
  // member.name sticks_moving
  dds_message.sticks_moving_ = ros_message.sticks_moving;
  // member.name buttons
  dds_message.buttons_ = ros_message.buttons;
}

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_EXPORT_px4_msgs
const char *
publish__ManualControlSetpoint(
  void * untyped_topic_writer,
  const void * untyped_ros_message)
{
  DDS::DataWriter * topic_writer = static_cast<DDS::DataWriter *>(untyped_topic_writer);

  const __ros_msg_type_ManualControlSetpoint & ros_message = *static_cast<const __ros_msg_type_ManualControlSetpoint *>(untyped_ros_message);
  __dds_msg_type_ManualControlSetpoint dds_message;
  convert_ros_message_to_dds(ros_message, dds_message);

  px4_msgs::msg::dds_::ManualControlSetpoint_DataWriter * data_writer =
    px4_msgs::msg::dds_::ManualControlSetpoint_DataWriter::_narrow(topic_writer);
  DDS::ReturnCode_t status = data_writer->write(dds_message, DDS::HANDLE_NIL);
  switch (status) {
    case DDS::RETCODE_ERROR:
      return "px4_msgs::msg::dds_::ManualControlSetpoint_DataWriter.write: "
             "an internal error has occurred";
    case DDS::RETCODE_BAD_PARAMETER:
      return "px4_msgs::msg::dds_::ManualControlSetpoint_DataWriter.write: "
             "bad handle or instance_data parameter";
    case DDS::RETCODE_ALREADY_DELETED:
      return "px4_msgs::msg::dds_::ManualControlSetpoint_DataWriter.write: "
             "this px4_msgs::msg::dds_::ManualControlSetpoint_DataWriter has already been deleted";
    case DDS::RETCODE_OUT_OF_RESOURCES:
      return "px4_msgs::msg::dds_::ManualControlSetpoint_DataWriter.write: "
             "out of resources";
    case DDS::RETCODE_NOT_ENABLED:
      return "px4_msgs::msg::dds_::ManualControlSetpoint_DataWriter.write: "
             "this px4_msgs::msg::dds_::ManualControlSetpoint_DataWriter is not enabled";
    case DDS::RETCODE_PRECONDITION_NOT_MET:
      return "px4_msgs::msg::dds_::ManualControlSetpoint_DataWriter.write: "
             "the handle has not been registered with this px4_msgs::msg::dds_::ManualControlSetpoint_DataWriter";
    case DDS::RETCODE_TIMEOUT:
      return "px4_msgs::msg::dds_::ManualControlSetpoint_DataWriter.write: "
             "writing resulted in blocking and then exceeded the timeout set by the "
             "max_blocking_time of the ReliabilityQosPolicy";
    case DDS::RETCODE_OK:
      return nullptr;
    default:
      return "px4_msgs::msg::dds_::ManualControlSetpoint_DataWriter.write: unknown return code";
  }
}

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_EXPORT_px4_msgs
void
convert_dds_message_to_ros(
  const __dds_msg_type_ManualControlSetpoint & dds_message,
  __ros_msg_type_ManualControlSetpoint & ros_message)
{
  // member.name timestamp
  ros_message.timestamp =
    dds_message.timestamp_;
  // member.name timestamp_sample
  ros_message.timestamp_sample =
    dds_message.timestamp_sample_;
  // member.name valid
  ros_message.valid =
    (dds_message.valid_ != 0);
  // member.name data_source
  ros_message.data_source =
    dds_message.data_source_;
  // member.name roll
  ros_message.roll =
    dds_message.roll_;
  // member.name pitch
  ros_message.pitch =
    dds_message.pitch_;
  // member.name yaw
  ros_message.yaw =
    dds_message.yaw_;
  // member.name throttle
  ros_message.throttle =
    dds_message.throttle_;
  // member.name flaps
  ros_message.flaps =
    dds_message.flaps_;
  // member.name aux1
  ros_message.aux1 =
    dds_message.aux1_;
  // member.name aux2
  ros_message.aux2 =
    dds_message.aux2_;
  // member.name aux3
  ros_message.aux3 =
    dds_message.aux3_;
  // member.name aux4
  ros_message.aux4 =
    dds_message.aux4_;
  // member.name aux5
  ros_message.aux5 =
    dds_message.aux5_;
  // member.name aux6
  ros_message.aux6 =
    dds_message.aux6_;
  // member.name sticks_moving
  ros_message.sticks_moving =
    (dds_message.sticks_moving_ != 0);
  // member.name buttons
  ros_message.buttons =
    dds_message.buttons_;
}

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_EXPORT_px4_msgs
const char *
take__ManualControlSetpoint(
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

  px4_msgs::msg::dds_::ManualControlSetpoint_DataReader * data_reader =
    px4_msgs::msg::dds_::ManualControlSetpoint_DataReader::_narrow(topic_reader);

  px4_msgs::msg::dds_::ManualControlSetpoint_Seq dds_messages;
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
      errs = "px4_msgs::msg::dds_::ManualControlSetpoint_DataReader.take: "
        "an internal error has occurred";
      goto finally;
    case DDS::RETCODE_ALREADY_DELETED:
      errs = "px4_msgs::msg::dds_::ManualControlSetpoint_DataReader.take: "
        "this px4_msgs::msg::dds_::ManualControlSetpoint_DataReader has already been deleted";
      goto finally;
    case DDS::RETCODE_OUT_OF_RESOURCES:
      errs = "px4_msgs::msg::dds_::ManualControlSetpoint_DataReader.take: "
        "out of resources";
      goto finally;
    case DDS::RETCODE_NOT_ENABLED:
      errs = "px4_msgs::msg::dds_::ManualControlSetpoint_DataReader.take: "
        "this px4_msgs::msg::dds_::ManualControlSetpoint_DataReader is not enabled";
      goto finally;
    case DDS::RETCODE_PRECONDITION_NOT_MET:
      errs = "px4_msgs::msg::dds_::ManualControlSetpoint_DataReader.take: "
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
      errs = "px4_msgs::msg::dds_::ManualControlSetpoint_DataReader.take: unknown return code";
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
    __ros_msg_type_ManualControlSetpoint & ros_message = *static_cast<__ros_msg_type_ManualControlSetpoint *>(untyped_ros_message);
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
      return "px4_msgs::msg::dds_::ManualControlSetpoint_DataReader.return_loan: "
             "an internal error has occurred";
    case DDS::RETCODE_ALREADY_DELETED:
      return "px4_msgs::msg::dds_::ManualControlSetpoint_DataReader.return_loan: "
             "this px4_msgs::msg::dds_::ManualControlSetpoint_DataReader has already been deleted";
    case DDS::RETCODE_OUT_OF_RESOURCES:
      return "px4_msgs::msg::dds_::ManualControlSetpoint_DataReader.return_loan: "
             "out of resources";
    case DDS::RETCODE_NOT_ENABLED:
      return "px4_msgs::msg::dds_::ManualControlSetpoint_DataReader.return_loan: "
             "this px4_msgs::msg::dds_::ManualControlSetpoint_DataReader is not enabled";
    case DDS::RETCODE_PRECONDITION_NOT_MET:
      return "px4_msgs::msg::dds_::ManualControlSetpoint_DataReader.return_loan: "
             "a precondition is not met, one of: "
             "the data_values and info_seq do not belong to a single related pair, or "
             "the data_values and info_seq were not obtained from this "
             "px4_msgs::msg::dds_::ManualControlSetpoint_DataReader";
    case DDS::RETCODE_OK:
      break;
    default:
      return "px4_msgs::msg::dds_::ManualControlSetpoint_DataReader.return_loan failed with "
             "unknown return code";
  }

  return errs;
}

ROSIDL_TYPESUPPORT_OPENSPLICE_CPP_EXPORT_px4_msgs
const char *
serialize__ManualControlSetpoint(
  const void * untyped_ros_message,
  void * untyped_serialized_data)
{
  const __ros_msg_type_ManualControlSetpoint & ros_message = *static_cast<const __ros_msg_type_ManualControlSetpoint *>(untyped_ros_message);
  __dds_msg_type_ManualControlSetpoint dds_message;

  convert_ros_message_to_dds(ros_message, dds_message);

  DDS::OpenSplice::CdrTypeSupport cdr_ts(__type_support_ManualControlSetpoint);
  DDS::OpenSplice::CdrSerializedData * serdata = nullptr;

  DDS::ReturnCode_t status = cdr_ts.serialize(&dds_message, &serdata);
  switch (status) {
    case DDS::RETCODE_ERROR:
      return "px4_msgs::msg::dds_::ManualControlSetpoint_TypeSupport.serialize: "
             "an internal error has occurred";
    case DDS::RETCODE_BAD_PARAMETER:
      return "px4_msgs::msg::dds_::ManualControlSetpoint_TypeSupport.serialize: "
             "bad parameter";
    case DDS::RETCODE_ALREADY_DELETED:
      return "px4_msgs::msg::dds_::ManualControlSetpoint_TypeSupport.serialize: "
             "this px4_msgs::msg::dds_::ManualControlSetpoint_TypeSupport has already been deleted";
    case DDS::RETCODE_OUT_OF_RESOURCES:
      return "px4_msgs::msg::dds_::ManualControlSetpoint_TypeSupport.serialize: "
             "out of resources";
    case DDS::RETCODE_OK:
      break;
    default:
      return "px4_msgs::msg::dds_::ManualControlSetpoint_TypeSupport.serialize failed with "
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
      return "px4_msgs::msg::dds_::ManualControlSetpoint_TypeSupport.serialize: "
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
deserialize__ManualControlSetpoint(
  const uint8_t * buffer,
  unsigned length,
  void * untyped_ros_message)
{
  __dds_msg_type_ManualControlSetpoint dds_message;

  DDS::OpenSplice::CdrTypeSupport cdr_ts(__type_support_ManualControlSetpoint);

  DDS::ReturnCode_t status = cdr_ts.deserialize(buffer, length, &dds_message);

  switch (status) {
    case DDS::RETCODE_ERROR:
      return "px4_msgs::msg::dds_::ManualControlSetpoint_TypeSupport.deserialize: "
             "an internal error has occurred";
    case DDS::RETCODE_BAD_PARAMETER:
      return "px4_msgs::msg::dds_::ManualControlSetpoint_TypeSupport.deserialize: "
             "bad parameter";
    case DDS::RETCODE_ALREADY_DELETED:
      return "px4_msgs::msg::dds_::ManualControlSetpoint_TypeSupport.deserialize: "
             "this px4_msgs::msg::dds_::ManualControlSetpoint_TypeSupport has already been deleted";
    case DDS::RETCODE_OUT_OF_RESOURCES:
      return "px4_msgs::msg::dds_::ManualControlSetpoint_TypeSupport.deserialize: "
             "out of resources";
    case DDS::RETCODE_OK:
      break;
    default:
      return "px4_msgs::msg::dds_::ManualControlSetpoint_TypeSupport.deserialize failed with "
             "unknown return code";
  }

  __ros_msg_type_ManualControlSetpoint & ros_message = *static_cast<__ros_msg_type_ManualControlSetpoint *>(untyped_ros_message);
  convert_dds_message_to_ros(dds_message, ros_message);

  return nullptr;
}

static message_type_support_callbacks_t ManualControlSetpoint_callbacks = {
  "px4_msgs::msg",
  "ManualControlSetpoint",
  &register_type__ManualControlSetpoint,
  &publish__ManualControlSetpoint,
  &take__ManualControlSetpoint,
  &serialize__ManualControlSetpoint,
  &deserialize__ManualControlSetpoint,
  nullptr,  // convert ros to dds (handled differently for C++)
  nullptr,  // convert dds to ros (handled differently for C++)
};

static rosidl_message_type_support_t ManualControlSetpoint_handle = {
  rosidl_typesupport_opensplice_cpp::typesupport_identifier,
  &ManualControlSetpoint_callbacks,
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
get_message_type_support_handle<px4_msgs::msg::ManualControlSetpoint>()
{
  return &px4_msgs::msg::typesupport_opensplice_cpp::ManualControlSetpoint_handle;
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
  ManualControlSetpoint)()
{
  return &px4_msgs::msg::typesupport_opensplice_cpp::ManualControlSetpoint_handle;
}

#ifdef __cplusplus
}
#endif
