#include "ManualControlSetpoint_SplDcps.h"
#include "ccpp_ManualControlSetpoint_.h"

#include <v_copyIn.h>
#include <v_topic.h>
#include <os_stdlib.h>
#include <string.h>
#include <os_report.h>

v_copyin_result
__px4_msgs_msg_dds__ManualControlSetpoint___copyIn(
    c_base base,
    const struct ::px4_msgs::msg::dds_::ManualControlSetpoint_ *from,
    struct _px4_msgs_msg_dds__ManualControlSetpoint_ *to)
{
    v_copyin_result result = V_COPYIN_RESULT_OK;
    (void) base;

    to->timestamp_ = (c_ulonglong)from->timestamp_;
    to->timestamp_sample_ = (c_ulonglong)from->timestamp_sample_;
    to->valid_ = (c_bool)from->valid_;
    to->data_source_ = (c_octet)from->data_source_;
    to->roll_ = (c_float)from->roll_;
    to->pitch_ = (c_float)from->pitch_;
    to->yaw_ = (c_float)from->yaw_;
    to->throttle_ = (c_float)from->throttle_;
    to->flaps_ = (c_float)from->flaps_;
    to->aux1_ = (c_float)from->aux1_;
    to->aux2_ = (c_float)from->aux2_;
    to->aux3_ = (c_float)from->aux3_;
    to->aux4_ = (c_float)from->aux4_;
    to->aux5_ = (c_float)from->aux5_;
    to->aux6_ = (c_float)from->aux6_;
    to->sticks_moving_ = (c_bool)from->sticks_moving_;
    to->buttons_ = (c_ushort)from->buttons_;
    return result;
}

void
__px4_msgs_msg_dds__ManualControlSetpoint___copyOut(
    const void *_from,
    void *_to)
{
    const struct _px4_msgs_msg_dds__ManualControlSetpoint_ *from = (const struct _px4_msgs_msg_dds__ManualControlSetpoint_ *)_from;
    struct ::px4_msgs::msg::dds_::ManualControlSetpoint_ *to = (struct ::px4_msgs::msg::dds_::ManualControlSetpoint_ *)_to;
    to->timestamp_ = (::DDS::ULongLong)from->timestamp_;
    to->timestamp_sample_ = (::DDS::ULongLong)from->timestamp_sample_;
    to->valid_ = (::DDS::Boolean)(from->valid_ != 0);
    to->data_source_ = (::DDS::Octet)from->data_source_;
    to->roll_ = (::DDS::Float)from->roll_;
    to->pitch_ = (::DDS::Float)from->pitch_;
    to->yaw_ = (::DDS::Float)from->yaw_;
    to->throttle_ = (::DDS::Float)from->throttle_;
    to->flaps_ = (::DDS::Float)from->flaps_;
    to->aux1_ = (::DDS::Float)from->aux1_;
    to->aux2_ = (::DDS::Float)from->aux2_;
    to->aux3_ = (::DDS::Float)from->aux3_;
    to->aux4_ = (::DDS::Float)from->aux4_;
    to->aux5_ = (::DDS::Float)from->aux5_;
    to->aux6_ = (::DDS::Float)from->aux6_;
    to->sticks_moving_ = (::DDS::Boolean)(from->sticks_moving_ != 0);
    to->buttons_ = (::DDS::UShort)from->buttons_;
}

