#ifndef H_8A825DA96616D2E7EFE699F458435BD8_ManualControlSetpoint_SPLTYPES_H
#define H_8A825DA96616D2E7EFE699F458435BD8_ManualControlSetpoint_SPLTYPES_H

#include <c_base.h>
#include <c_misc.h>
#include <c_sync.h>
#include <c_collection.h>
#include <c_field.h>
#include <v_copyIn.h>

#include "ccpp_ManualControlSetpoint_.h"


extern c_metaObject __ManualControlSetpoint__px4_msgs__load (c_base base);

extern c_metaObject __ManualControlSetpoint__px4_msgs_msg__load (c_base base);

extern c_metaObject __ManualControlSetpoint__px4_msgs_msg_dds___load (c_base base);

extern c_metaObject __ManualControlSetpoint__px4_msgs_msg_dds__ManualControlSetpoint_Constants__load (c_base base);

extern const char *px4_msgs_msg_dds__ManualControlSetpoint__metaDescriptor[];
extern const int px4_msgs_msg_dds__ManualControlSetpoint__metaDescriptorArrLength;
extern const int px4_msgs_msg_dds__ManualControlSetpoint__metaDescriptorLength;
extern c_metaObject __px4_msgs_msg_dds__ManualControlSetpoint___load (c_base base);
struct _px4_msgs_msg_dds__ManualControlSetpoint_ ;
extern  v_copyin_result __px4_msgs_msg_dds__ManualControlSetpoint___copyIn(c_base base, const struct px4_msgs::msg::dds_::ManualControlSetpoint_ *from, struct _px4_msgs_msg_dds__ManualControlSetpoint_ *to);
extern  void __px4_msgs_msg_dds__ManualControlSetpoint___copyOut(const void *_from, void *_to);
struct _px4_msgs_msg_dds__ManualControlSetpoint_ {
    c_ulonglong timestamp_;
    c_ulonglong timestamp_sample_;
    c_bool valid_;
    c_octet data_source_;
    c_float roll_;
    c_float pitch_;
    c_float yaw_;
    c_float throttle_;
    c_float flaps_;
    c_float aux1_;
    c_float aux2_;
    c_float aux3_;
    c_float aux4_;
    c_float aux5_;
    c_float aux6_;
    c_bool sticks_moving_;
    c_ushort buttons_;
};

#undef OS_API
#endif
