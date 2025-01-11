#ifndef H_1B404A462FB1A43AEEB9164E37331DD9_VehicleLocalPosition_SPLTYPES_H
#define H_1B404A462FB1A43AEEB9164E37331DD9_VehicleLocalPosition_SPLTYPES_H

#include <c_base.h>
#include <c_misc.h>
#include <c_sync.h>
#include <c_collection.h>
#include <c_field.h>
#include <v_copyIn.h>

#include "ccpp_VehicleLocalPosition_.h"


extern c_metaObject __VehicleLocalPosition__px4_msgs__load (c_base base);

extern c_metaObject __VehicleLocalPosition__px4_msgs_msg__load (c_base base);

extern c_metaObject __VehicleLocalPosition__px4_msgs_msg_dds___load (c_base base);

extern c_metaObject __VehicleLocalPosition__px4_msgs_msg_dds__VehicleLocalPosition_Constants__load (c_base base);

extern const char *px4_msgs_msg_dds__VehicleLocalPosition__metaDescriptor[];
extern const int px4_msgs_msg_dds__VehicleLocalPosition__metaDescriptorArrLength;
extern const int px4_msgs_msg_dds__VehicleLocalPosition__metaDescriptorLength;
extern c_metaObject __px4_msgs_msg_dds__VehicleLocalPosition___load (c_base base);
struct _px4_msgs_msg_dds__VehicleLocalPosition_ ;
extern  v_copyin_result __px4_msgs_msg_dds__VehicleLocalPosition___copyIn(c_base base, const struct px4_msgs::msg::dds_::VehicleLocalPosition_ *from, struct _px4_msgs_msg_dds__VehicleLocalPosition_ *to);
extern  void __px4_msgs_msg_dds__VehicleLocalPosition___copyOut(const void *_from, void *_to);
struct _px4_msgs_msg_dds__VehicleLocalPosition_ {
    c_ulonglong timestamp_;
    c_ulonglong timestamp_sample_;
    c_bool xy_valid_;
    c_bool z_valid_;
    c_bool v_xy_valid_;
    c_bool v_z_valid_;
    c_float x_;
    c_float y_;
    c_float z_;
    c_float delta_xy_[2];
    c_octet xy_reset_counter_;
    c_float delta_z_;
    c_octet z_reset_counter_;
    c_float vx_;
    c_float vy_;
    c_float vz_;
    c_float z_deriv_;
    c_float delta_vxy_[2];
    c_octet vxy_reset_counter_;
    c_float delta_vz_;
    c_octet vz_reset_counter_;
    c_float ax_;
    c_float ay_;
    c_float az_;
    c_float heading_;
    c_float heading_var_;
    c_float unaided_heading_;
    c_float delta_heading_;
    c_octet heading_reset_counter_;
    c_bool heading_good_for_control_;
    c_float tilt_var_;
    c_bool xy_global_;
    c_bool z_global_;
    c_ulonglong ref_timestamp_;
    c_double ref_lat_;
    c_double ref_lon_;
    c_float ref_alt_;
    c_bool dist_bottom_valid_;
    c_float dist_bottom_;
    c_float dist_bottom_var_;
    c_float delta_dist_bottom_;
    c_octet dist_bottom_reset_counter_;
    c_octet dist_bottom_sensor_bitfield_;
    c_float eph_;
    c_float epv_;
    c_float evh_;
    c_float evv_;
    c_bool dead_reckoning_;
    c_float vxy_max_;
    c_float vz_max_;
    c_float hagl_min_;
    c_float hagl_max_;
};

#undef OS_API
#endif
