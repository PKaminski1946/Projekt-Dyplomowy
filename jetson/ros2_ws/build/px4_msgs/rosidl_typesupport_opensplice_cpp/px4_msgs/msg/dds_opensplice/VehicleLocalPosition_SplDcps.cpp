#include "VehicleLocalPosition_SplDcps.h"
#include "ccpp_VehicleLocalPosition_.h"

#include <v_copyIn.h>
#include <v_topic.h>
#include <os_stdlib.h>
#include <string.h>
#include <os_report.h>

v_copyin_result
__px4_msgs_msg_dds__VehicleLocalPosition___copyIn(
    c_base base,
    const struct ::px4_msgs::msg::dds_::VehicleLocalPosition_ *from,
    struct _px4_msgs_msg_dds__VehicleLocalPosition_ *to)
{
    v_copyin_result result = V_COPYIN_RESULT_OK;
    (void) base;

    to->timestamp_ = (c_ulonglong)from->timestamp_;
    to->timestamp_sample_ = (c_ulonglong)from->timestamp_sample_;
    to->xy_valid_ = (c_bool)from->xy_valid_;
    to->z_valid_ = (c_bool)from->z_valid_;
    to->v_xy_valid_ = (c_bool)from->v_xy_valid_;
    to->v_z_valid_ = (c_bool)from->v_z_valid_;
    to->x_ = (c_float)from->x_;
    to->y_ = (c_float)from->y_;
    to->z_ = (c_float)from->z_;
    {
        typedef c_float _DestType[2];
        _DestType *dest = &to->delta_xy_;
        memcpy (dest, from->delta_xy_, sizeof (*dest));
    }
    to->xy_reset_counter_ = (c_octet)from->xy_reset_counter_;
    to->delta_z_ = (c_float)from->delta_z_;
    to->z_reset_counter_ = (c_octet)from->z_reset_counter_;
    to->vx_ = (c_float)from->vx_;
    to->vy_ = (c_float)from->vy_;
    to->vz_ = (c_float)from->vz_;
    to->z_deriv_ = (c_float)from->z_deriv_;
    {
        typedef c_float _DestType[2];
        _DestType *dest = &to->delta_vxy_;
        memcpy (dest, from->delta_vxy_, sizeof (*dest));
    }
    to->vxy_reset_counter_ = (c_octet)from->vxy_reset_counter_;
    to->delta_vz_ = (c_float)from->delta_vz_;
    to->vz_reset_counter_ = (c_octet)from->vz_reset_counter_;
    to->ax_ = (c_float)from->ax_;
    to->ay_ = (c_float)from->ay_;
    to->az_ = (c_float)from->az_;
    to->heading_ = (c_float)from->heading_;
    to->heading_var_ = (c_float)from->heading_var_;
    to->unaided_heading_ = (c_float)from->unaided_heading_;
    to->delta_heading_ = (c_float)from->delta_heading_;
    to->heading_reset_counter_ = (c_octet)from->heading_reset_counter_;
    to->heading_good_for_control_ = (c_bool)from->heading_good_for_control_;
    to->tilt_var_ = (c_float)from->tilt_var_;
    to->xy_global_ = (c_bool)from->xy_global_;
    to->z_global_ = (c_bool)from->z_global_;
    to->ref_timestamp_ = (c_ulonglong)from->ref_timestamp_;
    to->ref_lat_ = (c_double)from->ref_lat_;
    to->ref_lon_ = (c_double)from->ref_lon_;
    to->ref_alt_ = (c_float)from->ref_alt_;
    to->dist_bottom_valid_ = (c_bool)from->dist_bottom_valid_;
    to->dist_bottom_ = (c_float)from->dist_bottom_;
    to->dist_bottom_var_ = (c_float)from->dist_bottom_var_;
    to->delta_dist_bottom_ = (c_float)from->delta_dist_bottom_;
    to->dist_bottom_reset_counter_ = (c_octet)from->dist_bottom_reset_counter_;
    to->dist_bottom_sensor_bitfield_ = (c_octet)from->dist_bottom_sensor_bitfield_;
    to->eph_ = (c_float)from->eph_;
    to->epv_ = (c_float)from->epv_;
    to->evh_ = (c_float)from->evh_;
    to->evv_ = (c_float)from->evv_;
    to->dead_reckoning_ = (c_bool)from->dead_reckoning_;
    to->vxy_max_ = (c_float)from->vxy_max_;
    to->vz_max_ = (c_float)from->vz_max_;
    to->hagl_min_ = (c_float)from->hagl_min_;
    to->hagl_max_ = (c_float)from->hagl_max_;
    return result;
}

void
__px4_msgs_msg_dds__VehicleLocalPosition___copyOut(
    const void *_from,
    void *_to)
{
    const struct _px4_msgs_msg_dds__VehicleLocalPosition_ *from = (const struct _px4_msgs_msg_dds__VehicleLocalPosition_ *)_from;
    struct ::px4_msgs::msg::dds_::VehicleLocalPosition_ *to = (struct ::px4_msgs::msg::dds_::VehicleLocalPosition_ *)_to;
    to->timestamp_ = (::DDS::ULongLong)from->timestamp_;
    to->timestamp_sample_ = (::DDS::ULongLong)from->timestamp_sample_;
    to->xy_valid_ = (::DDS::Boolean)(from->xy_valid_ != 0);
    to->z_valid_ = (::DDS::Boolean)(from->z_valid_ != 0);
    to->v_xy_valid_ = (::DDS::Boolean)(from->v_xy_valid_ != 0);
    to->v_z_valid_ = (::DDS::Boolean)(from->v_z_valid_ != 0);
    to->x_ = (::DDS::Float)from->x_;
    to->y_ = (::DDS::Float)from->y_;
    to->z_ = (::DDS::Float)from->z_;
    {
        typedef c_float _DestType[2];
        const _DestType *src = &from->delta_xy_;

        memcpy ((void *)to->delta_xy_, src, sizeof (*src));
    }
    to->xy_reset_counter_ = (::DDS::Octet)from->xy_reset_counter_;
    to->delta_z_ = (::DDS::Float)from->delta_z_;
    to->z_reset_counter_ = (::DDS::Octet)from->z_reset_counter_;
    to->vx_ = (::DDS::Float)from->vx_;
    to->vy_ = (::DDS::Float)from->vy_;
    to->vz_ = (::DDS::Float)from->vz_;
    to->z_deriv_ = (::DDS::Float)from->z_deriv_;
    {
        typedef c_float _DestType[2];
        const _DestType *src = &from->delta_vxy_;

        memcpy ((void *)to->delta_vxy_, src, sizeof (*src));
    }
    to->vxy_reset_counter_ = (::DDS::Octet)from->vxy_reset_counter_;
    to->delta_vz_ = (::DDS::Float)from->delta_vz_;
    to->vz_reset_counter_ = (::DDS::Octet)from->vz_reset_counter_;
    to->ax_ = (::DDS::Float)from->ax_;
    to->ay_ = (::DDS::Float)from->ay_;
    to->az_ = (::DDS::Float)from->az_;
    to->heading_ = (::DDS::Float)from->heading_;
    to->heading_var_ = (::DDS::Float)from->heading_var_;
    to->unaided_heading_ = (::DDS::Float)from->unaided_heading_;
    to->delta_heading_ = (::DDS::Float)from->delta_heading_;
    to->heading_reset_counter_ = (::DDS::Octet)from->heading_reset_counter_;
    to->heading_good_for_control_ = (::DDS::Boolean)(from->heading_good_for_control_ != 0);
    to->tilt_var_ = (::DDS::Float)from->tilt_var_;
    to->xy_global_ = (::DDS::Boolean)(from->xy_global_ != 0);
    to->z_global_ = (::DDS::Boolean)(from->z_global_ != 0);
    to->ref_timestamp_ = (::DDS::ULongLong)from->ref_timestamp_;
    to->ref_lat_ = (::DDS::Double)from->ref_lat_;
    to->ref_lon_ = (::DDS::Double)from->ref_lon_;
    to->ref_alt_ = (::DDS::Float)from->ref_alt_;
    to->dist_bottom_valid_ = (::DDS::Boolean)(from->dist_bottom_valid_ != 0);
    to->dist_bottom_ = (::DDS::Float)from->dist_bottom_;
    to->dist_bottom_var_ = (::DDS::Float)from->dist_bottom_var_;
    to->delta_dist_bottom_ = (::DDS::Float)from->delta_dist_bottom_;
    to->dist_bottom_reset_counter_ = (::DDS::Octet)from->dist_bottom_reset_counter_;
    to->dist_bottom_sensor_bitfield_ = (::DDS::Octet)from->dist_bottom_sensor_bitfield_;
    to->eph_ = (::DDS::Float)from->eph_;
    to->epv_ = (::DDS::Float)from->epv_;
    to->evh_ = (::DDS::Float)from->evh_;
    to->evv_ = (::DDS::Float)from->evv_;
    to->dead_reckoning_ = (::DDS::Boolean)(from->dead_reckoning_ != 0);
    to->vxy_max_ = (::DDS::Float)from->vxy_max_;
    to->vz_max_ = (::DDS::Float)from->vz_max_;
    to->hagl_min_ = (::DDS::Float)from->hagl_min_;
    to->hagl_max_ = (::DDS::Float)from->hagl_max_;
}

