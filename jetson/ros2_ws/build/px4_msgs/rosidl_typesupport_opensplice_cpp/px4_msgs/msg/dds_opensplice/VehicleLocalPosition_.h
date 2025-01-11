#ifndef _H_1B404A462FB1A43AEEB9164E37331DD9_VehicleLocalPosition__H_
#define _H_1B404A462FB1A43AEEB9164E37331DD9_VehicleLocalPosition__H_

#include "sacpp_mapping.h"
#include "cpp_dcps_if.h"


namespace px4_msgs
{
    namespace msg
    {
        namespace dds_
        {
            namespace VehicleLocalPosition_Constants
            {
                const ::DDS::Octet DIST_BOTTOM_SENSOR_NONE_ = 0;

                const ::DDS::Octet DIST_BOTTOM_SENSOR_RANGE_ = 1;

                const ::DDS::Octet DIST_BOTTOM_SENSOR_FLOW_ = 2;

            }

            struct  VehicleLocalPosition_
            {
                struct _delta_xy__uniq_{};
                typedef ::DDS::Float _delta_xy__slice;
                typedef ::DDS::Float _delta_xy_[2];
                typedef DDS_DCPS_FArray_var< _delta_xy_, _delta_xy__slice, struct _delta_xy__uniq_ > _delta_xy__var;
                typedef _delta_xy_ _delta_xy__out;
                typedef DDS_DCPS_Array_forany< _delta_xy_, _delta_xy__slice, struct _delta_xy__uniq_ > _delta_xy__forany;
                static _delta_xy__slice *_delta_xy__alloc();
                static void _delta_xy__free(_delta_xy__slice *);
                static void _delta_xy__copy(_delta_xy__slice *to, const _delta_xy__slice *from);
                static _delta_xy__slice *_delta_xy__dup(const _delta_xy__slice *from);
                struct _delta_vxy__uniq_{};
                typedef ::DDS::Float _delta_vxy__slice;
                typedef ::DDS::Float _delta_vxy_[2];
                typedef DDS_DCPS_FArray_var< _delta_vxy_, _delta_vxy__slice, struct _delta_vxy__uniq_ > _delta_vxy__var;
                typedef _delta_vxy_ _delta_vxy__out;
                typedef DDS_DCPS_Array_forany< _delta_vxy_, _delta_vxy__slice, struct _delta_vxy__uniq_ > _delta_vxy__forany;
                static _delta_vxy__slice *_delta_vxy__alloc();
                static void _delta_vxy__free(_delta_vxy__slice *);
                static void _delta_vxy__copy(_delta_vxy__slice *to, const _delta_vxy__slice *from);
                static _delta_vxy__slice *_delta_vxy__dup(const _delta_vxy__slice *from);
                ::DDS::ULongLong timestamp_;
                ::DDS::ULongLong timestamp_sample_;
                ::DDS::Boolean xy_valid_;
                ::DDS::Boolean z_valid_;
                ::DDS::Boolean v_xy_valid_;
                ::DDS::Boolean v_z_valid_;
                ::DDS::Float x_;
                ::DDS::Float y_;
                ::DDS::Float z_;
                _delta_xy_ delta_xy_;
                ::DDS::Octet xy_reset_counter_;
                ::DDS::Float delta_z_;
                ::DDS::Octet z_reset_counter_;
                ::DDS::Float vx_;
                ::DDS::Float vy_;
                ::DDS::Float vz_;
                ::DDS::Float z_deriv_;
                _delta_vxy_ delta_vxy_;
                ::DDS::Octet vxy_reset_counter_;
                ::DDS::Float delta_vz_;
                ::DDS::Octet vz_reset_counter_;
                ::DDS::Float ax_;
                ::DDS::Float ay_;
                ::DDS::Float az_;
                ::DDS::Float heading_;
                ::DDS::Float heading_var_;
                ::DDS::Float unaided_heading_;
                ::DDS::Float delta_heading_;
                ::DDS::Octet heading_reset_counter_;
                ::DDS::Boolean heading_good_for_control_;
                ::DDS::Float tilt_var_;
                ::DDS::Boolean xy_global_;
                ::DDS::Boolean z_global_;
                ::DDS::ULongLong ref_timestamp_;
                ::DDS::Double ref_lat_;
                ::DDS::Double ref_lon_;
                ::DDS::Float ref_alt_;
                ::DDS::Boolean dist_bottom_valid_;
                ::DDS::Float dist_bottom_;
                ::DDS::Float dist_bottom_var_;
                ::DDS::Float delta_dist_bottom_;
                ::DDS::Octet dist_bottom_reset_counter_;
                ::DDS::Octet dist_bottom_sensor_bitfield_;
                ::DDS::Float eph_;
                ::DDS::Float epv_;
                ::DDS::Float evh_;
                ::DDS::Float evv_;
                ::DDS::Boolean dead_reckoning_;
                ::DDS::Float vxy_max_;
                ::DDS::Float vz_max_;
                ::DDS::Float hagl_min_;
                ::DDS::Float hagl_max_;
            };

            typedef DDS_DCPSStruct_var<VehicleLocalPosition_> VehicleLocalPosition__var;
            typedef VehicleLocalPosition_& VehicleLocalPosition__out;

        }

    }

}

#endif /* _H_1B404A462FB1A43AEEB9164E37331DD9_VehicleLocalPosition__H_ */
