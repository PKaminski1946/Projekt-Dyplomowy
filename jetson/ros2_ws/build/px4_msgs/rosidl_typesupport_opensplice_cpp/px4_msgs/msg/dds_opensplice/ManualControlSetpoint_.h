#ifndef _H_8A825DA96616D2E7EFE699F458435BD8_ManualControlSetpoint__H_
#define _H_8A825DA96616D2E7EFE699F458435BD8_ManualControlSetpoint__H_

#include "sacpp_mapping.h"
#include "cpp_dcps_if.h"


namespace px4_msgs
{
    namespace msg
    {
        namespace dds_
        {
            namespace ManualControlSetpoint_Constants
            {
                const ::DDS::Octet SOURCE_UNKNOWN_ = 0;

                const ::DDS::Octet SOURCE_RC_ = 1;

                const ::DDS::Octet SOURCE_MAVLINK_0_ = 2;

                const ::DDS::Octet SOURCE_MAVLINK_1_ = 3;

                const ::DDS::Octet SOURCE_MAVLINK_2_ = 4;

                const ::DDS::Octet SOURCE_MAVLINK_3_ = 5;

                const ::DDS::Octet SOURCE_MAVLINK_4_ = 6;

                const ::DDS::Octet SOURCE_MAVLINK_5_ = 7;

            }

            struct  ManualControlSetpoint_
            {
                ::DDS::ULongLong timestamp_;
                ::DDS::ULongLong timestamp_sample_;
                ::DDS::Boolean valid_;
                ::DDS::Octet data_source_;
                ::DDS::Float roll_;
                ::DDS::Float pitch_;
                ::DDS::Float yaw_;
                ::DDS::Float throttle_;
                ::DDS::Float flaps_;
                ::DDS::Float aux1_;
                ::DDS::Float aux2_;
                ::DDS::Float aux3_;
                ::DDS::Float aux4_;
                ::DDS::Float aux5_;
                ::DDS::Float aux6_;
                ::DDS::Boolean sticks_moving_;
                ::DDS::UShort buttons_;
            };

            typedef DDS_DCPSStruct_var<ManualControlSetpoint_> ManualControlSetpoint__var;
            typedef ManualControlSetpoint_& ManualControlSetpoint__out;

        }

    }

}

#endif /* _H_8A825DA96616D2E7EFE699F458435BD8_ManualControlSetpoint__H_ */
