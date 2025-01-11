#include "VehicleLocalPosition_.h"

#if DDS_USE_EXPLICIT_TEMPLATES
template class DDS_DCPS_FArray_var< ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_xy_, ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_xy__slice, struct ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_xy__uniq_>;
template class DDS_DCPS_Array_forany< ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_xy_, ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_xy__slice, struct ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_xy__uniq_>;
#endif

template <>
::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_xy__slice* DDS_DCPS_ArrayHelper < ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_xy_, ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_xy__slice, ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_xy__uniq_>::alloc ()
{
    return ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_xy__alloc ();
}

template <>
void DDS_DCPS_ArrayHelper < ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_xy_, ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_xy__slice, ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_xy__uniq_>::copy (::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_xy__slice *to, const ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_xy__slice* from)
{
    ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_xy__copy (to, from);
}

template <>
void DDS_DCPS_ArrayHelper < ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_xy_, ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_xy__slice, ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_xy__uniq_>::free (::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_xy__slice *ptr)
{
    ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_xy__free(ptr);
}

::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_xy__slice * ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_xy__alloc ()
{
    ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_xy__slice * ret = new ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_xy__slice[2];
    return ret;
}

void ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_xy__free (::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_xy__slice * s)
{
    delete [] s;
}

void ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_xy__copy(::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_xy__slice * to, const ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_xy__slice * from)
{
    for (DDS::ULong i0 = 0; i0 < 2; i0++) {
        to[i0] = from[i0];
    }
}

::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_xy__slice * ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_xy__dup(const ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_xy__slice * from)
{
    ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_xy__slice * to = ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_xy__alloc ();
    ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_xy__copy (to, from);
    return to;
}

#if DDS_USE_EXPLICIT_TEMPLATES
template class DDS_DCPS_FArray_var< ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_vxy_, ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_vxy__slice, struct ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_vxy__uniq_>;
template class DDS_DCPS_Array_forany< ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_vxy_, ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_vxy__slice, struct ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_vxy__uniq_>;
#endif

template <>
::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_vxy__slice* DDS_DCPS_ArrayHelper < ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_vxy_, ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_vxy__slice, ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_vxy__uniq_>::alloc ()
{
    return ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_vxy__alloc ();
}

template <>
void DDS_DCPS_ArrayHelper < ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_vxy_, ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_vxy__slice, ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_vxy__uniq_>::copy (::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_vxy__slice *to, const ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_vxy__slice* from)
{
    ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_vxy__copy (to, from);
}

template <>
void DDS_DCPS_ArrayHelper < ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_vxy_, ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_vxy__slice, ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_vxy__uniq_>::free (::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_vxy__slice *ptr)
{
    ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_vxy__free(ptr);
}

::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_vxy__slice * ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_vxy__alloc ()
{
    ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_vxy__slice * ret = new ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_vxy__slice[2];
    return ret;
}

void ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_vxy__free (::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_vxy__slice * s)
{
    delete [] s;
}

void ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_vxy__copy(::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_vxy__slice * to, const ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_vxy__slice * from)
{
    for (DDS::ULong i0 = 0; i0 < 2; i0++) {
        to[i0] = from[i0];
    }
}

::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_vxy__slice * ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_vxy__dup(const ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_vxy__slice * from)
{
    ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_vxy__slice * to = ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_vxy__alloc ();
    ::px4_msgs::msg::dds_::VehicleLocalPosition_::_delta_vxy__copy (to, from);
    return to;
}

