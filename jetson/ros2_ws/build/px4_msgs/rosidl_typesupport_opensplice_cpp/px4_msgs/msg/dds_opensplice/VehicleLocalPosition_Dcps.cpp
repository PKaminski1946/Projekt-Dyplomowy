#include "VehicleLocalPosition_.h"
#include "VehicleLocalPosition_Dcps.h"

namespace px4_msgs
{
    namespace msg
    {
        namespace dds_
        {
#if DDS_USE_EXPLICIT_TEMPLATES
template class DDS_DCPSUFLSeq < px4_msgs::msg::dds_::VehicleLocalPosition_, struct VehicleLocalPosition_Seq_uniq_>;
#endif

const char * px4_msgs::msg::dds_::VehicleLocalPosition_TypeSupportInterface::_local_id = "IDL:::px4_msgs::msg::dds_/VehicleLocalPosition_TypeSupportInterface:1.0";

px4_msgs::msg::dds_::VehicleLocalPosition_TypeSupportInterface_ptr px4_msgs::msg::dds_::VehicleLocalPosition_TypeSupportInterface::_duplicate (px4_msgs::msg::dds_::VehicleLocalPosition_TypeSupportInterface_ptr p)
{
    if (p) p->m_count++;
    return p;
}

DDS::Boolean px4_msgs::msg::dds_::VehicleLocalPosition_TypeSupportInterface::_local_is_a (const char * _id)
{
    if (strcmp (_id, px4_msgs::msg::dds_::VehicleLocalPosition_TypeSupportInterface::_local_id) == 0)
    {
        return true;
    }

    typedef DDS::TypeSupport NestedBase_1;

    if (NestedBase_1::_local_is_a (_id))
    {
        return true;
    }

    return false;
}

px4_msgs::msg::dds_::VehicleLocalPosition_TypeSupportInterface_ptr px4_msgs::msg::dds_::VehicleLocalPosition_TypeSupportInterface::_narrow (DDS::Object_ptr p)
{
    px4_msgs::msg::dds_::VehicleLocalPosition_TypeSupportInterface_ptr result = NULL;
    if (p && p->_is_a (px4_msgs::msg::dds_::VehicleLocalPosition_TypeSupportInterface::_local_id))
    {
        result = dynamic_cast < px4_msgs::msg::dds_::VehicleLocalPosition_TypeSupportInterface_ptr> (p);
        if (result) result->m_count++;
    }
    return result;
}

px4_msgs::msg::dds_::VehicleLocalPosition_TypeSupportInterface_ptr px4_msgs::msg::dds_::VehicleLocalPosition_TypeSupportInterface::_unchecked_narrow (DDS::Object_ptr p)
{
    px4_msgs::msg::dds_::VehicleLocalPosition_TypeSupportInterface_ptr result;
    result = dynamic_cast < px4_msgs::msg::dds_::VehicleLocalPosition_TypeSupportInterface_ptr> (p);
    if (result) result->m_count++;
    return result;
}

const char * px4_msgs::msg::dds_::VehicleLocalPosition_DataWriter::_local_id = "IDL:::px4_msgs::msg::dds_/VehicleLocalPosition_DataWriter:1.0";

px4_msgs::msg::dds_::VehicleLocalPosition_DataWriter_ptr px4_msgs::msg::dds_::VehicleLocalPosition_DataWriter::_duplicate (px4_msgs::msg::dds_::VehicleLocalPosition_DataWriter_ptr p)
{
    if (p) p->m_count++;
    return p;
}

DDS::Boolean px4_msgs::msg::dds_::VehicleLocalPosition_DataWriter::_local_is_a (const char * _id)
{
    if (strcmp (_id, px4_msgs::msg::dds_::VehicleLocalPosition_DataWriter::_local_id) == 0)
    {
        return true;
    }

    typedef DDS::DataWriter NestedBase_1;

    if (NestedBase_1::_local_is_a (_id))
    {
        return true;
    }

    return false;
}

px4_msgs::msg::dds_::VehicleLocalPosition_DataWriter_ptr px4_msgs::msg::dds_::VehicleLocalPosition_DataWriter::_narrow (DDS::Object_ptr p)
{
    px4_msgs::msg::dds_::VehicleLocalPosition_DataWriter_ptr result = NULL;
    if (p && p->_is_a (px4_msgs::msg::dds_::VehicleLocalPosition_DataWriter::_local_id))
    {
        result = dynamic_cast < px4_msgs::msg::dds_::VehicleLocalPosition_DataWriter_ptr> (p);
        if (result) result->m_count++;
    }
    return result;
}

px4_msgs::msg::dds_::VehicleLocalPosition_DataWriter_ptr px4_msgs::msg::dds_::VehicleLocalPosition_DataWriter::_unchecked_narrow (DDS::Object_ptr p)
{
    px4_msgs::msg::dds_::VehicleLocalPosition_DataWriter_ptr result;
    result = dynamic_cast < px4_msgs::msg::dds_::VehicleLocalPosition_DataWriter_ptr> (p);
    if (result) result->m_count++;
    return result;
}

const char * px4_msgs::msg::dds_::VehicleLocalPosition_DataReader::_local_id = "IDL:::px4_msgs::msg::dds_/VehicleLocalPosition_DataReader:1.0";

px4_msgs::msg::dds_::VehicleLocalPosition_DataReader_ptr px4_msgs::msg::dds_::VehicleLocalPosition_DataReader::_duplicate (px4_msgs::msg::dds_::VehicleLocalPosition_DataReader_ptr p)
{
    if (p) p->m_count++;
    return p;
}

DDS::Boolean px4_msgs::msg::dds_::VehicleLocalPosition_DataReader::_local_is_a (const char * _id)
{
    if (strcmp (_id, px4_msgs::msg::dds_::VehicleLocalPosition_DataReader::_local_id) == 0)
    {
        return true;
    }

    typedef DDS::DataReader NestedBase_1;

    if (NestedBase_1::_local_is_a (_id))
    {
        return true;
    }

    return false;
}

px4_msgs::msg::dds_::VehicleLocalPosition_DataReader_ptr px4_msgs::msg::dds_::VehicleLocalPosition_DataReader::_narrow (DDS::Object_ptr p)
{
    px4_msgs::msg::dds_::VehicleLocalPosition_DataReader_ptr result = NULL;
    if (p && p->_is_a (px4_msgs::msg::dds_::VehicleLocalPosition_DataReader::_local_id))
    {
        result = dynamic_cast < px4_msgs::msg::dds_::VehicleLocalPosition_DataReader_ptr> (p);
        if (result) result->m_count++;
    }
    return result;
}

px4_msgs::msg::dds_::VehicleLocalPosition_DataReader_ptr px4_msgs::msg::dds_::VehicleLocalPosition_DataReader::_unchecked_narrow (DDS::Object_ptr p)
{
    px4_msgs::msg::dds_::VehicleLocalPosition_DataReader_ptr result;
    result = dynamic_cast < px4_msgs::msg::dds_::VehicleLocalPosition_DataReader_ptr> (p);
    if (result) result->m_count++;
    return result;
}

const char * px4_msgs::msg::dds_::VehicleLocalPosition_DataReaderView::_local_id = "IDL:::px4_msgs::msg::dds_/VehicleLocalPosition_DataReaderView:1.0";

px4_msgs::msg::dds_::VehicleLocalPosition_DataReaderView_ptr px4_msgs::msg::dds_::VehicleLocalPosition_DataReaderView::_duplicate (px4_msgs::msg::dds_::VehicleLocalPosition_DataReaderView_ptr p)
{
    if (p) p->m_count++;
    return p;
}

DDS::Boolean px4_msgs::msg::dds_::VehicleLocalPosition_DataReaderView::_local_is_a (const char * _id)
{
    if (strcmp (_id, px4_msgs::msg::dds_::VehicleLocalPosition_DataReaderView::_local_id) == 0)
    {
        return true;
    }

    typedef DDS::DataReaderView NestedBase_1;

    if (NestedBase_1::_local_is_a (_id))
    {
        return true;
    }

    return false;
}

px4_msgs::msg::dds_::VehicleLocalPosition_DataReaderView_ptr px4_msgs::msg::dds_::VehicleLocalPosition_DataReaderView::_narrow (DDS::Object_ptr p)
{
    px4_msgs::msg::dds_::VehicleLocalPosition_DataReaderView_ptr result = NULL;
    if (p && p->_is_a (px4_msgs::msg::dds_::VehicleLocalPosition_DataReaderView::_local_id))
    {
        result = dynamic_cast < px4_msgs::msg::dds_::VehicleLocalPosition_DataReaderView_ptr> (p);
        if (result) result->m_count++;
    }
    return result;
}

px4_msgs::msg::dds_::VehicleLocalPosition_DataReaderView_ptr px4_msgs::msg::dds_::VehicleLocalPosition_DataReaderView::_unchecked_narrow (DDS::Object_ptr p)
{
    px4_msgs::msg::dds_::VehicleLocalPosition_DataReaderView_ptr result;
    result = dynamic_cast < px4_msgs::msg::dds_::VehicleLocalPosition_DataReaderView_ptr> (p);
    if (result) result->m_count++;
    return result;
}

        }

    }

}

