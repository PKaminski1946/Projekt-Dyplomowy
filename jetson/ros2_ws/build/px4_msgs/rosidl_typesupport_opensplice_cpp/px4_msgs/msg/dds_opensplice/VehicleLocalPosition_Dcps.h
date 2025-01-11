#ifndef _H_1B404A462FB1A43AEEB9164E37331DD9_VehicleLocalPosition_DCPS_H_
#define _H_1B404A462FB1A43AEEB9164E37331DD9_VehicleLocalPosition_DCPS_H_

#include "sacpp_mapping.h"
#include "dds_dcps.h"
#include "VehicleLocalPosition_.h"


namespace px4_msgs
{
    namespace msg
    {
        namespace dds_
        {
            class VehicleLocalPosition_TypeSupportInterface;

            typedef VehicleLocalPosition_TypeSupportInterface * VehicleLocalPosition_TypeSupportInterface_ptr;
            typedef DDS_DCPSInterface_var < VehicleLocalPosition_TypeSupportInterface> VehicleLocalPosition_TypeSupportInterface_var;
            typedef DDS_DCPSInterface_out < VehicleLocalPosition_TypeSupportInterface> VehicleLocalPosition_TypeSupportInterface_out;


            class VehicleLocalPosition_DataWriter;

            typedef VehicleLocalPosition_DataWriter * VehicleLocalPosition_DataWriter_ptr;
            typedef DDS_DCPSInterface_var < VehicleLocalPosition_DataWriter> VehicleLocalPosition_DataWriter_var;
            typedef DDS_DCPSInterface_out < VehicleLocalPosition_DataWriter> VehicleLocalPosition_DataWriter_out;


            class VehicleLocalPosition_DataReader;

            typedef VehicleLocalPosition_DataReader * VehicleLocalPosition_DataReader_ptr;
            typedef DDS_DCPSInterface_var < VehicleLocalPosition_DataReader> VehicleLocalPosition_DataReader_var;
            typedef DDS_DCPSInterface_out < VehicleLocalPosition_DataReader> VehicleLocalPosition_DataReader_out;


            class VehicleLocalPosition_DataReaderView;

            typedef VehicleLocalPosition_DataReaderView * VehicleLocalPosition_DataReaderView_ptr;
            typedef DDS_DCPSInterface_var < VehicleLocalPosition_DataReaderView> VehicleLocalPosition_DataReaderView_var;
            typedef DDS_DCPSInterface_out < VehicleLocalPosition_DataReaderView> VehicleLocalPosition_DataReaderView_out;

            struct VehicleLocalPosition_Seq_uniq_ {};
            typedef DDS_DCPSUFLSeq < VehicleLocalPosition_, struct VehicleLocalPosition_Seq_uniq_> VehicleLocalPosition_Seq;
            typedef DDS_DCPSSequence_var < VehicleLocalPosition_Seq> VehicleLocalPosition_Seq_var;
            typedef DDS_DCPSSequence_out < VehicleLocalPosition_Seq> VehicleLocalPosition_Seq_out;

            class  VehicleLocalPosition_TypeSupportInterface :
                virtual public DDS::TypeSupport
            { 
            public:
                typedef VehicleLocalPosition_TypeSupportInterface_ptr _ptr_type;
                typedef VehicleLocalPosition_TypeSupportInterface_var _var_type;

                static VehicleLocalPosition_TypeSupportInterface_ptr _duplicate (VehicleLocalPosition_TypeSupportInterface_ptr obj);
                DDS::Boolean _local_is_a (const char * id);

                static VehicleLocalPosition_TypeSupportInterface_ptr _narrow (DDS::Object_ptr obj);
                static VehicleLocalPosition_TypeSupportInterface_ptr _unchecked_narrow (DDS::Object_ptr obj);
                static VehicleLocalPosition_TypeSupportInterface_ptr _nil () { return 0; }
                static const char * _local_id;
                VehicleLocalPosition_TypeSupportInterface_ptr _this () { return this; }


            protected:
                VehicleLocalPosition_TypeSupportInterface () {};
                ~VehicleLocalPosition_TypeSupportInterface () {};
            private:
                VehicleLocalPosition_TypeSupportInterface (const VehicleLocalPosition_TypeSupportInterface &);
                VehicleLocalPosition_TypeSupportInterface & operator = (const VehicleLocalPosition_TypeSupportInterface &);
            };

            class  VehicleLocalPosition_DataWriter :
                virtual public DDS::DataWriter
            { 
            public:
                typedef VehicleLocalPosition_DataWriter_ptr _ptr_type;
                typedef VehicleLocalPosition_DataWriter_var _var_type;

                static VehicleLocalPosition_DataWriter_ptr _duplicate (VehicleLocalPosition_DataWriter_ptr obj);
                DDS::Boolean _local_is_a (const char * id);

                static VehicleLocalPosition_DataWriter_ptr _narrow (DDS::Object_ptr obj);
                static VehicleLocalPosition_DataWriter_ptr _unchecked_narrow (DDS::Object_ptr obj);
                static VehicleLocalPosition_DataWriter_ptr _nil () { return 0; }
                static const char * _local_id;
                VehicleLocalPosition_DataWriter_ptr _this () { return this; }

                virtual DDS::LongLong register_instance (const VehicleLocalPosition_& instance_data) = 0;
                virtual DDS::LongLong register_instance_w_timestamp (const VehicleLocalPosition_& instance_data, const DDS::Time_t& source_timestamp) = 0;
                virtual DDS::Long unregister_instance (const VehicleLocalPosition_& instance_data, DDS::LongLong handle) = 0;
                virtual DDS::Long unregister_instance_w_timestamp (const VehicleLocalPosition_& instance_data, DDS::LongLong handle, const DDS::Time_t& source_timestamp) = 0;
                virtual DDS::Long write (const VehicleLocalPosition_& instance_data, DDS::LongLong handle) = 0;
                virtual DDS::Long write_w_timestamp (const VehicleLocalPosition_& instance_data, DDS::LongLong handle, const DDS::Time_t& source_timestamp) = 0;
                virtual DDS::Long dispose (const VehicleLocalPosition_& instance_data, DDS::LongLong handle) = 0;
                virtual DDS::Long dispose_w_timestamp (const VehicleLocalPosition_& instance_data, DDS::LongLong handle, const DDS::Time_t& source_timestamp) = 0;
                virtual DDS::Long writedispose (const VehicleLocalPosition_& instance_data, DDS::LongLong handle) = 0;
                virtual DDS::Long writedispose_w_timestamp (const VehicleLocalPosition_& instance_data, DDS::LongLong handle, const DDS::Time_t& source_timestamp) = 0;
                virtual DDS::Long get_key_value (VehicleLocalPosition_& key_holder, DDS::LongLong handle) = 0;
                virtual DDS::LongLong lookup_instance (const VehicleLocalPosition_& instance_data) = 0;

            protected:
                VehicleLocalPosition_DataWriter () {};
                ~VehicleLocalPosition_DataWriter () {};
            private:
                VehicleLocalPosition_DataWriter (const VehicleLocalPosition_DataWriter &);
                VehicleLocalPosition_DataWriter & operator = (const VehicleLocalPosition_DataWriter &);
            };

            class  VehicleLocalPosition_DataReader :
                virtual public DDS::DataReader
            { 
            public:
                typedef VehicleLocalPosition_DataReader_ptr _ptr_type;
                typedef VehicleLocalPosition_DataReader_var _var_type;

                static VehicleLocalPosition_DataReader_ptr _duplicate (VehicleLocalPosition_DataReader_ptr obj);
                DDS::Boolean _local_is_a (const char * id);

                static VehicleLocalPosition_DataReader_ptr _narrow (DDS::Object_ptr obj);
                static VehicleLocalPosition_DataReader_ptr _unchecked_narrow (DDS::Object_ptr obj);
                static VehicleLocalPosition_DataReader_ptr _nil () { return 0; }
                static const char * _local_id;
                VehicleLocalPosition_DataReader_ptr _this () { return this; }

                virtual DDS::Long read (VehicleLocalPosition_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
                virtual DDS::Long take (VehicleLocalPosition_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
                virtual DDS::Long read_w_condition (VehicleLocalPosition_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::ReadCondition_ptr a_condition) = 0;
                virtual DDS::Long take_w_condition (VehicleLocalPosition_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::ReadCondition_ptr a_condition) = 0;
                virtual DDS::Long read_next_sample (VehicleLocalPosition_& received_data, DDS::SampleInfo& sample_info) = 0;
                virtual DDS::Long take_next_sample (VehicleLocalPosition_& received_data, DDS::SampleInfo& sample_info) = 0;
                virtual DDS::Long read_instance (VehicleLocalPosition_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
                virtual DDS::Long take_instance (VehicleLocalPosition_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
                virtual DDS::Long read_next_instance (VehicleLocalPosition_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
                virtual DDS::Long take_next_instance (VehicleLocalPosition_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
                virtual DDS::Long read_next_instance_w_condition (VehicleLocalPosition_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ReadCondition_ptr a_condition) = 0;
                virtual DDS::Long take_next_instance_w_condition (VehicleLocalPosition_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ReadCondition_ptr a_condition) = 0;
                virtual DDS::Long return_loan (VehicleLocalPosition_Seq& received_data, DDS::SampleInfoSeq& info_seq) = 0;
                virtual DDS::Long get_key_value (VehicleLocalPosition_& key_holder, DDS::LongLong handle) = 0;
                virtual DDS::LongLong lookup_instance (const VehicleLocalPosition_& instance) = 0;

            protected:
                VehicleLocalPosition_DataReader () {};
                ~VehicleLocalPosition_DataReader () {};
            private:
                VehicleLocalPosition_DataReader (const VehicleLocalPosition_DataReader &);
                VehicleLocalPosition_DataReader & operator = (const VehicleLocalPosition_DataReader &);
            };

            class  VehicleLocalPosition_DataReaderView :
                virtual public DDS::DataReaderView
            { 
            public:
                typedef VehicleLocalPosition_DataReaderView_ptr _ptr_type;
                typedef VehicleLocalPosition_DataReaderView_var _var_type;

                static VehicleLocalPosition_DataReaderView_ptr _duplicate (VehicleLocalPosition_DataReaderView_ptr obj);
                DDS::Boolean _local_is_a (const char * id);

                static VehicleLocalPosition_DataReaderView_ptr _narrow (DDS::Object_ptr obj);
                static VehicleLocalPosition_DataReaderView_ptr _unchecked_narrow (DDS::Object_ptr obj);
                static VehicleLocalPosition_DataReaderView_ptr _nil () { return 0; }
                static const char * _local_id;
                VehicleLocalPosition_DataReaderView_ptr _this () { return this; }

                virtual DDS::Long read (VehicleLocalPosition_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
                virtual DDS::Long take (VehicleLocalPosition_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
                virtual DDS::Long read_w_condition (VehicleLocalPosition_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::ReadCondition_ptr a_condition) = 0;
                virtual DDS::Long take_w_condition (VehicleLocalPosition_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::ReadCondition_ptr a_condition) = 0;
                virtual DDS::Long read_next_sample (VehicleLocalPosition_& received_data, DDS::SampleInfo& sample_info) = 0;
                virtual DDS::Long take_next_sample (VehicleLocalPosition_& received_data, DDS::SampleInfo& sample_info) = 0;
                virtual DDS::Long read_instance (VehicleLocalPosition_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
                virtual DDS::Long take_instance (VehicleLocalPosition_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
                virtual DDS::Long read_next_instance (VehicleLocalPosition_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
                virtual DDS::Long take_next_instance (VehicleLocalPosition_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
                virtual DDS::Long read_next_instance_w_condition (VehicleLocalPosition_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ReadCondition_ptr a_condition) = 0;
                virtual DDS::Long take_next_instance_w_condition (VehicleLocalPosition_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ReadCondition_ptr a_condition) = 0;
                virtual DDS::Long return_loan (VehicleLocalPosition_Seq& received_data, DDS::SampleInfoSeq& info_seq) = 0;
                virtual DDS::Long get_key_value (VehicleLocalPosition_& key_holder, DDS::LongLong handle) = 0;
                virtual DDS::LongLong lookup_instance (const VehicleLocalPosition_& instance) = 0;

            protected:
                VehicleLocalPosition_DataReaderView () {};
                ~VehicleLocalPosition_DataReaderView () {};
            private:
                VehicleLocalPosition_DataReaderView (const VehicleLocalPosition_DataReaderView &);
                VehicleLocalPosition_DataReaderView & operator = (const VehicleLocalPosition_DataReaderView &);
            };
        }

    }

}

#endif
