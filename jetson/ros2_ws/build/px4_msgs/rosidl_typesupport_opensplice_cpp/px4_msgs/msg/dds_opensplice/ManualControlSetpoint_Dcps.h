#ifndef _H_8A825DA96616D2E7EFE699F458435BD8_ManualControlSetpoint_DCPS_H_
#define _H_8A825DA96616D2E7EFE699F458435BD8_ManualControlSetpoint_DCPS_H_

#include "sacpp_mapping.h"
#include "dds_dcps.h"
#include "ManualControlSetpoint_.h"


namespace px4_msgs
{
    namespace msg
    {
        namespace dds_
        {
            class ManualControlSetpoint_TypeSupportInterface;

            typedef ManualControlSetpoint_TypeSupportInterface * ManualControlSetpoint_TypeSupportInterface_ptr;
            typedef DDS_DCPSInterface_var < ManualControlSetpoint_TypeSupportInterface> ManualControlSetpoint_TypeSupportInterface_var;
            typedef DDS_DCPSInterface_out < ManualControlSetpoint_TypeSupportInterface> ManualControlSetpoint_TypeSupportInterface_out;


            class ManualControlSetpoint_DataWriter;

            typedef ManualControlSetpoint_DataWriter * ManualControlSetpoint_DataWriter_ptr;
            typedef DDS_DCPSInterface_var < ManualControlSetpoint_DataWriter> ManualControlSetpoint_DataWriter_var;
            typedef DDS_DCPSInterface_out < ManualControlSetpoint_DataWriter> ManualControlSetpoint_DataWriter_out;


            class ManualControlSetpoint_DataReader;

            typedef ManualControlSetpoint_DataReader * ManualControlSetpoint_DataReader_ptr;
            typedef DDS_DCPSInterface_var < ManualControlSetpoint_DataReader> ManualControlSetpoint_DataReader_var;
            typedef DDS_DCPSInterface_out < ManualControlSetpoint_DataReader> ManualControlSetpoint_DataReader_out;


            class ManualControlSetpoint_DataReaderView;

            typedef ManualControlSetpoint_DataReaderView * ManualControlSetpoint_DataReaderView_ptr;
            typedef DDS_DCPSInterface_var < ManualControlSetpoint_DataReaderView> ManualControlSetpoint_DataReaderView_var;
            typedef DDS_DCPSInterface_out < ManualControlSetpoint_DataReaderView> ManualControlSetpoint_DataReaderView_out;

            struct ManualControlSetpoint_Seq_uniq_ {};
            typedef DDS_DCPSUFLSeq < ManualControlSetpoint_, struct ManualControlSetpoint_Seq_uniq_> ManualControlSetpoint_Seq;
            typedef DDS_DCPSSequence_var < ManualControlSetpoint_Seq> ManualControlSetpoint_Seq_var;
            typedef DDS_DCPSSequence_out < ManualControlSetpoint_Seq> ManualControlSetpoint_Seq_out;

            class  ManualControlSetpoint_TypeSupportInterface :
                virtual public DDS::TypeSupport
            { 
            public:
                typedef ManualControlSetpoint_TypeSupportInterface_ptr _ptr_type;
                typedef ManualControlSetpoint_TypeSupportInterface_var _var_type;

                static ManualControlSetpoint_TypeSupportInterface_ptr _duplicate (ManualControlSetpoint_TypeSupportInterface_ptr obj);
                DDS::Boolean _local_is_a (const char * id);

                static ManualControlSetpoint_TypeSupportInterface_ptr _narrow (DDS::Object_ptr obj);
                static ManualControlSetpoint_TypeSupportInterface_ptr _unchecked_narrow (DDS::Object_ptr obj);
                static ManualControlSetpoint_TypeSupportInterface_ptr _nil () { return 0; }
                static const char * _local_id;
                ManualControlSetpoint_TypeSupportInterface_ptr _this () { return this; }


            protected:
                ManualControlSetpoint_TypeSupportInterface () {};
                ~ManualControlSetpoint_TypeSupportInterface () {};
            private:
                ManualControlSetpoint_TypeSupportInterface (const ManualControlSetpoint_TypeSupportInterface &);
                ManualControlSetpoint_TypeSupportInterface & operator = (const ManualControlSetpoint_TypeSupportInterface &);
            };

            class  ManualControlSetpoint_DataWriter :
                virtual public DDS::DataWriter
            { 
            public:
                typedef ManualControlSetpoint_DataWriter_ptr _ptr_type;
                typedef ManualControlSetpoint_DataWriter_var _var_type;

                static ManualControlSetpoint_DataWriter_ptr _duplicate (ManualControlSetpoint_DataWriter_ptr obj);
                DDS::Boolean _local_is_a (const char * id);

                static ManualControlSetpoint_DataWriter_ptr _narrow (DDS::Object_ptr obj);
                static ManualControlSetpoint_DataWriter_ptr _unchecked_narrow (DDS::Object_ptr obj);
                static ManualControlSetpoint_DataWriter_ptr _nil () { return 0; }
                static const char * _local_id;
                ManualControlSetpoint_DataWriter_ptr _this () { return this; }

                virtual DDS::LongLong register_instance (const ManualControlSetpoint_& instance_data) = 0;
                virtual DDS::LongLong register_instance_w_timestamp (const ManualControlSetpoint_& instance_data, const DDS::Time_t& source_timestamp) = 0;
                virtual DDS::Long unregister_instance (const ManualControlSetpoint_& instance_data, DDS::LongLong handle) = 0;
                virtual DDS::Long unregister_instance_w_timestamp (const ManualControlSetpoint_& instance_data, DDS::LongLong handle, const DDS::Time_t& source_timestamp) = 0;
                virtual DDS::Long write (const ManualControlSetpoint_& instance_data, DDS::LongLong handle) = 0;
                virtual DDS::Long write_w_timestamp (const ManualControlSetpoint_& instance_data, DDS::LongLong handle, const DDS::Time_t& source_timestamp) = 0;
                virtual DDS::Long dispose (const ManualControlSetpoint_& instance_data, DDS::LongLong handle) = 0;
                virtual DDS::Long dispose_w_timestamp (const ManualControlSetpoint_& instance_data, DDS::LongLong handle, const DDS::Time_t& source_timestamp) = 0;
                virtual DDS::Long writedispose (const ManualControlSetpoint_& instance_data, DDS::LongLong handle) = 0;
                virtual DDS::Long writedispose_w_timestamp (const ManualControlSetpoint_& instance_data, DDS::LongLong handle, const DDS::Time_t& source_timestamp) = 0;
                virtual DDS::Long get_key_value (ManualControlSetpoint_& key_holder, DDS::LongLong handle) = 0;
                virtual DDS::LongLong lookup_instance (const ManualControlSetpoint_& instance_data) = 0;

            protected:
                ManualControlSetpoint_DataWriter () {};
                ~ManualControlSetpoint_DataWriter () {};
            private:
                ManualControlSetpoint_DataWriter (const ManualControlSetpoint_DataWriter &);
                ManualControlSetpoint_DataWriter & operator = (const ManualControlSetpoint_DataWriter &);
            };

            class  ManualControlSetpoint_DataReader :
                virtual public DDS::DataReader
            { 
            public:
                typedef ManualControlSetpoint_DataReader_ptr _ptr_type;
                typedef ManualControlSetpoint_DataReader_var _var_type;

                static ManualControlSetpoint_DataReader_ptr _duplicate (ManualControlSetpoint_DataReader_ptr obj);
                DDS::Boolean _local_is_a (const char * id);

                static ManualControlSetpoint_DataReader_ptr _narrow (DDS::Object_ptr obj);
                static ManualControlSetpoint_DataReader_ptr _unchecked_narrow (DDS::Object_ptr obj);
                static ManualControlSetpoint_DataReader_ptr _nil () { return 0; }
                static const char * _local_id;
                ManualControlSetpoint_DataReader_ptr _this () { return this; }

                virtual DDS::Long read (ManualControlSetpoint_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
                virtual DDS::Long take (ManualControlSetpoint_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
                virtual DDS::Long read_w_condition (ManualControlSetpoint_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::ReadCondition_ptr a_condition) = 0;
                virtual DDS::Long take_w_condition (ManualControlSetpoint_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::ReadCondition_ptr a_condition) = 0;
                virtual DDS::Long read_next_sample (ManualControlSetpoint_& received_data, DDS::SampleInfo& sample_info) = 0;
                virtual DDS::Long take_next_sample (ManualControlSetpoint_& received_data, DDS::SampleInfo& sample_info) = 0;
                virtual DDS::Long read_instance (ManualControlSetpoint_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
                virtual DDS::Long take_instance (ManualControlSetpoint_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
                virtual DDS::Long read_next_instance (ManualControlSetpoint_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
                virtual DDS::Long take_next_instance (ManualControlSetpoint_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
                virtual DDS::Long read_next_instance_w_condition (ManualControlSetpoint_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ReadCondition_ptr a_condition) = 0;
                virtual DDS::Long take_next_instance_w_condition (ManualControlSetpoint_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ReadCondition_ptr a_condition) = 0;
                virtual DDS::Long return_loan (ManualControlSetpoint_Seq& received_data, DDS::SampleInfoSeq& info_seq) = 0;
                virtual DDS::Long get_key_value (ManualControlSetpoint_& key_holder, DDS::LongLong handle) = 0;
                virtual DDS::LongLong lookup_instance (const ManualControlSetpoint_& instance) = 0;

            protected:
                ManualControlSetpoint_DataReader () {};
                ~ManualControlSetpoint_DataReader () {};
            private:
                ManualControlSetpoint_DataReader (const ManualControlSetpoint_DataReader &);
                ManualControlSetpoint_DataReader & operator = (const ManualControlSetpoint_DataReader &);
            };

            class  ManualControlSetpoint_DataReaderView :
                virtual public DDS::DataReaderView
            { 
            public:
                typedef ManualControlSetpoint_DataReaderView_ptr _ptr_type;
                typedef ManualControlSetpoint_DataReaderView_var _var_type;

                static ManualControlSetpoint_DataReaderView_ptr _duplicate (ManualControlSetpoint_DataReaderView_ptr obj);
                DDS::Boolean _local_is_a (const char * id);

                static ManualControlSetpoint_DataReaderView_ptr _narrow (DDS::Object_ptr obj);
                static ManualControlSetpoint_DataReaderView_ptr _unchecked_narrow (DDS::Object_ptr obj);
                static ManualControlSetpoint_DataReaderView_ptr _nil () { return 0; }
                static const char * _local_id;
                ManualControlSetpoint_DataReaderView_ptr _this () { return this; }

                virtual DDS::Long read (ManualControlSetpoint_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
                virtual DDS::Long take (ManualControlSetpoint_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
                virtual DDS::Long read_w_condition (ManualControlSetpoint_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::ReadCondition_ptr a_condition) = 0;
                virtual DDS::Long take_w_condition (ManualControlSetpoint_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::ReadCondition_ptr a_condition) = 0;
                virtual DDS::Long read_next_sample (ManualControlSetpoint_& received_data, DDS::SampleInfo& sample_info) = 0;
                virtual DDS::Long take_next_sample (ManualControlSetpoint_& received_data, DDS::SampleInfo& sample_info) = 0;
                virtual DDS::Long read_instance (ManualControlSetpoint_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
                virtual DDS::Long take_instance (ManualControlSetpoint_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
                virtual DDS::Long read_next_instance (ManualControlSetpoint_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
                virtual DDS::Long take_next_instance (ManualControlSetpoint_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ULong sample_states, DDS::ULong view_states, DDS::ULong instance_states) = 0;
                virtual DDS::Long read_next_instance_w_condition (ManualControlSetpoint_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ReadCondition_ptr a_condition) = 0;
                virtual DDS::Long take_next_instance_w_condition (ManualControlSetpoint_Seq& received_data, DDS::SampleInfoSeq& info_seq, DDS::Long max_samples, DDS::LongLong a_handle, DDS::ReadCondition_ptr a_condition) = 0;
                virtual DDS::Long return_loan (ManualControlSetpoint_Seq& received_data, DDS::SampleInfoSeq& info_seq) = 0;
                virtual DDS::Long get_key_value (ManualControlSetpoint_& key_holder, DDS::LongLong handle) = 0;
                virtual DDS::LongLong lookup_instance (const ManualControlSetpoint_& instance) = 0;

            protected:
                ManualControlSetpoint_DataReaderView () {};
                ~ManualControlSetpoint_DataReaderView () {};
            private:
                ManualControlSetpoint_DataReaderView (const ManualControlSetpoint_DataReaderView &);
                ManualControlSetpoint_DataReaderView & operator = (const ManualControlSetpoint_DataReaderView &);
            };
        }

    }

}

#endif
