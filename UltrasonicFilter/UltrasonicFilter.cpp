/**
 * Copyright (c)
Audi Autonomous Driving Cup. TEAM FAUtonOHM.  All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3. All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4. Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

ADTF Filter for Ultrasonic Sensors moving average filtering
**********************************************************************
* $Author::  $ fink, hiller  $Date:: 2015-12-06 00:00:00#$ $Rev:: 1.0   $
**********************************************************************/


#include "stdafx.h"
#include "UltrasonicFilter.h"

/// Create filter shell
ADTF_FILTER_PLUGIN("Ultrasonic Mean Filter", OID_ADTF_ULTRASONIC_FILTER, UltrasonicFilter);


UltrasonicFilter::UltrasonicFilter(const tChar* __info):cFilter(__info)
{

    // initial value for number of measurements used for moving average filter
	m_i32_FilterCount = 3;

	//filter enabled (default true)
	m_boolFilterEnable = tTrue;

	// initialization of memory sizes of media samples to create
	nSize_USStruct = 0;

    /** create the filter properties that have to be set in the Property Editor **/
	/** Properties for number of measurements **/
    SetPropertyFloat("Moving Average Count",m_i32_FilterCount);
    SetPropertyFloat("Moving Average Count" NSSUBPROP_REQUIRED, tTrue);
    SetPropertyStr("Moving Average Count" NSSUBPROP_DESCRIPTION, "The number of measurements used for moving average filter");

    /** enable filter property **/
    SetPropertyBool("Enable Filter",m_boolFilterEnable);
    SetPropertyBool("Enable Filter" NSSUBPROP_REQUIRED, tTrue);
    SetPropertyStr("Enable Filter" NSSUBPROP_DESCRIPTION, "Enables filter");

}

UltrasonicFilter::~UltrasonicFilter()
{

}

tResult UltrasonicFilter::CreateInputPins(__exception)
{
	//get the media description manager for this filter
	cObjectPtr<IMediaDescriptionManager> pDescManager;
	RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    //get description for tUltrasonicStruct input pin
    tChar const * strDescUltrasonicSctruct = pDescManager->GetMediaDescription("tUltrasonicStruct");
    // checks if exists
    RETURN_IF_POINTER_NULL(strDescUltrasonicSctruct);

    //get mediatype for tUltrasonicStruct
    cObjectPtr<IMediaType> pTypeUltrasonicStruct = new cMediaType(0, 0, 0, "tUltrasonicStruct", strDescUltrasonicSctruct,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_POINTER_NULL(pTypeUltrasonicStruct);
    
    // Create input pin for ultrasonic sensor
    RETURN_IF_FAILED(ultrasonicStruct_input.Create("Ultrasonic_Struct", pTypeUltrasonicStruct, static_cast<IPinEventSink*> (this)));
    RETURN_IF_FAILED(RegisterPin(&ultrasonicStruct_input));

    RETURN_NOERROR;
}

tResult UltrasonicFilter::CreateOutputPins(__exception)
{
    //get the media description manager for this filter
    cObjectPtr<IMediaDescriptionManager> pDescManager;
    RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

    /** Creation of the output pin speed filtered US values (struct) **/

    //get description for tUltrasonicStruct output pin
    tChar const * strDescUltrasonicSctruct = pDescManager->GetMediaDescription("tUltrasonicStruct");
    // checks if exists
    RETURN_IF_POINTER_NULL(strDescUltrasonicSctruct);

    //get mediatype
    cObjectPtr<IMediaType> pTypeUltrasonicStruct = new cMediaType(0, 0, 0, "tUltrasonicStruct", strDescUltrasonicSctruct,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
    RETURN_IF_POINTER_NULL(pTypeUltrasonicStruct);
    //get mediatype description for tUltrasonicStruct data type
    RETURN_IF_FAILED(pTypeUltrasonicStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescription_Ultrasonic_Struct));

    // create output pin for filtered US struct
    RETURN_IF_FAILED(ultrasonicStruct_output.Create("Ultrasonic_Struct_Filtered", pTypeUltrasonicStruct, NULL));
    RETURN_IF_FAILED(RegisterPin(&ultrasonicStruct_output));

    RETURN_NOERROR;
}


tResult UltrasonicFilter::Init(tInitStage eStage, __exception)
{
    // never miss calling the parent implementation!!
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))
    
    // in StageFirst you can create and register your static pins.
    if (eStage == StageFirst)
    {
        // create and register the input and output pin
        RETURN_IF_FAILED(CreateInputPins(__exception_ptr));
        RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));

    }
    else if (eStage == StageNormal)
    {
    	// get Properties from user interface
    	m_i32_FilterCount = tFloat32(GetPropertyFloat("Moving Average Count"));
    	m_boolFilterEnable = tBool(GetPropertyBool("Enable Filter"));

    }
    else if (eStage == StageGraphReady)
    {
        // All pin connections have been established in this stage so you can query your pins
        // about their media types and additional meta data.
        // Please take a look at the demo_imageproc example for further reference.
    	tboolIDs_UltrasonicStructSet_In = tFalse;

		//allocate memory with the size given by the descriptor
		cObjectPtr<IMediaSerializer> pSerializer_USStruct;
		m_pDescription_Ultrasonic_Struct->GetMediaSampleSerializer(&pSerializer_USStruct);
		nSize_USStruct = pSerializer_USStruct->GetDeserializedSize();
    }

    RETURN_NOERROR;
}


tResult UltrasonicFilter::Start(__exception)
{
    return cFilter::Start(__exception_ptr);
}

tResult UltrasonicFilter::Stop(__exception)
{
    return cFilter::Stop(__exception_ptr);
}


tResult UltrasonicFilter::Shutdown(tInitStage eStage, __exception)
{
    // In each stage clean up everything that you initiaized in the corresponding stage during Init.
    // Pins are an exception: 
    // - The base class takes care of static pins that are members of this class.
    // - Dynamic pins have to be cleaned up in the ReleasePins method, please see the demo_dynamicpin
    //   example for further reference.
    

    // call the base class implementation
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult UltrasonicFilter::ProcessUltrasonicInput(IMediaSample* pMediaSample, tempUltrasonicStruct* temp_US_struct)
{

    {   // focus for sample read lock
        // read-out the incoming Media Sample
        __adtf_sample_read_lock_mediadescription(m_pDescription_Ultrasonic_Struct,pMediaSample,pCoderInput);

        // get the IDs for the items in the media sample (only once for input and output (same type))
        if(!tboolIDs_UltrasonicStructSet_In)
        {
            pCoderInput->GetID("tFrontLeft", tbufID_tFrontLeft);
            pCoderInput->GetID("tFrontCenterLeft", tbufID_tFrontCenterLeft);
            pCoderInput->GetID("tFrontCenter", tbufID_tFrontCenter);
            pCoderInput->GetID("tFrontCenterRight", tbufID_tFrontCenterRight);
            pCoderInput->GetID("tFrontRight", tbufID_tFrontRight);
            pCoderInput->GetID("tSideLeft", tbufID_tSideLeft);
            pCoderInput->GetID("tSideRight", tbufID_tSideRight);
            pCoderInput->GetID("tRearLeft", tbufID_tRearLeft);
            pCoderInput->GetID("tRearCenter", tbufID_tRearCenter);
            pCoderInput->GetID("tRearRight", tbufID_tRearRight);
            tboolIDs_UltrasonicStructSet_In = tTrue;
            // LOG_WARNING(cString::Format("IDs Input set"));

        }

        // get values from media sample
        pCoderInput->Get(tbufID_tFrontLeft, (tVoid*)&temp_US_struct->FrontLeft);
        pCoderInput->Get(tbufID_tFrontCenterLeft, (tVoid*)&temp_US_struct->FrontCenterLeft);
        pCoderInput->Get(tbufID_tFrontCenter, (tVoid*)&temp_US_struct->FrontCenter);
        pCoderInput->Get(tbufID_tFrontCenterRight, (tVoid*)&temp_US_struct->FrontCenterRight);
        pCoderInput->Get(tbufID_tFrontRight, (tVoid*)&temp_US_struct->FrontRight);
        pCoderInput->Get(tbufID_tSideLeft, (tVoid*)&temp_US_struct->SideLeft);
        pCoderInput->Get(tbufID_tSideRight, (tVoid*)&temp_US_struct->SideRight);
        pCoderInput->Get(tbufID_tRearLeft, (tVoid*)&temp_US_struct->RearLeft);
        pCoderInput->Get(tbufID_tRearCenter, (tVoid*)&temp_US_struct->RearCenter);
        pCoderInput->Get(tbufID_tRearRight, (tVoid*)&temp_US_struct->RearRight);


        //LOG_WARNING(cString::Format("data value of FrontCenterUS: %f",temp_US_struct.FrontCenter.data_value));
    }

    RETURN_NOERROR;
}

UltrasonicFilter::tempUltrasonicStruct UltrasonicFilter::CalcUSMeanValue(tempUltrasonicStruct received_US_data){

	// Push last samples in US lists
	US_Mean_FrontLeft.push_back(received_US_data.FrontLeft.data_value);
	US_Mean_FrontCenterLeft.push_back(received_US_data.FrontCenterLeft.data_value);
	US_Mean_FrontCenter.push_back(received_US_data.FrontCenter.data_value);
	US_Mean_FrontCenterRight.push_back(received_US_data.FrontCenterRight.data_value);
	US_Mean_FrontRight.push_back(received_US_data.FrontRight.data_value);
	US_Mean_SideLeft.push_back(received_US_data.SideLeft.data_value);
	US_Mean_SideRight.push_back(received_US_data.SideRight.data_value);
	US_Mean_RearLeft.push_back(received_US_data.RearLeft.data_value);
	US_Mean_RearCenter.push_back(received_US_data.RearCenter.data_value);
	US_Mean_RearRight.push_back(received_US_data.RearRight.data_value);

	// remove first element in lists when count is reached
	if ((int)US_Mean_FrontLeft.size() > m_i32_FilterCount)
	{
		 US_Mean_FrontLeft.pop_front();
		 US_Mean_FrontCenterLeft.pop_front();
		 US_Mean_FrontCenter.pop_front();
		 US_Mean_FrontCenterRight.pop_front();
		 US_Mean_FrontRight.pop_front();
		 US_Mean_SideLeft.pop_front();
		 US_Mean_SideRight.pop_front();
		 US_Mean_RearLeft.pop_front();
		 US_Mean_RearCenter.pop_front();
		 US_Mean_RearRight.pop_front();
	}

	// calc mean values for all sensors and replace old values in received data

	// Front Left average
	float sum = 0;
	for (std::list<float>::iterator p = US_Mean_FrontLeft.begin(); p != US_Mean_FrontLeft.end(); ++p)
	{
		sum += (float)*p;
	}
	received_US_data.FrontLeft.data_value = sum / US_Mean_FrontLeft.size();

	// Front Center Left average
	sum = 0;
	for (std::list<float>::iterator p = US_Mean_FrontCenterLeft.begin(); p != US_Mean_FrontCenterLeft.end(); ++p)
	{
		sum += (float)*p;
	}
	received_US_data.FrontCenterLeft.data_value = sum / US_Mean_FrontCenterLeft.size();

	// Front Center average
	sum = 0;
	for (std::list<float>::iterator p = US_Mean_FrontCenter.begin(); p != US_Mean_FrontCenter.end(); ++p)
	{
		sum += (float)*p;
	}
	received_US_data.FrontCenter.data_value = sum / US_Mean_FrontCenter.size();

	// Front Center Right average
	sum = 0;
	for (std::list<float>::iterator p = US_Mean_FrontCenterRight.begin(); p != US_Mean_FrontCenterRight.end(); ++p)
	{
		sum += (float)*p;
	}
	received_US_data.FrontCenterRight.data_value = sum / US_Mean_FrontCenterRight.size();

	// Front Right average
	sum = 0;
	for (std::list<float>::iterator p = US_Mean_FrontRight.begin(); p != US_Mean_FrontRight.end(); ++p)
	{
		sum += (float)*p;
	}
	received_US_data.FrontRight.data_value = sum / US_Mean_FrontRight.size();

	// Side Left average
	sum = 0;
	for (std::list<float>::iterator p = US_Mean_SideLeft.begin(); p != US_Mean_SideLeft.end(); ++p)
	{
		sum += (float)*p;
	}
	received_US_data.SideLeft.data_value = sum / US_Mean_SideLeft.size();

	// Side Right average
	sum = 0;
	for (std::list<float>::iterator p = US_Mean_SideRight.begin(); p != US_Mean_SideRight.end(); ++p)
	{
		sum += (float)*p;
	}
	received_US_data.SideRight.data_value = sum / US_Mean_SideRight.size();

	// Rear Left average
	sum = 0;
	for (std::list<float>::iterator p = US_Mean_RearLeft.begin(); p != US_Mean_RearLeft.end(); ++p)
	{
		sum += (float)*p;
	}
	received_US_data.RearLeft.data_value = sum / US_Mean_RearLeft.size();

	// Rear Center average
	sum = 0;
	for (std::list<float>::iterator p = US_Mean_RearCenter.begin(); p != US_Mean_RearCenter.end(); ++p)
	{
		sum += (float)*p;
	}
	received_US_data.RearCenter.data_value = sum / US_Mean_RearCenter.size();

	// Rear Right average
	sum = 0;
	for (std::list<float>::iterator p = US_Mean_RearRight.begin(); p != US_Mean_RearRight.end(); ++p)
	{
		sum += (float)*p;
	}
	received_US_data.RearRight.data_value = sum / US_Mean_RearRight.size();

	// US output struct with enw values
	return received_US_data;

}

tResult UltrasonicFilter::TransmitUSStruct(const tempUltrasonicStruct output_US_Struct, tTimeStamp tsInputTime) {

	// create new media sample
	cObjectPtr<IMediaSample> pNewMediaSample;
	if (IS_OK(AllocMediaSample((tVoid**)&pNewMediaSample)))
	{
	  // New US Struct media sample
		  pNewMediaSample->AllocBuffer(nSize_USStruct);
		  {   // focus for sample write lock
			  // write date to the media sample with the coder of the descriptor
			  __adtf_sample_write_lock_mediadescription(m_pDescription_Ultrasonic_Struct,pNewMediaSample,pCoderOutput);


			  // get the IDs for the items in the media sample
			  // same as input above

			  // set values in new media sample
			   pCoderOutput->Set(tbufID_tFrontLeft, (tVoid*)&output_US_Struct.FrontLeft);
			   pCoderOutput->Set(tbufID_tFrontCenterLeft, (tVoid*)&output_US_Struct.FrontCenterLeft);
			   pCoderOutput->Set(tbufID_tFrontCenter, (tVoid*)&output_US_Struct.FrontCenter);
			   pCoderOutput->Set(tbufID_tFrontCenterRight, (tVoid*)&output_US_Struct.FrontCenterRight);
			   pCoderOutput->Set(tbufID_tFrontRight, (tVoid*)&output_US_Struct.FrontRight);
			   pCoderOutput->Set(tbufID_tSideLeft, (tVoid*)&output_US_Struct.SideLeft);
			   pCoderOutput->Set(tbufID_tSideRight, (tVoid*)&output_US_Struct.SideRight);
			   pCoderOutput->Set(tbufID_tRearLeft, (tVoid*)&output_US_Struct.RearLeft);
			   pCoderOutput->Set(tbufID_tRearCenter, (tVoid*)&output_US_Struct.RearCenter);
			   pCoderOutput->Set(tbufID_tRearRight, (tVoid*)&output_US_Struct.RearRight);
		  }

		  // transmit media sample over output pin
		  pNewMediaSample->SetTime(tsInputTime);
		  ultrasonicStruct_output.Transmit(pNewMediaSample);
	}

	RETURN_NOERROR;
}

tResult UltrasonicFilter::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{

	RETURN_IF_POINTER_NULL(pMediaSample);
	RETURN_IF_POINTER_NULL(pSource);

     // first check what kind of event it is (and if output exists?)
    if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
    {

        // by comparing it to our member pin variable we can find out which pin received
        // the sample
        if (pSource == &ultrasonicStruct_input)
        {

			__synchronized_obj(m_oProcessUltrasonicInput);

			// receive US Struct
			tempUltrasonicStruct received_US_struct;
			ProcessUltrasonicInput(pMediaSample, &received_US_struct);

			// new output US struct
			tempUltrasonicStruct output_Ultrasonic_Struct;

			// if filter enabled
			if(m_boolFilterEnable)
			{
				output_Ultrasonic_Struct = CalcUSMeanValue(received_US_struct);
			} else { // else send input as output
				output_Ultrasonic_Struct = received_US_struct;
			}

			// transmit US Struct
			TransmitUSStruct(output_Ultrasonic_Struct, pMediaSample->GetTime());

        }
    }

    RETURN_NOERROR;
}
