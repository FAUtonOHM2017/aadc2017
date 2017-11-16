/**
 * Copyright (c)
Audi Autonomous Driving Cup. TEAM FAUtonOHM.  All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3. All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4. Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

ADTF Filter for splitting up the defined 'Ultrasonic Struct' into its different components
**********************************************************************
* $Author::  $ hiller  $Date:: 2015-12-13 00:00:00#$ $Rev:: 1.0   $
**********************************************************************/

#include "stdafx.h"
#include "UltrasonicStructSplit.h"

/**   <struct alignment="1" name="tUltrasonicStruct" version="1">
 <element alignment="1" arraysize="1" byteorder="LE" bytepos="0" name="tFrontLeft" type="tSignalValue" />
 <element alignment="1" arraysize="1" byteorder="LE" bytepos="8" name="tFrontCenterLeft" type="tSignalValue" />
 <element alignment="1" arraysize="1" byteorder="LE" bytepos="16" name="tFrontCenter" type="tSignalValue" />
 <element alignment="1" arraysize="1" byteorder="LE" bytepos="24" name="tFrontCenterRight" type="tSignalValue" />
 <element alignment="1" arraysize="1" byteorder="LE" bytepos="32" name="tFrontRight" type="tSignalValue" />
 <element alignment="1" arraysize="1" byteorder="LE" bytepos="40" name="tSideLeft" type="tSignalValue" />
 <element alignment="1" arraysize="1" byteorder="LE" bytepos="48" name="tSideRight" type="tSignalValue" />
 <element alignment="1" arraysize="1" byteorder="LE" bytepos="56" name="tRearLeft" type="tSignalValue" />
 <element alignment="1" arraysize="1" byteorder="LE" bytepos="64" name="tRearCenter" type="tSignalValue" />
 <element alignment="1" arraysize="1" byteorder="LE" bytepos="72" name="tRearRight" type="tSignalValue" />
 </struct> **/

/// Create filter shell
ADTF_FILTER_PLUGIN("Ultrasonic Struct Split", OID_ADTF_US_STRUCT_SPLIT, UltrasonicStructSplit);

UltrasonicStructSplit::UltrasonicStructSplit(const tChar* __info) :
		cFilter(__info) {

}

UltrasonicStructSplit::~UltrasonicStructSplit() {

}

tResult UltrasonicStructSplit::CreateInputPins(__exception)
{

	//get the media description manager for this filter
	cObjectPtr<IMediaDescriptionManager> pDescManager;
	RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

	/** Creation of the input pin ultrasonic struct **/

	//get description for tUltrasonicStruct input pin
	tChar const * strDesctUltrasonicStruct = pDescManager->GetMediaDescription("tUltrasonicStruct");
	// checks if exists
	RETURN_IF_POINTER_NULL(strDesctUltrasonicStruct);

	//get mediatype for tUltrasonicStruct
	cObjectPtr<IMediaType> pTypeUltrasonicStruct = new cMediaType(0, 0, 0, "tUltrasonicStruct", strDesctUltrasonicStruct,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
	RETURN_IF_POINTER_NULL(pTypeUltrasonicStruct);

	//get mediatype description for tUltrasonicStruct data type, necessary for locking media sample
	RETURN_IF_FAILED(pTypeUltrasonicStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionUltrasonicStruct));

	// Create input pin for ultrasonic sensor
	RETURN_IF_FAILED(ultrasonicStruct_input.Create("Ultrasonic_Struct", pTypeUltrasonicStruct, static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&ultrasonicStruct_input));

	RETURN_NOERROR;
}

tResult UltrasonicStructSplit::CreateOutputPins(__exception)
{
	//get the media description manager for this filter
	cObjectPtr<IMediaDescriptionManager> pDescManager;
	RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

	/** Creation of the output pins **/

	//get description for tSignalValue output pin
	tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
	// checks if exists
	RETURN_IF_POINTER_NULL(strDescSignalValue);

	//get mediatype
	cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
	RETURN_IF_POINTER_NULL(pTypeSignalValue);
	//get mediatype description for tSignalValue data type,  necessary for locking media sample
	RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescription_tSignalValue));

	// create output pins
	// creation is done without registering output pins to event sink, as nothing should be triggered inside filter by those pins
	RETURN_IF_FAILED(US_FrontLeft_out.Create("US_FrontLeft", pTypeSignalValue, NULL));
	RETURN_IF_FAILED(RegisterPin(&US_FrontLeft_out));

	RETURN_IF_FAILED(US_FrontCenterLeft_out.Create("US_FrontCenterLeft", pTypeSignalValue, NULL));
	RETURN_IF_FAILED(RegisterPin(&US_FrontCenterLeft_out));

	RETURN_IF_FAILED(US_FrontCenter_out.Create("US_FrontCenter", pTypeSignalValue, NULL));
	RETURN_IF_FAILED(RegisterPin(&US_FrontCenter_out));

	RETURN_IF_FAILED(US_FrontCenterRight_out.Create("US_FrontCenterRight", pTypeSignalValue, NULL));
	RETURN_IF_FAILED(RegisterPin(&US_FrontCenterRight_out));

	RETURN_IF_FAILED(US_FrontRight_out.Create("US_FrontRight", pTypeSignalValue, NULL));
	RETURN_IF_FAILED(RegisterPin(&US_FrontRight_out));

	RETURN_IF_FAILED(US_SideLeft_out.Create("US_SideLeft", pTypeSignalValue, NULL));
	RETURN_IF_FAILED(RegisterPin(&US_SideLeft_out));

	RETURN_IF_FAILED(US_SideRight_out.Create("US_SideRight", pTypeSignalValue, NULL));
	RETURN_IF_FAILED(RegisterPin(&US_SideRight_out));

	RETURN_IF_FAILED(US_RearLeft_out.Create("US_RearLeft", pTypeSignalValue, NULL));
	RETURN_IF_FAILED(RegisterPin(&US_RearLeft_out));

	RETURN_IF_FAILED(US_RearCenter_out.Create("US_RearCenter", pTypeSignalValue, NULL));
	RETURN_IF_FAILED(RegisterPin(&US_RearCenter_out));

	RETURN_IF_FAILED(US_RearRight_out.Create("US_RearRight", pTypeSignalValue, NULL));
	RETURN_IF_FAILED(RegisterPin(&US_RearRight_out));

	RETURN_NOERROR;
}

tResult UltrasonicStructSplit::Init(tInitStage eStage, __exception)
{
	// never miss calling the parent implementation!!
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

	// in StageFirst you can create and register your static pins.
	if (eStage == StageFirst)
	{

		// create and register the input pin
		RETURN_IF_FAILED(CreateInputPins(__exception_ptr));
		RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));

	}
	else if (eStage == StageNormal)
	{
		// get Properties from user interface

	}
	else if (eStage == StageGraphReady)
	{
		// All pin connections have been established in this stage so you can query your pins
		// about their media types and additional meta data.
		// Please take a look at the demo_imageproc example for further reference.

		// if it is checked here, only needs to be executed ONCE -> saves method-calls and thus time!
		//allocate memory with the size given by the descriptor
		cObjectPtr<IMediaSerializer> pSerializer;
		m_pDescription_tSignalValue->GetMediaSampleSerializer(&pSerializer);
		nSize_tVal = pSerializer->GetDeserializedSize();// nSize_tVal is declared as member variable

		tboolIDs_UltrasonicStructSet = false;
		tboolIDs_outputsSet = false;

	}

	RETURN_NOERROR;
}

tResult UltrasonicStructSplit::Start(__exception)
{
	return cFilter::Start(__exception_ptr);
}

tResult UltrasonicStructSplit::Stop(__exception)
{
	return cFilter::Stop(__exception_ptr);
}

tResult UltrasonicStructSplit::Shutdown(tInitStage eStage, __exception)
{
	// In each stage clean up everything that you initiaized in the corresponding stage during Init.
	// Pins are an exception:
	// - The base class takes care of static pins that are members of this class.
	// - Dynamic pins have to be cleaned up in the ReleasePins method, please see the demo_dynamicpin
	//   example for further reference.

	// call the base class implementation
	return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult UltrasonicStructSplit::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2,
		IMediaSample* pMediaSample) {
	// first check what kind of event it is (and if output exists?)
	if (nEventCode
			== IPinEventSink::PE_MediaSampleReceived&& pMediaSample != NULL && m_pDescriptionUltrasonicStruct != NULL) {
		// so we received a media sample, so this pointer better be valid.
		RETURN_IF_POINTER_NULL(pMediaSample);

		// by comparing it to our member pin variable we can find out which pin received
		// the sample
		if (pSource == &ultrasonicStruct_input) {

			{   // focus for sample read lock
				// read-out the incoming Media Sample
				__adtf_sample_read_lock_mediadescription(m_pDescriptionUltrasonicStruct, pMediaSample, pCoderInput);
				//tVoid* pCoderInput_tmp = (tVoid*) pCoderInput;
				//RETURN_IF_POINTER_NULL(pCoderInput_tmp);

				// get the IDs for the items in the media sample
				if (!tboolIDs_UltrasonicStructSet) {
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
					tboolIDs_UltrasonicStructSet = tTrue;
				}

				// get values from media sample
				pCoderInput->Get(tbufID_tFrontLeft, (tVoid*) &temp_US_struct.FrontLeft);
				pCoderInput->Get(tbufID_tFrontCenterLeft, (tVoid*) &temp_US_struct.FrontCenterLeft);
				pCoderInput->Get(tbufID_tFrontCenter, (tVoid*) &temp_US_struct.FrontCenter);
				pCoderInput->Get(tbufID_tFrontCenterRight, (tVoid*) &temp_US_struct.FrontCenterRight);
				pCoderInput->Get(tbufID_tFrontRight, (tVoid*) &temp_US_struct.FrontRight);
				pCoderInput->Get(tbufID_tSideLeft, (tVoid*) &temp_US_struct.SideLeft);
				pCoderInput->Get(tbufID_tSideRight, (tVoid*) &temp_US_struct.SideRight);
				pCoderInput->Get(tbufID_tRearLeft, (tVoid*) &temp_US_struct.RearLeft);
				pCoderInput->Get(tbufID_tRearCenter, (tVoid*) &temp_US_struct.RearCenter);
				pCoderInput->Get(tbufID_tRearRight, (tVoid*) &temp_US_struct.RearRight);

				//	 LOG_WARNING(cString::Format("data value of FrontCenterUS: %f",temp_US_struct.FrontCenter.d));

			}

			/** create new media samples, one for each output; all have same type and therefore size **/

			// media sample Front Left
			cObjectPtr<IMediaSample> pNewMediaSampleFrontLeft;
			if (IS_OK(AllocMediaSample((tVoid** )&pNewMediaSampleFrontLeft))) {

				pNewMediaSampleFrontLeft->AllocBuffer(nSize_tVal);	//nSize_tVal declared as a member variable
				{   // focus for sample write lock
					//write date to the media sample with the coder of the descriptor
					__adtf_sample_write_lock_mediadescription(m_pDescription_tSignalValue, pNewMediaSampleFrontLeft,
							pCoderOutput);

					// get the IDs for the items in the media sample of type tSignalValue, just necessary once!
					if (!tboolIDs_outputsSet) {
						pCoderOutput->GetID("f32Value", tbufID_tSignalValue_F32Value);
						pCoderOutput->GetID("ui32ArduinoTimestamp", tbufID_tSignalValue_ArduinoTimestamp);
						tboolIDs_outputsSet = tTrue;
					}

					// set values in new media sample
					pCoderOutput->Set(tbufID_tSignalValue_F32Value, (tVoid*) &(temp_US_struct.FrontLeft.data_value));
					pCoderOutput->Set(tbufID_tSignalValue_ArduinoTimestamp,
							(tVoid*) &temp_US_struct.FrontLeft.arduino_timestep);
				}
			}

			// media sample Front Center Left
			cObjectPtr<IMediaSample> pNewMediaSampleFrontCenterLeft;
			if (IS_OK(AllocMediaSample((tVoid** )&pNewMediaSampleFrontCenterLeft))) {

				pNewMediaSampleFrontCenterLeft->AllocBuffer(nSize_tVal);	//nSize_tVal declared as a member variable
				{   // focus for sample write lock
					//write date to the media sample with the coder of the descriptor
					__adtf_sample_write_lock_mediadescription(m_pDescription_tSignalValue,
							pNewMediaSampleFrontCenterLeft, pCoderOutput);

					// set values in new media sample
					pCoderOutput->Set(tbufID_tSignalValue_F32Value,
							(tVoid*) &(temp_US_struct.FrontCenterLeft.data_value));
					pCoderOutput->Set(tbufID_tSignalValue_ArduinoTimestamp,
							(tVoid*) &temp_US_struct.FrontCenterLeft.arduino_timestep);
				}

			}

			// media sample Front Center
			cObjectPtr<IMediaSample> pNewMediaSampleFrontCenter;
			if (IS_OK(AllocMediaSample((tVoid** )&pNewMediaSampleFrontCenter))) {

				pNewMediaSampleFrontCenter->AllocBuffer(nSize_tVal);	//nSize_tVal declared as a member variable
				{   // focus for sample write lock
					//write date to the media sample with the coder of the descriptor
					__adtf_sample_write_lock_mediadescription(m_pDescription_tSignalValue, pNewMediaSampleFrontCenter,
							pCoderOutput);

					// set values in new media sample
					pCoderOutput->Set(tbufID_tSignalValue_F32Value, (tVoid*) &(temp_US_struct.FrontCenter.data_value));
					pCoderOutput->Set(tbufID_tSignalValue_ArduinoTimestamp,
							(tVoid*) &temp_US_struct.FrontCenter.arduino_timestep);
				}

			}

			// media sample Front Center Right
			cObjectPtr<IMediaSample> pNewMediaSampleFrontCenterRight;
			if (IS_OK(AllocMediaSample((tVoid** )&pNewMediaSampleFrontCenterRight))) {

				pNewMediaSampleFrontCenterRight->AllocBuffer(nSize_tVal);	//nSize_tVal declared as a member variable
				{   // focus for sample write lock
					//write date to the media sample with the coder of the descriptor
					__adtf_sample_write_lock_mediadescription(m_pDescription_tSignalValue,
							pNewMediaSampleFrontCenterRight, pCoderOutput);

					// set values in new media sample
					pCoderOutput->Set(tbufID_tSignalValue_F32Value,
							(tVoid*) &(temp_US_struct.FrontCenterRight.data_value));
					pCoderOutput->Set(tbufID_tSignalValue_ArduinoTimestamp,
							(tVoid*) &temp_US_struct.FrontCenterRight.arduino_timestep);
				}

			}

			// media sample Front Right
			cObjectPtr<IMediaSample> pNewMediaSampleFrontRight;
			if (IS_OK(AllocMediaSample((tVoid** )&pNewMediaSampleFrontRight))) {

				pNewMediaSampleFrontRight->AllocBuffer(nSize_tVal);	//nSize_tVal declared as a member variable
				{   // focus for sample write lock
					//write date to the media sample with the coder of the descriptor
					__adtf_sample_write_lock_mediadescription(m_pDescription_tSignalValue, pNewMediaSampleFrontRight,
							pCoderOutput);

					// set values in new media sample
					pCoderOutput->Set(tbufID_tSignalValue_F32Value, (tVoid*) &(temp_US_struct.FrontRight.data_value));
					pCoderOutput->Set(tbufID_tSignalValue_ArduinoTimestamp,
							(tVoid*) &temp_US_struct.FrontRight.arduino_timestep);
				}

			}

			// media sample Side Left
			cObjectPtr<IMediaSample> pNewMediaSampleSideLeft;
			if (IS_OK(AllocMediaSample((tVoid** )&pNewMediaSampleSideLeft))) {

				pNewMediaSampleSideLeft->AllocBuffer(nSize_tVal);	//nSize_tVal declared as a member variable
				{   // focus for sample write lock
					//write date to the media sample with the coder of the descriptor
					__adtf_sample_write_lock_mediadescription(m_pDescription_tSignalValue, pNewMediaSampleSideLeft,
							pCoderOutput);

					// set values in new media sample
					pCoderOutput->Set(tbufID_tSignalValue_F32Value, (tVoid*) &(temp_US_struct.SideLeft.data_value));
					pCoderOutput->Set(tbufID_tSignalValue_ArduinoTimestamp,
							(tVoid*) &temp_US_struct.SideLeft.arduino_timestep);
				}

			}

			// media sample Side Right
			cObjectPtr<IMediaSample> pNewMediaSampleSideRight;
			if (IS_OK(AllocMediaSample((tVoid** )&pNewMediaSampleSideRight))) {

				pNewMediaSampleSideRight->AllocBuffer(nSize_tVal);	//nSize_tVal declared as a member variable
				{   // focus for sample write lock
					//write date to the media sample with the coder of the descriptor
					__adtf_sample_write_lock_mediadescription(m_pDescription_tSignalValue, pNewMediaSampleSideRight,
							pCoderOutput);

					// set values in new media sample
					pCoderOutput->Set(tbufID_tSignalValue_F32Value, (tVoid*) &(temp_US_struct.SideRight.data_value));
					pCoderOutput->Set(tbufID_tSignalValue_ArduinoTimestamp,
							(tVoid*) &temp_US_struct.SideRight.arduino_timestep);
				}

			}

			// media sample Rear Left
			cObjectPtr<IMediaSample> pNewMediaSampleRearLeft;
			if (IS_OK(AllocMediaSample((tVoid** )&pNewMediaSampleRearLeft))) {

				pNewMediaSampleRearLeft->AllocBuffer(nSize_tVal);	//nSize_tVal declared as a member variable
				{   // focus for sample write lock
					//write date to the media sample with the coder of the descriptor
					__adtf_sample_write_lock_mediadescription(m_pDescription_tSignalValue, pNewMediaSampleRearLeft,
							pCoderOutput);

					// set values in new media sample
					pCoderOutput->Set(tbufID_tSignalValue_F32Value, (tVoid*) &(temp_US_struct.RearLeft.data_value));
					pCoderOutput->Set(tbufID_tSignalValue_ArduinoTimestamp,
							(tVoid*) &temp_US_struct.RearLeft.arduino_timestep);
				}

			}

			// media sample Front Right
			cObjectPtr<IMediaSample> pNewMediaSampleRearCenter;
			if (IS_OK(AllocMediaSample((tVoid** )&pNewMediaSampleRearCenter))) {

				pNewMediaSampleRearCenter->AllocBuffer(nSize_tVal);	//nSize_tVal declared as a member variable
				{   // focus for sample write lock
					//write date to the media sample with the coder of the descriptor
					__adtf_sample_write_lock_mediadescription(m_pDescription_tSignalValue, pNewMediaSampleRearCenter,
							pCoderOutput);

					// set values in new media sample
					pCoderOutput->Set(tbufID_tSignalValue_F32Value, (tVoid*) &(temp_US_struct.RearCenter.data_value));
					pCoderOutput->Set(tbufID_tSignalValue_ArduinoTimestamp,
							(tVoid*) &temp_US_struct.RearCenter.arduino_timestep);
				}

			}

			// media sample Front Right
			cObjectPtr<IMediaSample> pNewMediaSampleRearRight;
			if (IS_OK(AllocMediaSample((tVoid** )&pNewMediaSampleRearRight))) {

				pNewMediaSampleRearRight->AllocBuffer(nSize_tVal);	//nSize_tVal declared as a member variable
				{   // focus for sample write lock
					//write date to the media sample with the coder of the descriptor
					__adtf_sample_write_lock_mediadescription(m_pDescription_tSignalValue, pNewMediaSampleRearRight,
							pCoderOutput);

					// set values in new media sample
					pCoderOutput->Set(tbufID_tSignalValue_F32Value, (tVoid*) &(temp_US_struct.RearRight.data_value));
					pCoderOutput->Set(tbufID_tSignalValue_ArduinoTimestamp,
							(tVoid*) &temp_US_struct.RearRight.arduino_timestep);
				}

			}

			/** Transmission of all media samples over all pins**/
			//transmit media samples over output pins
			pNewMediaSampleFrontLeft->SetTime(pMediaSample->GetTime());
			US_FrontLeft_out.Transmit(pNewMediaSampleFrontLeft);
			pNewMediaSampleFrontCenterLeft->SetTime(pMediaSample->GetTime());
			US_FrontCenterLeft_out.Transmit(pNewMediaSampleFrontCenterLeft);
			pNewMediaSampleFrontCenter->SetTime(pMediaSample->GetTime());
			US_FrontCenter_out.Transmit(pNewMediaSampleFrontCenter);
			pNewMediaSampleFrontCenterRight->SetTime(pMediaSample->GetTime());
			US_FrontCenterRight_out.Transmit(pNewMediaSampleFrontCenterRight);
			pNewMediaSampleFrontRight->SetTime(pMediaSample->GetTime());
			US_FrontRight_out.Transmit(pNewMediaSampleFrontRight);
			pNewMediaSampleSideLeft->SetTime(pMediaSample->GetTime());
			US_SideLeft_out.Transmit(pNewMediaSampleSideLeft);
			pNewMediaSampleSideRight->SetTime(pMediaSample->GetTime());
			US_SideRight_out.Transmit(pNewMediaSampleSideRight);
			pNewMediaSampleRearLeft->SetTime(pMediaSample->GetTime());
			US_RearLeft_out.Transmit(pNewMediaSampleRearLeft);
			pNewMediaSampleRearCenter->SetTime(pMediaSample->GetTime());
			US_RearCenter_out.Transmit(pNewMediaSampleRearCenter);
			pNewMediaSampleRearRight->SetTime(pMediaSample->GetTime());
			US_RearRight_out.Transmit(pNewMediaSampleRearRight);

		}
	}

	RETURN_NOERROR;
}
