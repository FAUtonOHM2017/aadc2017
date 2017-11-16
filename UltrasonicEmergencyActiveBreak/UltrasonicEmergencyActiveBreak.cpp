/**
 * Copyright (c)
Audi Autonomous Driving Cup. TEAM FAUtonOHM.  All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

ADTF Filter for Emergency Stopping Functionality based on Ultrasonic Sensors
**********************************************************************
* $Author::  $ fink, hiller  $Date:: 2016-01-05 00:00:00#$ $Rev:: 1.2 (hiller)   $
**********************************************************************/

#include "stdafx.h"
#include "UltrasonicEmergencyActiveBreak.h"

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
ADTF_FILTER_PLUGIN("Ultrasonic Emergency Stop Active", OID_ADTF_EMERGENCY_STOP_ACTIVE, UltrasonicEmergencyActiveBreak);

UltrasonicEmergencyActiveBreak::UltrasonicEmergencyActiveBreak(const tChar* __info) :
		cFilter(__info) {

	// initial value for distance at which emergency shut down should be initiated (in metres)
	m_f32ThresValueFront = 0.5;
	m_f32ThresValueRear = 0.5;
	m_f32ThresValueSideLeft = 0.10;
	m_f32ThresValueSideRight = 0.10;

	// initial value to enable all US sensors
	m_boolThresFrontEnable = tTrue;
	m_boolThresRearEnable = tTrue;
	m_boolThresSideLeftEnable = tFalse;
	m_boolThresSideRightEnable = tFalse;

	// restart property
	m_boolDoNotRestartAfterObstacle = tTrue;

	// initial value to enable emergency out
	m_boolEmergency_output_enable = tFalse;

	// percentage that should be used for active breaking
	// emerg_output_speed = (-1)*(actualSpeed* PercentageActiveBreaking)
	m_f32PercentageActiveBreaking = 0;

	// initialization of memory sizes of media samples to create
	nSize_emergency = 0;
	nSize_tSignalValue = 0;

	// set 'obstacle detected' flag to false
	m_boolObstacleFlag = tFalse;

	// set 'hazard lights' to disabled mode (off=false)
	m_boolHazardLightsEnabled = tFalse;

	/** create the filter properties that have to be set in the Property Editor**/
	/** Properties for the front side**/
	SetPropertyFloat("Emergency threshold Front", m_f32ThresValueFront);
	SetPropertyFloat("Emergency threshold Front" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("Emergency threshold Front" NSSUBPROP_DESCRIPTION,
			"The value for front distance at which emergency shut down should be initiated (in meter)");

	SetPropertyBool("Enable Front", m_boolThresFrontEnable);
	SetPropertyBool("Enable Front" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("Enable Front" NSSUBPROP_DESCRIPTION, "Sensors will only be checked if option is ENABLED");

	/** Properties for the rear side**/
	SetPropertyFloat("Emergency threshold Rear", m_f32ThresValueRear);
	SetPropertyFloat("Emergency threshold Rear" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("Emergency threshold Rear" NSSUBPROP_DESCRIPTION,
			"The value for rear distance at which emergency shut down should be initiated (in meter)");

	SetPropertyBool("Enable Rear", m_boolThresRearEnable);
	SetPropertyBool("Enable Rear" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("Enable Rear" NSSUBPROP_DESCRIPTION, "Sensors will only be checked if option is ENABLED");

	/** Properties for the left hand side**/
	SetPropertyFloat("Emergency threshold Side Left", m_f32ThresValueSideLeft);
	SetPropertyFloat("Emergency threshold Side Left" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("Emergency threshold Side Left" NSSUBPROP_DESCRIPTION,
			"The value for left hand side distance at which emergency shut down should be initiated (in meter)");

	SetPropertyBool("Enable Side Left", m_boolThresSideLeftEnable);
	SetPropertyBool("Enable Side Left" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("Enable Side Left" NSSUBPROP_DESCRIPTION, "Sensors will only be checked if option is ENABLED");

	/** Properties for the right hand side**/
	SetPropertyFloat("Emergency threshold Side Right", m_f32ThresValueSideRight);
	SetPropertyFloat("Emergency threshold Side Right" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("Emergency threshold Side Right" NSSUBPROP_DESCRIPTION,
			"The value for right hand side distance at which emergency shut down should be initiated (in meter)");

	SetPropertyBool("Enable Side Right", m_boolThresSideRightEnable);
	SetPropertyBool("Enable Side Right" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("Enable Side Right" NSSUBPROP_DESCRIPTION, "Sensors will only be checked if option is ENABLED");

	/** Property to enable emergency out **/
	SetPropertyBool("Enable Emergency Output", m_boolEmergency_output_enable);
	SetPropertyBool("Enable Emergency Output" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("Enable Emergency Output" NSSUBPROP_DESCRIPTION,
			"Emergency Output is only active if option is ENABLED");

	/** Restart property for tSignalOutput **/
	SetPropertyBool("No Restart After Obstacle", m_boolDoNotRestartAfterObstacle);
	SetPropertyBool("No Restart After Obstacle" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("No Restart After Obstacle" NSSUBPROP_DESCRIPTION,
			"tSignalOutput remains 0 after obstacle is gone ");

	/** Percentage that should be used for active breaking**/
	SetPropertyFloat("Percentage for active breaking", m_f32PercentageActiveBreaking);
	SetPropertyFloat("Percentage for active breaking" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("Percentage for active breaking" NSSUBPROP_DESCRIPTION,
			"Percentage of actual speed that is applied for a active breaking; '0' means disabled, '1' is full break-force");

}

UltrasonicEmergencyActiveBreak::~UltrasonicEmergencyActiveBreak() {

}

tResult UltrasonicEmergencyActiveBreak::CreateInputPins(__exception)
{

	//get the media description manager for this filter
	cObjectPtr<IMediaDescriptionManager> pDescManager;
	RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

	/** Creation of the input pin speed controller **/

	//get description for tSignalValue input pins
	tChar const * strDesctSignalValue = pDescManager->GetMediaDescription("tSignalValue");
	// checks if exists
	RETURN_IF_POINTER_NULL(strDesctSignalValue);

	//get mediatype for tSignalValue
	cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDesctSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
	RETURN_IF_POINTER_NULL(pTypeSignalValue);
	//get mediatype description for tSignalValue data type, needed for locking
	RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescription_tSignalValue_IN));

	// Create input pin for speedControllerOut_input
	RETURN_IF_FAILED(speedController_input.Create("SpeedController_Input", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&speedController_input));

	// Create input pin for speedControllerOut_input
	RETURN_IF_FAILED(actualSpeed_input.Create("ActualSpeed_Input", pTypeSignalValue, static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&actualSpeed_input));

	/** Creation of the input pin ultrasonic struct **/

	//get description for tUltrasonicStruct input pins
	tChar const * strDesctUltrasonicStruct = pDescManager->GetMediaDescription("tUltrasonicStruct");
	// checks if exists
	RETURN_IF_POINTER_NULL(strDesctUltrasonicStruct);

	//get mediatype for tUltrasonicStruct
	cObjectPtr<IMediaType> pTypeUltrasonicStruct = new cMediaType(0, 0, 0, "tUltrasonicStruct", strDesctUltrasonicStruct,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
	RETURN_IF_POINTER_NULL(pTypeUltrasonicStruct);
	//get mediatype description for tUltrasonicStruct data type, needed for locking
	RETURN_IF_FAILED(pTypeUltrasonicStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionUltrasonicStruct));

	// Create input pin for ultrasonic sensor
	RETURN_IF_FAILED(ultrasonicStruct_input.Create("Ultrasonic_Struct", pTypeUltrasonicStruct, static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&ultrasonicStruct_input));

	RETURN_NOERROR;
}

tResult UltrasonicEmergencyActiveBreak::CreateOutputPins(__exception)
{
	//get the media description manager for this filter
	cObjectPtr<IMediaDescriptionManager> pDescManager;
	RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,IID_ADTF_MEDIA_DESCRIPTION_MANAGER,(tVoid**)&pDescManager,__exception_ptr));

	/** Creation of the output pin speed controller (emergency) **/

	//get description for tSignalValue output pin
	tChar const * strDescSignalValue = pDescManager->GetMediaDescription("tSignalValue");
	// checks if exists
	RETURN_IF_POINTER_NULL(strDescSignalValue);

	//get mediatype
	cObjectPtr<IMediaType> pTypeSignalValue = new cMediaType(0, 0, 0, "tSignalValue", strDescSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
	RETURN_IF_POINTER_NULL(pTypeSignalValue);
	//get mediatype description for tSignalValue data type, needed for locking
	RETURN_IF_FAILED(pTypeSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescription_tSignalValue_OUT));

	// create output pin for speed controller
	RETURN_IF_FAILED(speedControllerEmergency_output.Create("SpeedController_Output_Emergency", pTypeSignalValue, NULL));
	RETURN_IF_FAILED(RegisterPin(&speedControllerEmergency_output));

	/** Creation of the output pin emergency output (emergency) **/

	//get description for tJuryEmergencyStop output pin
	tChar const * strDescJuryEmergencyStop = pDescManager->GetMediaDescription("tJuryEmergencyStop");
	// checks if exists
	RETURN_IF_POINTER_NULL(strDescJuryEmergencyStop);

	//get mediatype
	cObjectPtr<IMediaType> pTypeJuryEmergencyStop = new cMediaType(0, 0, 0, "tJuryEmergencyStop", strDescJuryEmergencyStop,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
	RETURN_IF_POINTER_NULL(pTypeJuryEmergencyStop);
	//get mediatype description for tSignalValue data type, needed for locking
	RETURN_IF_FAILED(pTypeJuryEmergencyStop->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescription_JuryEmergencyStop));

	// create output pin for speed controller
	RETURN_IF_FAILED(emergency_output.Create("Emergency_Output", pTypeJuryEmergencyStop, NULL));
	RETURN_IF_FAILED(RegisterPin(&emergency_output));

	/** Creation of the output pin for enabling hazard lights **/

	//get description for tBoolSignalValue input pin
	tChar const * strDescBoolSignalValue = pDescManager->GetMediaDescription("tBoolSignalValue");
	// checks if exists
	RETURN_IF_POINTER_NULL(strDescBoolSignalValue);

	//get mediatype for tBoolSignalValue
	cObjectPtr<IMediaType> pTypeBoolSignalValue = new cMediaType(0, 0, 0, "tBoolSignalValue", strDescBoolSignalValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);
	RETURN_IF_POINTER_NULL(pTypeBoolSignalValue);
	//get mediatype description for tBoolSignalValue data type, needed for locking
	RETURN_IF_FAILED(pTypeBoolSignalValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescription_Bool_Signal_Value));

	// Create output pin for hazard lights
	RETURN_IF_FAILED(hazardLights.Create("Hazard_Lights", pTypeBoolSignalValue, NULL));
	RETURN_IF_FAILED(RegisterPin(&hazardLights));


	RETURN_NOERROR;
}

tResult UltrasonicEmergencyActiveBreak::Init(tInitStage eStage, __exception)
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
		m_f32ThresValueFront = tFloat32(GetPropertyFloat("Emergency threshold Front"));
		m_f32ThresValueRear = tFloat32(GetPropertyFloat("Emergency threshold Rear"));
		m_f32ThresValueSideLeft = tFloat32(GetPropertyFloat("Emergency threshold Side Left"));
		m_f32ThresValueSideRight = tFloat32(GetPropertyFloat("Emergency threshold Side Right"));
		m_boolThresFrontEnable = tBool(GetPropertyBool("Enable Front"));
		m_boolThresRearEnable = tBool(GetPropertyBool("Enable Rear"));
		m_boolThresSideLeftEnable = tBool(GetPropertyBool("Enable Side Left"));
		m_boolThresSideRightEnable = tBool(GetPropertyBool("Enable Side Right"));
		m_boolEmergency_output_enable = tBool(GetPropertyBool("Enable Emergency Output"));
		m_boolDoNotRestartAfterObstacle = tBool(GetPropertyBool("No Restart After Obstacle"));
		m_f32PercentageActiveBreaking = tFloat32(GetPropertyFloat("Percentage for active breaking"));

	}
	else if (eStage == StageGraphReady)
	{
		// All pin connections have been established in this stage so you can query your pins
		// about their media types and additional meta data.
		// Please take a look at the demo_imageproc example for further reference.
		tboolIDs_UltrasonicStructSet = tFalse;
		tboolIDs_tSignalValue_IN_Set = tFalse;
		tboolIDs_tSignalValue_OUT_Set = tFalse;
		tboolIDs_EmergencyOutput_Set = tFalse;
		tboolIDs_tBoolSignalValue_Set = tFalse;

		//allocate memory with the size given by the descriptor for JuryEmergencyStop
		cObjectPtr<IMediaSerializer> pSerializer_emergency;
		m_pDescription_JuryEmergencyStop->GetMediaSampleSerializer(&pSerializer_emergency);
		nSize_emergency = pSerializer_emergency->GetDeserializedSize();

		//allocate memory with the size given by the descriptor for tSignalValue
		cObjectPtr<IMediaSerializer> pSerializer;
		m_pDescription_tSignalValue_OUT->GetMediaSampleSerializer(&pSerializer);
		nSize_tSignalValue = pSerializer->GetDeserializedSize();

		//allocate memory with the size given by the descriptor for tBoolSignalValue
		cObjectPtr<IMediaSerializer> pSerializer_tBool;
		m_pDescription_Bool_Signal_Value->GetMediaSampleSerializer(&pSerializer_tBool);
		nSize_tBoolSignalValue = pSerializer_tBool->GetDeserializedSize();
	}

	RETURN_NOERROR;
}

tResult UltrasonicEmergencyActiveBreak::Start(__exception)
{
	return cFilter::Start(__exception_ptr);
}

tResult UltrasonicEmergencyActiveBreak::Stop(__exception)
{
	return cFilter::Stop(__exception_ptr);
}

tResult UltrasonicEmergencyActiveBreak::Shutdown(tInitStage eStage, __exception)
{
	// In each stage clean up everything that you initiaized in the corresponding stage during Init.
	// Pins are an exception:
	// - The base class takes care of static pins that are members of this class.
	// - Dynamic pins have to be cleaned up in the ReleasePins method, please see the demo_dynamicpin
	//   example for further reference.

	// call the base class implementation
	return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult UltrasonicEmergencyActiveBreak::ProcessUSInput(IMediaSample* pMediaSample){
	__synchronized_obj(m_oCriticalSectionUSdataAccess);
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
					pCoderInput->Get(tbufID_tFrontLeft, (tVoid*) &tmp_US_struct.FrontLeft);
					pCoderInput->Get(tbufID_tFrontCenterLeft, (tVoid*) &tmp_US_struct.FrontCenterLeft);
					pCoderInput->Get(tbufID_tFrontCenter, (tVoid*) &tmp_US_struct.FrontCenter);
					pCoderInput->Get(tbufID_tFrontCenterRight, (tVoid*) &tmp_US_struct.FrontCenterRight);
					pCoderInput->Get(tbufID_tFrontRight, (tVoid*) &tmp_US_struct.FrontRight);
					pCoderInput->Get(tbufID_tSideLeft, (tVoid*) &tmp_US_struct.SideLeft);
					pCoderInput->Get(tbufID_tSideRight, (tVoid*) &tmp_US_struct.SideRight);
					pCoderInput->Get(tbufID_tRearLeft, (tVoid*) &tmp_US_struct.RearLeft);
					pCoderInput->Get(tbufID_tRearCenter, (tVoid*) &tmp_US_struct.RearCenter);
					pCoderInput->Get(tbufID_tRearRight, (tVoid*) &tmp_US_struct.RearRight);

					//	 LOG_WARNING(cString::Format("data value of FrontCenterUS: %f",temp_US_struct.FrontCenter.d));

				}

	RETURN_NOERROR;
}

tResult UltrasonicEmergencyActiveBreak::ProcessActSpeedInput(IMediaSample* pMediaSample){
	__synchronized_obj(m_oCriticalSectionActSpeeddataAccess);
	{   // focus for sample read lock
					// read-out the incoming Media Sample
					__adtf_sample_read_lock_mediadescription(m_pDescription_tSignalValue_IN, pMediaSample, pCoderInput);

					// get the IDs for the items in the media sample
					if (!tboolIDs_tSignalValue_IN_Set) {
						pCoderInput->GetID("f32Value", tbufID_tSignalValue_IN_F32Value);
						pCoderInput->GetID("ui32ArduinoTimestamp", tbufID_tSignalValue_IN_ArduinoTimestamp);
						tboolIDs_tSignalValue_IN_Set = tTrue;
					}

					//get values from media sample
					pCoderInput->Get(tbufID_tSignalValue_IN_F32Value, (tVoid*) &tmp_actualSpeed.data_value);
					pCoderInput->Get(tbufID_tSignalValue_IN_ArduinoTimestamp, (tVoid*) &tmp_actualSpeed.arduino_timestamp);

				}
	RETURN_NOERROR;
}

tBool UltrasonicEmergencyActiveBreak::CheckUSValuesForStopSignal(){
	__synchronized_obj(m_oCriticalSectionUSdataAccess);
	tBool emergency_stop = tFalse;
	if ((m_boolThresFrontEnable == tTrue)
						&& (tmp_US_struct.FrontLeft.data_value < m_f32ThresValueFront
								|| tmp_US_struct.FrontCenterLeft.data_value < m_f32ThresValueFront
								|| tmp_US_struct.FrontCenter.data_value < m_f32ThresValueFront
								|| tmp_US_struct.FrontCenterRight.data_value < m_f32ThresValueFront
								|| tmp_US_struct.FrontRight.data_value < m_f32ThresValueFront)) {
					emergency_stop = tTrue;
				}
				/** check values at the back**/
				else if ((m_boolThresRearEnable == tTrue)
						&& (tmp_US_struct.RearLeft.data_value < m_f32ThresValueRear
								|| tmp_US_struct.RearCenter.data_value < m_f32ThresValueRear
								|| tmp_US_struct.RearRight.data_value < m_f32ThresValueRear)) {
					emergency_stop = tTrue;
				}

				/** check values at the left hand side**/
				else if ((m_boolThresSideLeftEnable == tTrue)
						&& (tmp_US_struct.SideLeft.data_value < m_f32ThresValueSideLeft)) {
					emergency_stop = tTrue;
				}

				/** check values at the right hand side**/
				else if ((m_boolThresSideRightEnable == tTrue)
						&& (tmp_US_struct.SideRight.data_value < m_f32ThresValueSideRight)) {
					emergency_stop = tTrue;
				}
	return emergency_stop;
}

tFloat32 UltrasonicEmergencyActiveBreak::CalcBreakSpeed(){
	__synchronized_obj(m_oCriticalSectionActSpeeddataAccess);
	tFloat32 retval;
	retval = -(tmp_actualSpeed.data_value * m_f32PercentageActiveBreaking);
	return retval;
}

tResult UltrasonicEmergencyActiveBreak::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2,
		IMediaSample* pMediaSample) {
	/** From cSEnsorAnalyzer.cpp :
	 __synchronized_obj(m_oProcessUsDataCritSection);
	 critical section for the processing of the ultrasonic data samples because function can be called from different onPinEvents
	 --> In Header definiert : cCriticalSection    m_oProcessUsDataCritSection; **/

	// so we received a media sample, so this pointer better be valid.
	RETURN_IF_POINTER_NULL(pMediaSample);
	RETURN_IF_POINTER_NULL(pSource);

	// first check what kind of event it is (and if output exists)
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {


		// by comparing it to our member pin variable we can find out which pin received
		// the sample
		// received an ultrasonic struct, save sample
		if (pSource == &ultrasonicStruct_input) {

			ProcessUSInput(pMediaSample);

			// received input sample on pin 'actual speed'
		} else if (pSource == &actualSpeed_input) {

			ProcessActSpeedInput(pMediaSample);

			// received input sample on pin 'speed controller'
		} else if (pSource == &speedController_input) {
			// tSignalValue struct variable to temporarily save current value
			tmp_tSignalValue SpeedControllerOut;

			{   // focus for sample read lock
				// read-out the incoming Media Sample
				__adtf_sample_read_lock_mediadescription(m_pDescription_tSignalValue_IN, pMediaSample, pCoderInput);

				// get the IDs for the items in the media sample
				if (!tboolIDs_tSignalValue_IN_Set) {
					pCoderInput->GetID("f32Value", tbufID_tSignalValue_IN_F32Value);
					pCoderInput->GetID("ui32ArduinoTimestamp", tbufID_tSignalValue_IN_ArduinoTimestamp);
					tboolIDs_tSignalValue_IN_Set = tTrue;
				}

				//get values from media sample
				pCoderInput->Get(tbufID_tSignalValue_IN_F32Value, (tVoid*) &SpeedControllerOut.data_value);
				pCoderInput->Get(tbufID_tSignalValue_IN_ArduinoTimestamp,
						(tVoid*) &SpeedControllerOut.arduino_timestamp);

			}

			tBool emergency_stop = tFalse;

			/** set controller output to zero if necessary (based on sensor values)**/
			/** check values at the front**/

				emergency_stop = CheckUSValuesForStopSignal();

			// emergency stop: set tSignalValue to desired value and set flag
			if (emergency_stop == tTrue) {
				if (m_f32PercentageActiveBreaking >= 0 && m_f32PercentageActiveBreaking <= 10) {
					SpeedControllerOut.data_value = CalcBreakSpeed();
				} else {
					SpeedControllerOut.data_value = 0.0;// done for safety reasons, in this way no extreme values are possible
				}
				m_boolObstacleFlag = tTrue;
				m_boolHazardLightsEnabled = tTrue;
			}

			/** tSignalValue output: remain on breaking mode when property is set, even after obstacle disappeared **/
			if (m_boolDoNotRestartAfterObstacle == tTrue && m_boolObstacleFlag == tTrue) {
				if (m_f32PercentageActiveBreaking >= 0 && m_f32PercentageActiveBreaking <= 10) {
					SpeedControllerOut.data_value = CalcBreakSpeed();
				} else {
					SpeedControllerOut.data_value = 0.0;// done for safety reasons, in this way no extreme values are possible
				}

			}

			// Transmit emergency output (according to property and input values)
			if (emergency_stop == tTrue && m_boolEmergency_output_enable == tTrue) {
				//LOG_WARNING(cString::Format("Emergency Stop"));
				TransmitEmergencyOutput(pMediaSample, emergency_stop);
			}


			// Transmit hazard lights output
			//create new media sample for hazard lights of type tBoolSignalValue
			TransmitHazardLights(pMediaSample);

			// Transmit Speed Controller Output
			//create new media sample for speed controller output
			TransmitSpeedControllerEmergencyOut(pMediaSample,SpeedControllerOut);

		}
	}

	RETURN_NOERROR;
}

tResult UltrasonicEmergencyActiveBreak::TransmitEmergencyOutput(IMediaSample* pMediaSample, tBool emergency_stop){
	// Set Emergency out (tBool)
	__synchronized_obj(m_oCriticalSectionTransmitEmergencyOut);
	//create new media sample for emergency output
					cObjectPtr<IMediaSample> pNewMediaSample_emergency;
					if (IS_OK(AllocMediaSample((tVoid** )&pNewMediaSample_emergency))) {
						pNewMediaSample_emergency->AllocBuffer(nSize_emergency);
						{   // focus for sample write lock
							//write date to the media sample with the coder of the descriptor
							__adtf_sample_write_lock_mediadescription(m_pDescription_JuryEmergencyStop,
									pNewMediaSample_emergency, pCoderOutput_emergency);

							// get the IDs for the items in the media sample
							if (!tboolIDs_EmergencyOutput_Set) {
								pCoderOutput_emergency->GetID("bEmergencyStop", tbufID_bEmergencyStop);
							}

							// set values in new media sample
							pCoderOutput_emergency->Set(tbufID_bEmergencyStop, (void*) &emergency_stop);
						}

						//transmit media sample over output pin
						pNewMediaSample_emergency->SetTime(pMediaSample->GetTime());
						emergency_output.Transmit(pNewMediaSample_emergency);
					}

	RETURN_NOERROR;
}

tResult UltrasonicEmergencyActiveBreak::TransmitHazardLights(IMediaSample* pMediaSample){
	// Set hazard lights output
	__synchronized_obj(m_oCriticalSectionTransmitHazardLights);
	//create new media sample for hazard lights of type tBoolSignalValue
	cObjectPtr<IMediaSample> pNewMediaSample_hazardLights;
				if (IS_OK(AllocMediaSample((tVoid** )&pNewMediaSample_hazardLights))) {
					pNewMediaSample_hazardLights->AllocBuffer(nSize_tBoolSignalValue);
					{   // focus for sample write lock
						//write date to the media sample with the coder of the descriptor
						__adtf_sample_write_lock_mediadescription(m_pDescription_Bool_Signal_Value,
								pNewMediaSample_hazardLights, pCoderOutput_hazardLights);

						// get the IDs for the items in the media sample
						if (!tboolIDs_tBoolSignalValue_Set) {
							pCoderOutput_hazardLights->GetID("ui32ArduinoTimestamp", tbufID_tBoolSignalValue_ArduinoTimestamp);
							pCoderOutput_hazardLights->GetID("bValue", tbufID_tBoolSignalValue_Bool);
							tboolIDs_tBoolSignalValue_Set = tTrue;
						}
						tUInt32 arduinoTime = (tUInt32) (pMediaSample->GetTime());
						// set values in new media sample, timestamp of speedcontroller is used for arduino
						pCoderOutput_hazardLights->Set(tbufID_tBoolSignalValue_ArduinoTimestamp, (void*) &arduinoTime);
						pCoderOutput_hazardLights->Set(tbufID_tBoolSignalValue_Bool, (void*) &m_boolHazardLightsEnabled);

					}

					//transmit media sample over output pin
					pNewMediaSample_hazardLights->SetTime(pMediaSample->GetTime());
					hazardLights.Transmit(pNewMediaSample_hazardLights);
				}
		RETURN_NOERROR;
}

tResult UltrasonicEmergencyActiveBreak::TransmitSpeedControllerEmergencyOut(IMediaSample* pMediaSample, tmp_tSignalValue SpeedControllerOut){
	// Set speedController Emergency Output
	__synchronized_obj(m_oCriticalSectionTransmitSpeedControllerEmergency);
		//create new media sample for speed controller output
			cObjectPtr<IMediaSample> pNewMediaSample_SCout;
			if (IS_OK(AllocMediaSample((tVoid** )&pNewMediaSample_SCout))) {
				pNewMediaSample_SCout->AllocBuffer(nSize_tSignalValue);
				{   // focus for sample write lock
					//write date to the media sample with the coder of the descriptor
					__adtf_sample_write_lock_mediadescription(m_pDescription_tSignalValue_OUT, pNewMediaSample_SCout,
							pCoderOutput);

					// get the IDs for the items in the media sample
					if (!tboolIDs_tSignalValue_OUT_Set) {
						pCoderOutput->GetID("f32Value", tbufID_tSignalValue_OUT_F32Value);
						pCoderOutput->GetID("ui32ArduinoTimestamp", tbufID_tSignalValue_OUT_ArduinoTimestamp);
						tboolIDs_tSignalValue_OUT_Set = tTrue;
					}

					// set values in new media sample
					pCoderOutput->Set(tbufID_tSignalValue_OUT_F32Value, (tVoid*) &(SpeedControllerOut.data_value));
					pCoderOutput->Set(tbufID_tSignalValue_OUT_ArduinoTimestamp,
							(tVoid*) &SpeedControllerOut.arduino_timestamp);
				}

				//transmit media sample over output pin
				pNewMediaSample_SCout->SetTime(pMediaSample->GetTime());
				speedControllerEmergency_output.Transmit(pNewMediaSample_SCout);
			}
		RETURN_NOERROR;
}
