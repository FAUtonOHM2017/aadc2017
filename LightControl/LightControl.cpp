/**
 * Copyright (c)
Audi Autonomous Driving Cup. TEAM FAUtonOHM.  All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3. All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4. Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

ADTF Filter for controlling car lights with state machine controller
**********************************************************************
* $Author::  $ fink  $Date:: 2016-01-25 00:00:00#$ $Rev:: 1.0.0   $
**********************************************************************/

#include "math.h"
#include "stdafx.h"
#include "LightControl.h"

/// Create filter shell
ADTF_FILTER_PLUGIN("Light Control", OID_ADTF_LIGHT_CONTROL, LightControl);

LightControl::LightControl(const tChar* __info):cFilter(__info)
{

	factorEnableBrake = 0.95; // should be less than 1
	factorDisableBrake = 1.04; // should be greater than 1
	averageSampleCount = 10; // number of samples used to calculate speed average

    SetPropertyFloat("Factor for enabling brake lights",0.95);
    SetPropertyFloat("Factor for enabling brake lights" NSSUBPROP_REQUIRED, tTrue);
    SetPropertyStr("Factor for enabling brake lights" NSSUBPROP_DESCRIPTION, "Factor for enabling brake lights, should be less than 1");

    SetPropertyFloat("Factor for disabling brake lights",1.04);
    SetPropertyFloat("Factor for disabling brake lights" NSSUBPROP_REQUIRED, tTrue);
    SetPropertyStr("Factor for disabling brake lights" NSSUBPROP_DESCRIPTION, "Factor for disabling brake lights, should be greater than 1");

    SetPropertyInt("Speed Average Count",averageSampleCount);
    SetPropertyInt("Speed Average Count" NSSUBPROP_REQUIRED, tTrue);
    SetPropertyStr("Speed Average Count" NSSUBPROP_DESCRIPTION, "The number of measurements used for average speed calculation (used for brake lights)");
}

LightControl::~LightControl()
{

}

tResult LightControl::CreateInputPins(__exception)
{
	RETURN_IF_FAILED(actionInput.Create("action", tActionStruct.GetMediaType(), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&actionInput));

	RETURN_IF_FAILED(carspeedInput.Create("car_speed", CarSpeed.GetMediaType(), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&carspeedInput));

	RETURN_IF_FAILED(setspeedInput.Create("set_speed", SetSpeed.GetMediaType(), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&setspeedInput));

	RETURN_NOERROR;
}

tResult LightControl::CreateOutputPins(__exception)
{

	// create output pin for statemachine
	RETURN_IF_FAILED(feedbackOutput.Create("feedback", tFeedbackStruct.GetMediaType(), NULL));
	RETURN_IF_FAILED(RegisterPin(&feedbackOutput));

	// create tBoolSignalValues for light outputs
	RETURN_IF_FAILED(headLightOutput.Create("headLight", HeadLight.GetMediaType(), NULL));
	RETURN_IF_FAILED(RegisterPin(&headLightOutput));

	RETURN_IF_FAILED(reverseLightOutput.Create("reverseLight", ReverseLight.GetMediaType(), NULL));
	RETURN_IF_FAILED(RegisterPin(&reverseLightOutput));

	RETURN_IF_FAILED(brakeLightOutput.Create("brakeLight", BrakeLight.GetMediaType(), NULL));
	RETURN_IF_FAILED(RegisterPin(&brakeLightOutput));

	RETURN_IF_FAILED(turnRightLightOutput.Create("turnRightLight", TurnRightLight.GetMediaType(), NULL));
	RETURN_IF_FAILED(RegisterPin(&turnRightLightOutput));

	RETURN_IF_FAILED(turnLeftLightOutput.Create("turnLeftLight", TurnLeftLight.GetMediaType(), NULL));
	RETURN_IF_FAILED(RegisterPin(&turnLeftLightOutput));

	RETURN_IF_FAILED(hazardLightOutput.Create("hazardLight", HazardLight.GetMediaType(), NULL));
	RETURN_IF_FAILED(RegisterPin(&hazardLightOutput));

	RETURN_NOERROR;
}


tResult LightControl::Init(tInitStage eStage, __exception)
{
	// never miss calling the parent implementation!!
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

	// in StageFirst you can create and register your static pins.
	if (eStage == StageFirst)
	{

		RETURN_IF_FAILED(tActionStruct.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(tFeedbackStruct.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(CarSpeed.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(SetSpeed.StageFirst(__exception_ptr));

		RETURN_IF_FAILED(HeadLight.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(ReverseLight.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(BrakeLight.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(TurnRightLight.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(TurnLeftLight.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(HazardLight.StageFirst(__exception_ptr));

		// create and register the input and output pin
		RETURN_IF_FAILED(CreateInputPins(__exception_ptr));
		RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));

	}
	else if (eStage == StageNormal)
	{
		// get Properties from user interface
		factorEnableBrake = tFloat32(GetPropertyFloat("Factor for enabling brake lights"));
		factorDisableBrake = tFloat32(GetPropertyFloat("Factor for disabling brake lights"));
		averageSampleCount = tUInt32(GetPropertyInt("Speed Average Count"));

	}
	else if (eStage == StageGraphReady)
	{
		// All pin connections have been established in this stage so you can query your pins
		// about their media types and additional meta data.
		// Please take a look at the demo_imageproc example for further reference.
		RETURN_IF_FAILED(tActionStruct.StageGraphReady());
		RETURN_IF_FAILED(tFeedbackStruct.StageGraphReady());
		RETURN_IF_FAILED(CarSpeed.StageGraphReady());
		RETURN_IF_FAILED(SetSpeed.StageGraphReady());

		RETURN_IF_FAILED(HeadLight.StageGraphReady());
		RETURN_IF_FAILED(ReverseLight.StageGraphReady());
		RETURN_IF_FAILED(BrakeLight.StageGraphReady());
		RETURN_IF_FAILED(TurnRightLight.StageGraphReady());
		RETURN_IF_FAILED(TurnLeftLight.StageGraphReady());
		RETURN_IF_FAILED(HazardLight.StageGraphReady());

		// TODO deactivate all lights on startup
		brakelightEnabled = tFalse;
		reverselightEnabled = tFalse;
	}

	RETURN_NOERROR;
	}


tResult LightControl::Start(__exception)
{
	return cFilter::Start(__exception_ptr);
}

tResult LightControl::Stop(__exception)
{
	return cFilter::Stop(__exception_ptr);
}


tResult LightControl::Shutdown(tInitStage eStage, __exception)
{
	// In each stage clean up everything that you initiaized in the corresponding stage during Init.
	// Pins are an exception:
	// - The base class takes care of static pins that are members of this class.
	// - Dynamic pins have to be cleaned up in the ReleasePins method, please see the demo_dynamicpin
	//   example for further reference.


	// call the base class implementation
	return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult LightControl::ProcessSpeedInput(IMediaSample* mediaSample) {

	// get car speed
	TSignalValue::Data rec_carspeed;
	CarSpeed.Read(mediaSample, &rec_carspeed);

	// save received car speed value
	last_carspeeds.push_back(fabsf(rec_carspeed.f32_value));
	if (last_carspeeds.size() > averageSampleCount) {
		last_carspeeds.pop_front();
	}

	// average calculation
	tFloat32 average_speed = 0;
	for (std::list<tFloat32>::iterator p = last_carspeeds.begin(); p != last_carspeeds.end(); ++p) {
		average_speed += (tFloat32)*p;
	}

	tFloat32 speed_average = average_speed / last_carspeeds.size();

	// enable brake lights
	if ( fabsf(rec_carspeed.f32_value) < ( speed_average * factorEnableBrake) && !brakelightEnabled ) {
		tFloat32 diff = (rec_carspeed.f32_value) - ( speed_average * factorEnableBrake);
		if(diff < -0.5 )
		{
			TBoolSignalValue::Data BrakeLightData;
			BrakeLightData.ui32_arduinoTimestamp = 0;
			BrakeLightData.bValue = tTrue;

			BrakeLight.Transmit(&brakeLightOutput, BrakeLightData, _clock->GetStreamTime() );
			brakelightEnabled = tTrue;

		}
	}

	// disable brake lights
	if ( fabsf(rec_carspeed.f32_value) > ( speed_average * factorDisableBrake ) && brakelightEnabled ) {
		TBoolSignalValue::Data BrakeLightData;
		BrakeLightData.ui32_arduinoTimestamp = 0;
		BrakeLightData.bValue = tFalse;

		BrakeLight.Transmit(&brakeLightOutput, BrakeLightData, _clock->GetStreamTime() );
		brakelightEnabled = tFalse;
	}

	RETURN_NOERROR;
}

tResult LightControl::ProcessSetSpeedInput(IMediaSample* mediaSample) {

	// get set car speed
	TSignalValue::Data rec_setcarspeed;
	SetSpeed.Read(mediaSample, &rec_setcarspeed);

	// enable reverse lights
	if (rec_setcarspeed.f32_value < 0 && !reverselightEnabled ) {
		TBoolSignalValue::Data ReverseLightData;
		ReverseLightData.ui32_arduinoTimestamp = 0;
		ReverseLightData.bValue = tTrue;

		ReverseLight.Transmit(&reverseLightOutput, ReverseLightData, _clock->GetStreamTime() );
		reverselightEnabled = tTrue;
	}

	// disable reverse lights
	if ( rec_setcarspeed.f32_value >= 0 && reverselightEnabled ) {
		TBoolSignalValue::Data ReverseLightData;
		ReverseLightData.ui32_arduinoTimestamp = 0;
		ReverseLightData.bValue = tFalse;

		ReverseLight.Transmit(&reverseLightOutput, ReverseLightData, _clock->GetStreamTime() );
		reverselightEnabled = tFalse;
	}

	RETURN_NOERROR;
}

tResult LightControl::ProcessAction(IMediaSample* mediaSample) {

	TActionStruct::ActionSub actionSub;
	actionSub = tActionStruct.Read_Action(mediaSample, F_LIGHT_FILTER);

	TFeedbackStruct::Data feedback;
	feedback.ui32_filterID = F_LIGHT_FILTER;

	if(actionSub.enabled == tTrue && actionSub.started == tTrue) {
		// Light Command: 7000 - 7999
		// 7 a b c
		// a	Head Light 0: no change, 1: disable, 2: enable
		// b	Reverse Light: 0: no change, 1: disable, 2: enable
		// c	Turn Lights: 0: no change, 1: disable, 2: left turn, 3: right turn, 4: hazard

		// check command input (range 7000 to 7999)
		if(actionSub.command > F_LIGHT_FILTER * 1000 && actionSub.command < (F_LIGHT_FILTER + 1) * 1000 ) {

			tUInt32 a_headLight = ( actionSub.command % 1000 ) / 100;
			tUInt32 b_reverseLight = ( actionSub.command % 100 ) / 10;
			tUInt32 c_turnLight = actionSub.command % 10;

			// output clock for mediasamples
			tTimeStamp cur_time = _clock->GetStreamTime();

			//////////////////////
			// process head light
			/////////////////////
			TBoolSignalValue::Data headLightData;
			headLightData.ui32_arduinoTimestamp = 0;

			if ( a_headLight == 0 ) {
				// do nothing
			} else if ( a_headLight == 1 ) {
				headLightData.bValue = tFalse;
				HeadLight.Transmit(&headLightOutput, headLightData, cur_time);
			} else if (a_headLight == 2) {
				headLightData.bValue = tTrue;
				HeadLight.Transmit(&headLightOutput, headLightData, cur_time);
			} else {
				LOG_ERROR(cString::Format("LightControl: Head lights, invalid command %d", a_headLight));
				RETURN_ERROR(ERR_INVALID_STATE);
			}

			//////////////////////
			// process reverse light
			/////////////////////
			TBoolSignalValue::Data reverseLightData;
			reverseLightData.ui32_arduinoTimestamp = 0;

			if ( b_reverseLight == 0 ) {
				// do nothing
			} else if ( b_reverseLight == 1 ) {
				reverseLightData.bValue = tFalse;
				ReverseLight.Transmit(&reverseLightOutput, reverseLightData, cur_time);
			} else if (b_reverseLight == 2) {
				reverseLightData.bValue = tTrue;
				ReverseLight.Transmit(&reverseLightOutput, reverseLightData, cur_time);
			} else {
				LOG_ERROR(cString::Format("LightControl: Reverse lights, invalid command %d", b_reverseLight));
				RETURN_ERROR(ERR_INVALID_STATE);
			}

			//////////////////////
			// process turn lights
			/////////////////////
			TBoolSignalValue::Data TurnRightLightData;
			TBoolSignalValue::Data TurnLeftLightData;
			TBoolSignalValue::Data HazardLightData;

			TurnLeftLightData.ui32_arduinoTimestamp = 0;
			TurnRightLightData.ui32_arduinoTimestamp = 0;
			HazardLightData.ui32_arduinoTimestamp = 0;

			if ( c_turnLight == 0 ) {
				// do nothing
			} else if ( c_turnLight == 1 ) {
				// disable all
				TurnLeftLightData.bValue = tFalse;
				TurnRightLightData.bValue = tFalse;
				HazardLightData.bValue = tFalse;
				TurnLeftLight.Transmit(&turnLeftLightOutput, TurnLeftLightData, cur_time);
				TurnRightLight.Transmit(&turnRightLightOutput, TurnRightLightData, cur_time);
				HazardLight.Transmit(&hazardLightOutput, HazardLightData, cur_time);
			} else if (c_turnLight == 2) {
				// left turn
				TurnLeftLightData.bValue = tTrue;
				TurnRightLightData.bValue = tFalse;
				HazardLightData.bValue = tFalse;
				TurnRightLight.Transmit(&turnRightLightOutput, TurnRightLightData, cur_time);
				HazardLight.Transmit(&hazardLightOutput, HazardLightData, cur_time);
				TurnLeftLight.Transmit(&turnLeftLightOutput, TurnLeftLightData, cur_time);
			} else if (c_turnLight == 3) {
				// right turn
				TurnLeftLightData.bValue = tFalse;
				TurnRightLightData.bValue = tTrue;
				HazardLightData.bValue = tFalse;
				TurnLeftLight.Transmit(&turnLeftLightOutput, TurnLeftLightData, cur_time);
				HazardLight.Transmit(&hazardLightOutput, HazardLightData, cur_time);
				TurnRightLight.Transmit(&turnRightLightOutput, TurnRightLightData, cur_time);
			} else if (c_turnLight == 4) {
				// hazard light
				TurnLeftLightData.bValue = tFalse;
				TurnRightLightData.bValue = tFalse;
				HazardLightData.bValue = tTrue;
				TurnLeftLight.Transmit(&turnLeftLightOutput, TurnLeftLightData, cur_time);
				TurnRightLight.Transmit(&turnRightLightOutput, TurnRightLightData, cur_time);
				HazardLight.Transmit(&hazardLightOutput, HazardLightData, cur_time);
			} else {
				LOG_ERROR(cString::Format("LightControl: turn lights, invalid command %d", c_turnLight));
				RETURN_ERROR(ERR_INVALID_STATE);
			}

			feedback.ui32_status = actionSub.command; // TODO send status? enum
			tFeedbackStruct.Transmit(&feedbackOutput, feedback, cur_time);

		} else {
				LOG_ERROR(cString::Format("LightControl: Received command %d, expected command in range %d to %d", actionSub.command,  F_LIGHT_FILTER * 1000, (F_LIGHT_FILTER + 1) * 1000 ));
		}
	}


	RETURN_NOERROR;
}

tResult LightControl::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
	//tTimeStamp inputTime = _clock ? _clock->GetStreamTime() : adtf_util::cHighResTimer::GetTime();
	__synchronized_obj(criticalSection_OnPinEvent);

	 // first check what kind of event it is
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
		RETURN_IF_POINTER_NULL(pMediaSample);
		RETURN_IF_POINTER_NULL(pSource);

		// by comparing it to our member pin variable we can find out which pin received
		if (pSource == &actionInput) {
			// process action input
			ProcessAction(pMediaSample);
		} else if (pSource == &carspeedInput) {
			// process wheel speed input and set brake light
			ProcessSpeedInput(pMediaSample);
		}  else if (pSource == &setspeedInput) {
			// process set car speed input and set reverse light
			ProcessSetSpeedInput(pMediaSample);
		}
	}

	RETURN_NOERROR;
}
