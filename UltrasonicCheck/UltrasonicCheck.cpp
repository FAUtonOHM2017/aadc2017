/**
 * Copyright (c)
Audi Autonomous Driving Cup. TEAM FAUtonOHM.  All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3. All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4. Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

ADTF Filter for checking ultrasonic sensors on state machine request
**********************************************************************
* $Author::  $ fink  $Date:: 2016-02-09 00:00:00#$ $Rev:: 1.0.0   $
**********************************************************************/

#include "stdafx.h"
#include "UltrasonicCheck.h"

/// Create filter shell
ADTF_FILTER_PLUGIN("Light Control", OID_ADTF_ULTRASONIC_CHECK, UltrasonicCheck);

#define ERROR_RATE_THRESHOLD "General::Error Rate Threshold"

UltrasonicCheck::UltrasonicCheck(const tChar* __info):cFilter(__info)

{

	m_bDebugModeEnabled = tFalse; // debug messsages
	number_of_samples = 20; // number of samples used for obstacle detection (US: 20 samples/s)
	number_of_samples_overtaking = 5; // number of samples used for overtaking detection (US: 20 samples/s)
	no_obstacle_threshold = 0.7; // samples (in percent) that have to be clear
	parking_space_long_threshold = 0.3;
	parking_space_trans_threshold = 0.4;
	giveway_threshold = 0.6;
	overtaking_threshold = 0.3;

	SetPropertyBool("General::Enable Debug", m_bDebugModeEnabled);
	SetPropertyBool("General::Enable Debug" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("General::Enable Debug" NSSUBPROP_DESCRIPTION,
				"Enables Debug messages");

	SetPropertyInt("General::Number Of Samples", number_of_samples);
	SetPropertyInt("General::Number Of Samples" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("General::Number Of Samples" NSSUBPROP_DESCRIPTION,
				"Number of samples used for obstacle detection");

	SetPropertyInt("General::Number Of Samples Overtaking", number_of_samples_overtaking);
	SetPropertyInt("General::Number Of Samples Overtaking" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("General::Number Of Samples Overtaking" NSSUBPROP_DESCRIPTION,
				"Number of samples used for overtaking detection");

	SetPropertyFloat("General::No Obstacle Threshold", 0.7);
	SetPropertyFloat("General::No Obstacle Threshold" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("General::No Obstacle Threshold" NSSUBPROP_DESCRIPTION,
				"Samples (in percent) that have to be clear");

        SetPropertyFloat(ERROR_RATE_THRESHOLD, errorRateThreshold = 0.3);
        SetPropertyFloat(ERROR_RATE_THRESHOLD NSSUBPROP_REQUIRED, tTrue);
        SetPropertyStr(ERROR_RATE_THRESHOLD NSSUBPROP_DESCRIPTION,
                                "percentage of error samples till measurement is deemed false");

	SetPropertyFloat("Distances::Parking Space Long Threshold", 0.3);
	SetPropertyFloat("Distances::Parking Space Long Threshold" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("Distances::Parking Space Long Threshold" NSSUBPROP_DESCRIPTION,
				"US Value > Threshold: Parking Space Long No Obstacle");

	SetPropertyFloat("Distances::Parking Space Trans Threshold", 0.5);
	SetPropertyFloat("Distances::Parking Space Trans Threshold" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("Distances::Parking Space Trans Threshold" NSSUBPROP_DESCRIPTION,
				"US Value > Threshold: Parking Space Trans No Obstacle");

	SetPropertyFloat("Distances::Giveway Left Threshold", 0.8);
	SetPropertyFloat("Distances::Giveway Left Threshold" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("Distances::Giveway Left Threshold" NSSUBPROP_DESCRIPTION,
				"US Value > Threshold: Intersection Left No Obstacle");

	SetPropertyFloat("Distances::Overtaking Obstacle Threshold", 0.4);
	SetPropertyFloat("Distances::Overtaking Obstacle Threshold" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("Distances::Overtaking Obstacle Threshold" NSSUBPROP_DESCRIPTION,
				"US Value > Threshold: Overtaking No Obstacle (Right Sensor)");
}

UltrasonicCheck::~UltrasonicCheck()
{

}

tResult UltrasonicCheck::CreateInputPins(__exception)
{
	RETURN_IF_FAILED(actionInput.Create("action", tActionStruct.GetMediaType(), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&actionInput));

	RETURN_IF_FAILED(ultrasonicInput.Create("ultrasonic_struct", UltrasonicStruct.GetMediaType(), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&ultrasonicInput));

	RETURN_NOERROR;
}

tResult UltrasonicCheck::CreateOutputPins(__exception)
{
	// create output pin for statemachine
	RETURN_IF_FAILED(feedbackOutput.Create("feedback", tFeedbackStruct.GetMediaType(), NULL));
	RETURN_IF_FAILED(RegisterPin(&feedbackOutput));

	RETURN_NOERROR;
}


tResult UltrasonicCheck::Init(tInitStage eStage, __exception)
{
	// never miss calling the parent implementation!!
	RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr))

	// in StageFirst you can create and register your static pins.
	if (eStage == StageFirst)
	{

		RETURN_IF_FAILED(tActionStruct.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(tFeedbackStruct.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(UltrasonicStruct.StageFirst(__exception_ptr));

		// create and register the input and output pin
		RETURN_IF_FAILED(CreateInputPins(__exception_ptr));
		RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));

	}
	else if (eStage == StageNormal)
	{
		// get Properties from user interface
		m_bDebugModeEnabled = tBool(GetPropertyBool("General::Enable Debug"));
		number_of_samples = tUInt32(GetPropertyInt("General::Number Of Samples"));
		number_of_samples_overtaking = tUInt32(GetPropertyInt("General::Number Of Samples Overtaking"));
		no_obstacle_threshold = tFloat32(GetPropertyFloat("General::No Obstacle Threshold"));
		parking_space_long_threshold = tFloat32(GetPropertyFloat("Distances::Parking Space Long Threshold"));
		parking_space_trans_threshold = tFloat32(GetPropertyFloat("Distances::Parking Space Trans Threshold"));
		giveway_threshold = tFloat32(GetPropertyFloat("Distances::Giveway Left Threshold"));
		overtaking_threshold = tFloat32(GetPropertyFloat("Distances::Overtaking Obstacle Threshold"));
	}
	else if (eStage == StageGraphReady)
	{
		// All pin connections have been established in this stage so you can query your pins
		// about their media types and additional meta data.
		// Please take a look at the demo_imageproc example for further reference.
		RETURN_IF_FAILED(tActionStruct.StageGraphReady());
		RETURN_IF_FAILED(tFeedbackStruct.StageGraphReady());
		RETURN_IF_FAILED(UltrasonicStruct.StageGraphReady());

		running = tFalse; // set running state
		command = 0; // received command
		sample_counter = 0; // reset sample counter
		no_obstacle_counter = 0; // count samples without obstacles
		error_counter = 0; // count samples with US distance < 0.03
	}

	RETURN_NOERROR;
	}


tResult UltrasonicCheck::Start(__exception)
{
	return cFilter::Start(__exception_ptr);
}

tResult UltrasonicCheck::Stop(__exception)
{
	return cFilter::Stop(__exception_ptr);
}


tResult UltrasonicCheck::Shutdown(tInitStage eStage, __exception)
{
	// In each stage clean up everything that you initiaized in the corresponding stage during Init.
	// Pins are an exception:
	// - The base class takes care of static pins that are members of this class.
	// - Dynamic pins have to be cleaned up in the ReleasePins method, please see the demo_dynamicpin
	//   example for further reference.


	// call the base class implementation
	return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult UltrasonicCheck::ProcessUltrasonicInput(IMediaSample* mediaSample) {
	__synchronized_obj(m_oCriticalSectionUltrasonicInput);

	// get ultrasonic sensor input
	TUltrasonicStruct::Data rec_ultrasonic;
	UltrasonicStruct.Read(mediaSample, &rec_ultrasonic);
        rec_ultrasonic.FrontCenter.f32_value /= 100;
        rec_ultrasonic.FrontCenterLeft.f32_value /= 100;
        rec_ultrasonic.FrontCenterRight.f32_value /= 100;
        rec_ultrasonic.FrontLeft.f32_value /= 100;
        rec_ultrasonic.FrontRight.f32_value /= 100;
        rec_ultrasonic.RearCenter.f32_value /= 100;
        rec_ultrasonic.RearLeft.f32_value /= 100;
        rec_ultrasonic.RearRight.f32_value /= 100;
        rec_ultrasonic.SideLeft.f32_value /= 100;
        rec_ultrasonic.SideRight.f32_value /= 100;

	/*
	if(m_bDebugModeEnabled) {
		LOG_WARNING(cString::Format("Ultrasonic Check input sensor values: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f",
				rec_ultrasonic.FrontCenter.f32_value,
				rec_ultrasonic.FrontCenterLeft.f32_value,
				rec_ultrasonic.FrontCenterRight.f32_value,
				rec_ultrasonic.FrontLeft.f32_value,
				rec_ultrasonic.FrontRight.f32_value,
				rec_ultrasonic.RearCenter.f32_value,
				rec_ultrasonic.RearLeft.f32_value,
				rec_ultrasonic.RearRight.f32_value,
				rec_ultrasonic.SideLeft.f32_value,
				rec_ultrasonic.SideRight.f32_value));
	}
	*/

	switch (command) {
		case AC_UC_CHECK_OVERTAKING_OBSTACLE:
			if(rec_ultrasonic.SideRight.f32_value > overtaking_threshold) {
				no_obstacle_counter++;
			} else if (rec_ultrasonic.SideRight.f32_value < 0.03) {
				//error_counter++; //TODO check this!
				if(m_bDebugModeEnabled) {
					LOG_WARNING(cString::Format("Ultrasonic Check Overtaking: error in sensor side right value: %f", rec_ultrasonic.SideRight.f32_value));
				}
			}
			break;
		case AC_UC_CHECK_GIVEWAY_LEFT:
			if(rec_ultrasonic.FrontLeft.f32_value > giveway_threshold) {
				no_obstacle_counter++;
			} else if (rec_ultrasonic.FrontLeft.f32_value < 0.03) {
				error_counter++;
				if(m_bDebugModeEnabled) {
					LOG_WARNING(cString::Format("Ultrasonic Check Giveway Left: error in sensor front left value: %f", rec_ultrasonic.FrontLeft.f32_value));
				}
			}
			break;
		case AC_UC_CHECK_PARKING_SPACE_LONG:
			if(rec_ultrasonic.SideRight.f32_value > parking_space_long_threshold) {
				no_obstacle_counter++;
			} else if (rec_ultrasonic.SideRight.f32_value < 0.03) {
				error_counter++;
				if(m_bDebugModeEnabled) {
					LOG_WARNING(cString::Format("Ultrasonic Check Parking Space Long: error in sensor side right value: %f", rec_ultrasonic.SideRight.f32_value));
				}
			}
			break;
		case AC_UC_CHECK_PARKING_SPACE_TRANS:
			if(rec_ultrasonic.SideRight.f32_value > parking_space_trans_threshold) {
				no_obstacle_counter++;
			} else if (rec_ultrasonic.SideRight.f32_value < 0.03) {
				error_counter++;
				if(m_bDebugModeEnabled) {
					LOG_WARNING(cString::Format("Ultrasonic Check Parking Space Trans: error in sensor side right value: %f", rec_ultrasonic.SideRight.f32_value));
				}
			}
			break;
		case AC_UC_CHECK_PARKING_TRANS_FRONT:
			if(rec_ultrasonic.FrontLeft.f32_value > 0.3) {
				no_obstacle_counter++;
			}
			if(rec_ultrasonic.FrontCenterLeft.f32_value > 0.3) {
				no_obstacle_counter++;
			}
			if (rec_ultrasonic.FrontLeft.f32_value < 0.03 || rec_ultrasonic.FrontCenterLeft.f32_value < 0.03) {
				error_counter++;
				if(m_bDebugModeEnabled) {
					LOG_WARNING(cString::Format("Ultrasonic Check Parking Trans Front: error in sensors, front left value: %f, front center left: %f", rec_ultrasonic.FrontLeft.f32_value, rec_ultrasonic.FrontCenterLeft.f32_value));
				}
			}
			break;
		case AC_UC_CHECK_PULL_OUT_LONG:
			if(rec_ultrasonic.FrontLeft.f32_value > 0.4) {
				no_obstacle_counter++;
			}
			if(rec_ultrasonic.SideLeft.f32_value > 0.3) {
				no_obstacle_counter++;
			}
			if(rec_ultrasonic.RearLeft.f32_value > 0.4) {
				no_obstacle_counter++;
			}
			if (rec_ultrasonic.FrontLeft.f32_value < 0.03 || rec_ultrasonic.SideLeft.f32_value < 0.03 || rec_ultrasonic.RearLeft.f32_value < 0.03) {
				error_counter+=3;
				if(m_bDebugModeEnabled) {
					LOG_WARNING(cString::Format("Ultrasonic Check: error in sensor front left, side left or rear left; values: %f, %f, %f", rec_ultrasonic.FrontLeft.f32_value, rec_ultrasonic.SideLeft.f32_value, rec_ultrasonic.RearLeft.f32_value));
				}
			}
			break;
		case AC_UC_CHECK_PULL_OUT_TRANS:
			if(rec_ultrasonic.FrontLeft.f32_value > 1.0) {
				no_obstacle_counter++;
			}
			if(rec_ultrasonic.FrontCenterLeft.f32_value > 0.8) {
				no_obstacle_counter++;
			}
			if(rec_ultrasonic.FrontCenter.f32_value > 0.7) {
				no_obstacle_counter++;
			}
			if(rec_ultrasonic.FrontCenterRight.f32_value > 0.8) {
				no_obstacle_counter++;
			}
			if(rec_ultrasonic.FrontRight.f32_value > 1.0) {
				no_obstacle_counter++;
			}
			if (rec_ultrasonic.FrontLeft.f32_value < 0.03 || rec_ultrasonic.FrontCenterLeft.f32_value < 0.03 || rec_ultrasonic.FrontCenter.f32_value < 0.03 || rec_ultrasonic.FrontCenterRight.f32_value < 0.03 || rec_ultrasonic.FrontRight.f32_value < 0.03) {
				error_counter+=5;
				if(m_bDebugModeEnabled) {
					LOG_WARNING(cString::Format("Ultrasonic Check: error in front sensor(s); values: %f, %f, %f, %f, %f", rec_ultrasonic.FrontLeft.f32_value, rec_ultrasonic.FrontCenterLeft.f32_value, rec_ultrasonic.FrontCenter.f32_value, rec_ultrasonic.FrontCenterRight.f32_value, rec_ultrasonic.FrontRight.f32_value));
				}
			}
			break;
		default:
			SetRunningState(tFalse);
			LOG_ERROR(cString::Format("Ultrasonic Check: Error occurred in ProcessUltrasonicInput(), command id (%d) not known", command));
	}

	// increase counter
	sample_counter++;

	// evaluation
	if( (sample_counter >= number_of_samples && command != AC_UC_CHECK_OVERTAKING_OBSTACLE) || ( sample_counter >= number_of_samples_overtaking && command == AC_UC_CHECK_OVERTAKING_OBSTACLE) ) {
		if(command == AC_UC_CHECK_PARKING_TRANS_FRONT) sample_counter *= 2;
		else if(command == AC_UC_CHECK_PULL_OUT_LONG) sample_counter *= 3;
		else if (command == AC_UC_CHECK_PULL_OUT_TRANS) sample_counter *= 5;

		// calculate number of free samples (relative to all received samples)
		tFloat32 free_samples = no_obstacle_counter/sample_counter;
		tFloat32 error_samples = error_counter/sample_counter;

		// create feedback
		TFeedbackStruct::Data feedback;
		feedback.ui32_filterID = F_ULTRASONIC_CHECK;

		// check error count
                if(error_samples > errorRateThreshold) {
			SetRunningState(tFalse);
			error_counter = 0;
			sample_counter = 0;
			feedback.ui32_status = FB_UC_ERROR_CODE; // error
			tFeedbackStruct.Transmit(&feedbackOutput, feedback, _clock->GetStreamTime());
			LOG_ERROR(cString::Format("Ultrasonic Check: error in mode %d, error rate too high: %d out of %d samples, rate: %f", command, error_counter, sample_counter, error_samples));
			RETURN_ERROR(ERR_BAD_DEVICE);
		}

		// check free samples: no obstacle -> free
		if(free_samples > no_obstacle_threshold) {
			feedback.ui32_status = FB_UC_NO_OBSTACLE;
			tFeedbackStruct.Transmit(&feedbackOutput, feedback, _clock->GetStreamTime());
			SetRunningState(tFalse);
			if(m_bDebugModeEnabled) {
				LOG_WARNING(cString::Format("Ultrasonic Check: Checked command %d; no obstacle counter: %d, samples: %d, rate: %f => free", command, no_obstacle_counter, sample_counter, free_samples));
			}
		// obstacle
		} else {
			// parking mode: send feedback obstacle
			if( command == AC_UC_CHECK_PARKING_SPACE_LONG || command == AC_UC_CHECK_PARKING_SPACE_TRANS )  {
				feedback.ui32_status = FB_UC_OBSTACLE;
				tFeedbackStruct.Transmit(&feedbackOutput, feedback, _clock->GetStreamTime());
				SetRunningState(tFalse);
				if(m_bDebugModeEnabled) {
					LOG_WARNING(cString::Format("Ultrasonic Check: Checked command %d; no obstacle counter: %d, samples: %d, rate: %f => occupied", command, no_obstacle_counter, sample_counter, free_samples));
				}
			// restart cycle (wait until obstacle disappears)
			} else {
				if(m_bDebugModeEnabled) {
					LOG_WARNING(cString::Format("Ultrasonic Check [Restart]: Checked command %d; no obstacle counter: %d, samples: %d, rate: %f => occupied", command, no_obstacle_counter, sample_counter, free_samples));
				}
				sample_counter = 0;
				no_obstacle_counter = 0;
			}
		}
	}

	RETURN_NOERROR;
}

tResult UltrasonicCheck::ProcessActionInput(IMediaSample* pMediaSample){
	__synchronized_obj(m_oCriticalSectionActionAccess);

	TActionStruct::ActionSub actionSub;
	actionSub = tActionStruct.Read_Action(pMediaSample, F_ULTRASONIC_CHECK);

	// set running state
	if(!GetRunningState() && actionSub.enabled && actionSub.started) {
		sample_counter = 0; // reset sample counter
		no_obstacle_counter = 0; // count samples without obstacles
		error_counter = 0; // count samples with US distance < 0.03
		SetRunningState(tTrue);
		command = actionSub.command;
		RETURN_NOERROR;
	}

	// send disable information when in running mode
	if (!actionSub.enabled && GetRunningState()) {
		SetRunningState(tFalse);
	}

	RETURN_NOERROR;
}

tBool UltrasonicCheck::GetRunningState(){
	__synchronized_obj(m_oCriticalSectionRunningStateAccess);
	return running;
}

tResult UltrasonicCheck::SetRunningState(tBool state){
	__synchronized_obj(m_oCriticalSectionRunningStateAccess);
	running = state;
	RETURN_NOERROR;
}

tResult UltrasonicCheck::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
	__synchronized_obj(criticalSection_OnPinEvent);

	 // first check what kind of event it is
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
		RETURN_IF_POINTER_NULL(pMediaSample);
		RETURN_IF_POINTER_NULL(pSource);

		// by comparing it to our member pin variable we can find out which pin received
		if (pSource == &actionInput) {
			// process action input
			ProcessActionInput(pMediaSample);
		} else if (pSource == &ultrasonicInput) {
			// process ultrasonic input
			if(GetRunningState()){
				ProcessUltrasonicInput(pMediaSample);
			}
		}
	}

	RETURN_NOERROR;
}
