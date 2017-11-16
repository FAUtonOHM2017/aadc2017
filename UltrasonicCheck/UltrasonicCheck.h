/**
 * Copyright (c)
Audi Autonomous Driving Cup. TEAM FAUtonOHM.  All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

ADTF Filter for checking ultrasonic sensors on state machine request
**********************************************************************
* $Author::  $ fink  $Date:: 2016-02-09 00:00:00#$ $Rev:: 1.0.0   $
**********************************************************************/

#ifndef _ULTRASONIC_CHECK_H_
#define _ULTRASONIC_CHECK_H_

#define OID_ADTF_ULTRASONIC_CHECK "adtf.user.ultrasonic_check"

#include "TActionStruct.h"
#include "TFeedbackStruct.h"
#include "TSignalValue.h"
#include "TUltrasonicStruct.h"
#include "ScmCommunication.h"


//*************************************************************************************************
class UltrasonicCheck : public adtf::cFilter
{
	ADTF_DECLARE_FILTER_VERSION(OID_ADTF_ULTRASONIC_CHECK, "Ultrasonic Check", OBJCAT_DataFilter, "Ultrasonic Check", 1, 0, 0, "FAUtonOHM");

protected:

	cInputPin		actionInput;
	cInputPin		ultrasonicInput;

	cOutputPin		feedbackOutput;

	TFeedbackStruct	tFeedbackStruct;
	TActionStruct	tActionStruct;
	TUltrasonicStruct	UltrasonicStruct;

public:
	UltrasonicCheck(const tChar* __info);
	virtual ~UltrasonicCheck();

private: //private methods


protected:
	tResult Init(tInitStage eStage, __exception);
	tResult Start(__exception = NULL);
	tResult Stop(__exception = NULL);
	tResult Shutdown(tInitStage eStage, __exception);

	// implements IPinEventSink
	tResult OnPinEvent(IPin* pSource,
					   tInt nEventCode,
					   tInt nParam1,
					   tInt nParam2,
					   IMediaSample* pMediaSample);
private:

	// filter properties
	tBool m_bDebugModeEnabled;
	tUInt32 number_of_samples; // number of samples used for obstacle detection (US: 20 samples/s)
	tUInt32 number_of_samples_overtaking;
	tFloat32 no_obstacle_threshold; // samples (in percent) that have to be clear
	tFloat32 parking_space_long_threshold;
	tFloat32 parking_space_trans_threshold;
	tFloat32 giveway_threshold;
	tFloat32 overtaking_threshold;
        tFloat32 errorRateThreshold;

	/*! creates all the input Pins*/
	tResult CreateInputPins(__exception = NULL);
	/*! creates all the output Pins*/
	tResult CreateOutputPins(__exception = NULL);

	// Running State
	tBool GetRunningState();
	tResult SetRunningState(tBool);

	// process state machine action input
	tResult ProcessActionInput(IMediaSample* mediaSample);

	// process ultrasonic input
	tResult ProcessUltrasonicInput(IMediaSample* mediaSample);

	// flags and values
	tBool running; // running state
	tUInt32 command; // received command
	tUInt32 sample_counter; // count received ultrasonic samples
	tUInt32 no_obstacle_counter; // count samples without obstacles
	tUInt32 error_counter; // count samples with US distance < 0.03

	// critical section pin event
	cCriticalSection	criticalSection_OnPinEvent;
	cCriticalSection	m_oCriticalSectionActionAccess;
	cCriticalSection	m_oCriticalSectionRunningStateAccess;
	cCriticalSection	m_oCriticalSectionUltrasonicInput;

};

//*************************************************************************************************
#endif // _ULTRASONIC_CHECK_H_
