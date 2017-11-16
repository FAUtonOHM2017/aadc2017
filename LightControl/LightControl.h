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

#ifndef _LIGHT_CONTROL_H_
#define _LIGHT_CONTROL_H_

#define OID_ADTF_LIGHT_CONTROL "adtf.user.light_control"

#include "TActionStruct.h"
#include "TFeedbackStruct.h"
#include "TSignalValue.h"
#include "TBoolSignalValue.h"
#include "ScmCommunication.h"


//*************************************************************************************************
class LightControl : public adtf::cFilter
{
	ADTF_DECLARE_FILTER_VERSION(OID_ADTF_LIGHT_CONTROL, "Light Control", OBJCAT_DataFilter, "Light Control", 1, 0, 0, "FAUtonOHM");

protected:

	cInputPin		actionInput;
	cInputPin		carspeedInput;
	cInputPin		setspeedInput;

	cOutputPin		feedbackOutput;
	cOutputPin		headLightOutput;
	cOutputPin		reverseLightOutput;
	cOutputPin		brakeLightOutput;
	cOutputPin		turnRightLightOutput;
	cOutputPin		turnLeftLightOutput;
	cOutputPin		hazardLightOutput;

	TFeedbackStruct	tFeedbackStruct;
	TActionStruct	tActionStruct;
	TSignalValue	CarSpeed;
	TSignalValue	SetSpeed;

	TBoolSignalValue HeadLight;
	TBoolSignalValue ReverseLight;
	TBoolSignalValue BrakeLight;
	TBoolSignalValue TurnRightLight;
	TBoolSignalValue TurnLeftLight;
	TBoolSignalValue HazardLight;

public:
	LightControl(const tChar* __info);
	virtual ~LightControl();

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
	tFloat32 factorEnableBrake;
	tFloat32 factorDisableBrake;
	tUInt32 averageSampleCount;

	/*! creates all the input Pins*/
	tResult CreateInputPins(__exception = NULL);
	/*! creates all the output Pins*/
	tResult CreateOutputPins(__exception = NULL);

	// process state machine action input
	tResult ProcessAction(IMediaSample* mediaSample);

	// process car speed input
	tResult ProcessSpeedInput(IMediaSample* mediaSample);

	// process set car speed input
	tResult ProcessSetSpeedInput(IMediaSample* mediaSample);

	// save car speed values
	std::list<tFloat32> last_carspeeds;

	// save brake light status
	tBool brakelightEnabled;

	// save reverse light status
	tBool reverselightEnabled;

	// critical section pin event
	cCriticalSection	criticalSection_OnPinEvent;

};

//*************************************************************************************************
#endif // _LIGHT_CONTROL_H_
