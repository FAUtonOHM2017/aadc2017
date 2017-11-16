/**
 * Copyright (c)
Audi Autonomous Driving Cup. All rights reserved. Team FAUtonOHM

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: �This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS �AS IS� AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

New Filter managing the car of team FAUtonOHM
**********************************************************************
* $Author:: mahill $   $Date:: 2016-02-02 21:15:07#$ $Rev:: 0.2.1   $
**********************************************************************/

#ifndef _ACTION_STOP_FILTER_H_
#define _ACTION_STOP_FILTER_H_

#define OID_ADTF_ACTION_STOP_FILTER "adtf.user.action_stop_filter"

#include "stdafx.h"


//*************************************************************************************************
class ActionStop: public adtf::cAsyncDataTriggeredFilter {
	ADTF_DECLARE_FILTER_VERSION(OID_ADTF_ACTION_STOP_FILTER, "Action Stop", OBJCAT_DataFilter, "Action Stop", 1, 0, 0, "FAUtonOHM");

protected:
	cInputPin measured_speed_input;
	cInputPin actionStruct_input;
	cOutputPin feedbackStruct_output;
	cOutputPin targetSpeed_output;

public:
	ActionStop(const tChar* __info);
	virtual ~ActionStop();

protected:
	tResult Init(tInitStage eStage, __exception);
	tResult Start(__exception = NULL);
	tResult Stop(__exception = NULL);
	tResult Shutdown(tInitStage eStage, __exception);

	// implements IPinEventSink
	tResult OnAsyncPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
private:


	tFloat32 m_f32PercentageActiveBreaking;
	tFloat32 m_f32SpeedStoppedThreshold;
	tBool m_bDebugModeEnabled;

	tBool m_bCommandActivated;

    /* Object of type TActionStruct */
    TActionStruct TActionStruct_object;

    /* Object of type TFeedbackStruct */
    TFeedbackStruct TFeedbackStruct_object;

    /* Objects of type TSignalValue, one for input and one for output */
    TSignalValue TSignalValue_Input_object;
    TSignalValue TSignalValue_Output_object;

	/*! creates all the input Pins*/
	tResult CreateInputPins(__exception = NULL);
	/*! creates all the output Pins*/
	tResult CreateOutputPins(__exception = NULL);


	/* functions for processing and transmitting */
	tResult ProcessActionData(TActionStruct::ActionSub actionSub);
	tResult ProcessSpeedInput(TSignalValue::Data measuredSpeed);
	tResult TransmitFeedback();
	tResult TransmitTargetSpeed(TSignalValue::Data targetSpeed);

	/* set flag that actionStop was activated */
	tResult setActivatedFlag(tBool activated);
	tBool getActivatedFlag();


	/*! the critical section of accessing the activation-flag */
	cCriticalSection m_oCriticalSectionActivationFlag;
	/*! the critical section of transmitting the feedbackStruct*/
	cCriticalSection m_oCriticalSectionTransmitFeedbackStruct;
	/*! the critical section of transmitting the targetSpeed*/
	cCriticalSection m_oCriticalSectionTransmitTargetSpeed;

};

//*************************************************************************************************
#endif // _ACTION_STOP_FILTER_H_
