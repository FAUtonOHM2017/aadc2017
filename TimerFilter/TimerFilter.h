/**
 * Copyright (c)
Audi Autonomous Driving Cup. TEAM FAUtonOHM.  All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: �This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS �AS IS� AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Filter implements a wait timer which calls a function after the time is over
**********************************************************************
* $Author:: mahill $   $Date:: 2016-01-28 19:49:07#$ $Rev:: 1.0.0   $
**********************************************************************/


#ifndef _TIMER_FILTER_H_
#define _TIMER_FILTER_H_

#define OID_ADTF_TIMER_FILTER "adtf.user.timer_filter"

#include "stdafx.h"


//*************************************************************************************************
class TimerFilter : public adtf::cAsyncDataTriggeredFilter
{
	ADTF_DECLARE_FILTER_VERSION(OID_ADTF_TIMER_FILTER, "Timer Filter", OBJCAT_DataFilter, "Timer Filter", 1, 0, 1, "FAUtonOHM");

protected:

	cInputPin		actionInput;
	cOutputPin		feedbackOutput;

	TFeedbackStruct	tFeedbackStruct_object;
	TActionStruct	tActionStruct_object;


	tBool m_bDebugModeEnabled;

public:
	TimerFilter(const tChar* __info);
	virtual ~TimerFilter();

private: //private methods


protected:
	tResult Init(tInitStage eStage, __exception);
	tResult Start(__exception = NULL);
	tResult Stop(__exception = NULL);
	tResult Shutdown(tInitStage eStage, __exception);

	/*! overrides cFilter */
	tResult Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr=NULL);


	// implements IPinEventSink
	tResult OnAsyncPinEvent(IPin* pSource,
					   tInt nEventCode,
					   tInt nParam1,
					   tInt nParam2,
					   IMediaSample* pMediaSample);

private:

	/*! creates all the input Pins*/
	tResult CreateInputPins(__exception = NULL);
	/*! creates all the output Pins*/
	tResult CreateOutputPins(__exception = NULL);

	/*! creates the timer for the cyclic transmits*/
	tResult createTimer(tFloat32  timeStamp);

   /*! destroys the timer for the cyclic transmits*/
	tResult destroyTimer(__exception = NULL);

	/* returns/sets current running state, true if running, false if disabled */
	tBool getRunningState();
	tResult setRunningState(tBool state);

	/* process action struct input from SCm*/
	tResult ProcessAction(TActionStruct::ActionSub actionSub);

	/* transmit the feedback to SCM*/
	tResult TransmitFeedback();

	/* sets the status of the feedback to be returned */
	tResult SetFeedbackStatus();

	/* sets last received action sub command in no-reset mode */
	tUInt32 GetCurActionSubCommandNoReset();

	/* sets last received action sub command in no-reset mode */
	tResult SetCurActionSubCommandNoReset(tUInt32 actionSubNoResetcommand);
	/* running state for timer-mode without reset */
	tBool b_runningState_noreset;
	tBool getRunningStateNoReset();
	tResult setRunningStateNoReset(tBool state_noreset);

	/* running state for timer-mode with reset */
	tBool b_runningState;

	/*! handle for the timer */
	tHandle m_handleTimer;

	/* variable to save feedback before transmitting*/
	TFeedbackStruct::Data feedback;

	tUInt32 tmp_actionsub_command_noReset;

	// critical sections
	cCriticalSection	m_oCriticalSectionTimerSetup;
	cCriticalSection	m_oCriticalSectionTransmitFeedback;
	cCriticalSection	m_oCriticalSectionAccessFeedback;
	cCriticalSection	m_oCriticalSectionRunningsState;
	cCriticalSection 	m_oCriticalSectionActionSubNoResetAccess;
	cCriticalSection	m_oCriticalSectionRunningsStateNoReset;

};

//*************************************************************************************************
#endif // _TIMER_FILTER_H_
