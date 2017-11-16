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


#include "TimerFilter.h"



ADTF_FILTER_PLUGIN("Timer Filter", OID_ADTF_TIMER_FILTER, TimerFilter);

TimerFilter::TimerFilter(const tChar* __info):cAsyncDataTriggeredFilter(__info), m_bDebugModeEnabled(tFalse), m_handleTimer(NULL)
{
	SetPropertyBool("Debug Output to Console",tFalse);
	SetPropertyStr("Debug Output to Console" NSSUBPROP_DESCRIPTION, "If enabled additional debug information is printed to the console (Warning: decreases performance)");
	b_runningState = tFalse;
	b_runningState_noreset = tFalse;

}

TimerFilter::~TimerFilter()
{

}

tResult TimerFilter::CreateInputPins(__exception)
{
	RETURN_IF_FAILED(actionInput.Create("action", tActionStruct_object.GetMediaType(), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&actionInput));



	RETURN_NOERROR;
}

tResult TimerFilter::CreateOutputPins(__exception)
{

	// create output pin for statemachine
	RETURN_IF_FAILED(feedbackOutput.Create("feedback", tFeedbackStruct_object.GetMediaType(), NULL));
	RETURN_IF_FAILED(RegisterPin(&feedbackOutput));

	RETURN_NOERROR;
}


tResult TimerFilter::Init(tInitStage eStage, __exception)
{
	// never miss calling the parent implementation!!
	RETURN_IF_FAILED(cAsyncDataTriggeredFilter::Init(eStage, __exception_ptr))

	// in StageFirst you can create and register your static pins.
	if (eStage == StageFirst)
	{

		RETURN_IF_FAILED(tActionStruct_object.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(tFeedbackStruct_object.StageFirst(__exception_ptr));



		// create and register the input and output pin
		RETURN_IF_FAILED(CreateInputPins(__exception_ptr));
		RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));

	}
	else if (eStage == StageNormal)
	{
		// get Properties from user interface
		m_bDebugModeEnabled = GetPropertyBool("Debug Output to Console");
		tmp_actionsub_command_noReset = 0;

	}
	else if (eStage == StageGraphReady)
	{
		// get size of media samples that has to be assigned later
		RETURN_IF_FAILED(tActionStruct_object.StageGraphReady());
		RETURN_IF_FAILED(tFeedbackStruct_object.StageGraphReady());


	}

	RETURN_NOERROR;
	}


tResult TimerFilter::Start(__exception)
{
	return cAsyncDataTriggeredFilter::Start(__exception_ptr);
}

tResult TimerFilter::Stop(__exception)
{
	__synchronized_obj(m_oCriticalSectionTimerSetup);

	destroyTimer(__exception_ptr);

	return cAsyncDataTriggeredFilter::Stop(__exception_ptr);
}


tResult TimerFilter::Shutdown(tInitStage eStage, __exception)
{


	// call the base class implementation
	return cAsyncDataTriggeredFilter::Shutdown(eStage, __exception_ptr);
}

tResult TimerFilter::Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr/* =NULL */)
{
    if (nActivationCode == IRunnable::RUN_TIMER)
    {
    	RETURN_IF_FAILED(destroyTimer());

    	setRunningState(tFalse);
    	setRunningStateNoReset(tFalse);

    	/* Transmit feedback that timer has ended */
    	TransmitFeedback();
    	//if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("TimerFilter: timer handle function called at: %d",_clock->GetStreamTime()));


    }

    return cAsyncDataTriggeredFilter::Run(nActivationCode, pvUserData, szUserDataSize, __exception_ptr);
}

tResult TimerFilter::ProcessAction(TActionStruct::ActionSub actionSub) {

	/* Filter gets command to start the timer */
	if(actionSub.enabled && actionSub.started) {
		/* check command input (range 4000 to 4999) */
		if(actionSub.command >= AC_TF_WAIT_ONEH_MSEC && actionSub.command <= AC_TF_WAIT_TWENTY_SEC) {
			tFloat32 tmp_TimeStamp = (static_cast<tFloat32>(actionSub.command - 4000)) / 10.0f ;	// time for timer in s
			if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("TimerFilter: Trying to initiate Timer with %f seconds.",tmp_TimeStamp));
			/* Another noreset-timer is already running */
			if(getRunningStateNoReset()) {
				if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("TimerFilter: Another 'noReset'-timer is already existing, trying to destroy previous one."));
				RETURN_IF_FAILED(destroyTimer());
				RETURN_IF_FAILED(setRunningStateNoReset(tFalse));
				if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("TimerFilter: Previous 'noReset'-timer destroyed, creating new instance with %f seconds.",tmp_TimeStamp));
				/* Create Timer */
				RETURN_IF_FAILED(createTimer(tmp_TimeStamp));
				/* Set Running State */
				RETURN_IF_FAILED(setRunningState(tTrue));
				if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("TimerFilter: Timer with %f seconds created.",tmp_TimeStamp));
			}
			/* no timer is running yet */
			else if(!getRunningState() && !getRunningStateNoReset()){
				/* Create Timer */
				RETURN_IF_FAILED(createTimer(tmp_TimeStamp));
				// if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("TimerFilter: Timer with %f seconds created.",tmp_TimeStamp));
				/* Set Running State */
				RETURN_IF_FAILED(setRunningState(tTrue));
			}
			/* if timer was still enabled but new action command to create timer is sent, it is assumed that previous  timer is not necessary anymore;
			 * so new timer will be created */
			else if(getRunningState()){ // running mode is already in active mode
				if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("TimerFilter: Another timer is already existing, trying to destroy previous one."));
				RETURN_IF_FAILED(destroyTimer());
				if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("TimerFilter: Previous timer destroyed, creating new instance with %f seconds.",tmp_TimeStamp));
				/* Create Timer */
				RETURN_IF_FAILED(createTimer(tmp_TimeStamp));
				if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("TimerFilter: New Timer with %f seconds created.",tmp_TimeStamp));
			}
			else{ // noReset-timer is active
				 RETURN_AND_LOG_ERROR_STR(ERR_ACCESS_DENIED,cString::Format("TimerFilter: Error occured - 'noReset'-timer is already existing, not able to create 'normal' required one!"));
			}
		}
		else if(actionSub.command >= AC_TF_NORESET_WAIT_ONE_SEC && actionSub.command <= AC_TF_NORESET_WAIT_TWENTY_SEC){
			tFloat32 tmp_TimeStampNoreset = (static_cast<tFloat32>(actionSub.command - 4500)) / 10.0f ;	// time for timer in s
			/* Another normal timer without reset is already running */
			if(getRunningState()) {
				if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("TimerFilter: Another 'normal'-timer is already existing, trying to destroy previous one."));
				RETURN_IF_FAILED(destroyTimer());
				RETURN_IF_FAILED(setRunningState(tFalse));
				if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("TimerFilter: Previous 'normal'-timer destroyed, creating new instance with %f seconds.",tmp_TimeStampNoreset));
				/* Create Timer */
				RETURN_IF_FAILED(createTimer(tmp_TimeStampNoreset));
				/* Set Running State */
				RETURN_IF_FAILED(setRunningStateNoReset(tTrue));
				RETURN_IF_FAILED(SetCurActionSubCommandNoReset(actionSub.command));
				if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("TimerFilter: New 'noReset'-timer with %f seconds created.",tmp_TimeStampNoreset));
			}
			/* Timer has not yet been started */
			else if(!getRunningStateNoReset() && !getRunningState()){
				/* Create Timer */
				RETURN_IF_FAILED(createTimer(tmp_TimeStampNoreset));
				/* Set Running State */
				RETURN_IF_FAILED(setRunningStateNoReset(tTrue));
				RETURN_IF_FAILED(SetCurActionSubCommandNoReset(actionSub.command));
				if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("TimerFilter: New 'noReturn'-Timer with %f seconds created.",tmp_TimeStampNoreset));
			}
			/* if timer is still enabled but new action command to create timer is sent and the actionCommand is exactly the same as before,
			 * it is assumed that program is in a loop and timer should continue running (in a timeout-manner) */
			else if(getRunningStateNoReset() && (actionSub.command == GetCurActionSubCommandNoReset())){
				if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("TimerFilter: 'NoReturn'-Timer with %f seconds already running, ignoring new action command (loop-mode).",tmp_TimeStampNoreset));
			}
			/* if timer is still enabled but new action command to create timer is sent with a different command (different time), it is assumed
			 * that timer is unnecessary; previous one will be destroyed, new one will be created */
			else if(getRunningStateNoReset() && (actionSub.command != GetCurActionSubCommandNoReset())){
				if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("TimerFilter: Another 'noReset'-timer is already existing, trying to destroy previous one."));
				RETURN_IF_FAILED(destroyTimer());
				if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("TimerFilter: Previous 'noReset'-timer destroyed, creating new instance with %f seconds.",tmp_TimeStampNoreset));
				/* Create Timer */
				RETURN_IF_FAILED(createTimer(tmp_TimeStampNoreset));
				RETURN_IF_FAILED(SetCurActionSubCommandNoReset(actionSub.command));
				if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("TimerFilter: New 'noReset'-timer with %f seconds created.",tmp_TimeStampNoreset));
			}
			else{ // normal timer active
				RETURN_AND_LOG_ERROR_STR(ERR_ACCESS_DENIED,cString::Format("TimerFilter: Error occured - 'normal'-timer is already existing, not able to create 'noReset' required one!"));
			}
		/* if received command is not in predefined numberspace for timer filter */
		} else {
			RETURN_AND_LOG_ERROR_STR(ERR_INVALID_STATE, cString::Format("TimerFilter: received invalid command %d", actionSub.command));
		}
	}else{
		/* if timer was still enabled but new action command is sent, it is assumed that timer is not necessary anymore */
		if(getRunningState() || getRunningStateNoReset()){
			if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("TimerFilter: timer was running, not necessary anymore -> will be destroyed."));
			RETURN_IF_FAILED(destroyTimer());
			setRunningState(tFalse);
			setRunningStateNoReset(tFalse);
			SetCurActionSubCommandNoReset(0);
		}

	}

	RETURN_NOERROR;
}

tResult TimerFilter::TransmitFeedback(){
	__synchronized_obj(m_oCriticalSectionTransmitFeedback);
	RETURN_IF_FAILED(tFeedbackStruct_object.Transmit(&feedbackOutput, feedback, _clock->GetStreamTime()));

	RETURN_NOERROR;
}

tResult TimerFilter::SetFeedbackStatus(){
	__synchronized_obj(m_oCriticalSectionAccessFeedback);
	/* Set status-variable to finished for upcoming transmission-process of feedback;
	 *  only called if timer was successfully created */
	feedback.ui32_filterID = F_TIMER;
	feedback.ui32_status = FB_TF_FINISHED;
	RETURN_NOERROR;
}

tBool TimerFilter::getRunningState(){
	__synchronized_obj(m_oCriticalSectionRunningsState);
	return b_runningState;
}

tResult TimerFilter::setRunningState(tBool state){
	__synchronized_obj(m_oCriticalSectionRunningsState);
	b_runningState = state;
	RETURN_NOERROR;
}

tUInt32 TimerFilter::GetCurActionSubCommandNoReset(){
	__synchronized_obj(m_oCriticalSectionActionSubNoResetAccess);
	return tmp_actionsub_command_noReset;
}

tResult TimerFilter::SetCurActionSubCommandNoReset(tUInt32 actionSubCommandNoReset){
	__synchronized_obj(m_oCriticalSectionActionSubNoResetAccess);
	tmp_actionsub_command_noReset = actionSubCommandNoReset;
	RETURN_NOERROR;
}

tBool TimerFilter::getRunningStateNoReset(){
	__synchronized_obj(m_oCriticalSectionRunningsStateNoReset);
	return b_runningState_noreset;
}

tResult TimerFilter::setRunningStateNoReset(tBool state_noreset){
	__synchronized_obj(m_oCriticalSectionRunningsStateNoReset);
	b_runningState_noreset = state_noreset;
	RETURN_NOERROR;
}

tResult TimerFilter::OnAsyncPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{
	 // first check what kind of event it is
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived)
	{
		RETURN_IF_POINTER_NULL(pMediaSample);
		RETURN_IF_POINTER_NULL(pSource);

		// check which pin mediasample was received on
		if (pSource == &actionInput) {

			TActionStruct::ActionSub actionSub;
			actionSub = tActionStruct_object.Read_Action(pMediaSample, F_TIMER);

			ProcessAction(actionSub);
		}
	}

	RETURN_NOERROR;
}


/* Method for creating a timer with name m_handleTimer with a variable time 'timeStamp' in seconds */
tResult TimerFilter::createTimer(tFloat32 timeStamp)
{
     __synchronized_obj(m_oCriticalSectionTimerSetup);
     // additional check necessary because theoretically, more than one request to start a time could be possible (only one will be started!)
     if (m_handleTimer == NULL)
     {
    	 m_handleTimer = _kernel->TimerCreate(0, tTimeStamp(timeStamp*1000000.0f), static_cast<IRunnable*>(this),
                                        NULL, NULL, 0, 0, adtf_util::cString::Format("%s.timer", OIGetInstanceName()));

    	 /* Set feedback status to finished; will be transmitted after timer has ended */
    	 SetFeedbackStatus();

    	 if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("TimerFilter: Timer with %f seconds created successfully at time: %d",timeStamp, _clock->GetStreamTime()));
     }
     else
     {
        LOG_ERROR("TimerFilter: Timer is already running. Unable to create a new one. Previous timer must first be destroyed.");
     }
     RETURN_NOERROR;
}

/* Method for destroying the timer m_hTimer*/
tResult TimerFilter::destroyTimer(__exception)
{
    __synchronized_obj(m_oCriticalSectionTimerSetup);
    //destroy timer
    if (m_handleTimer != NULL)
    {
        tResult nResult = _kernel->TimerDestroy(m_handleTimer);
        if (IS_FAILED(nResult))
        {
            LOG_ERROR("TimerFilter: Unable to destroy the timer.");
            THROW_ERROR(nResult);
        }
        m_handleTimer = NULL;
    }
    //check if handle for some unknown reason still exists
    else
    {
    	if(m_bDebugModeEnabled) LOG_WARNING("TimerFilter: Timer handle not set, but I should destroy the timer. Try to find a timer with my name.");
        tHandle hFoundHandle = _kernel->FindHandle(adtf_util::cString::Format("%s.timer", OIGetInstanceName()));
        if (hFoundHandle)
        {
            tResult nResult = _kernel->TimerDestroy(hFoundHandle);
            if (IS_FAILED(nResult))
            {
                LOG_ERROR("TimerFilter: Unable to destroy the found timer.");
                THROW_ERROR(nResult);
            }
        }
    }
    if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("TimerFilter: Timer destroyed at time: %d", _clock->GetStreamTime()));
    RETURN_NOERROR;
}
