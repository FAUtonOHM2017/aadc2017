/**
 * Copyright (c)
Audi Autonomous Driving Cup. All rights reserved. Team FAUtonOHM.

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

#include <math.h>
#include "ActionStop.h"


ADTF_FILTER_PLUGIN("Action Stop", OID_ADTF_ACTION_STOP_FILTER, ActionStop);

ActionStop::ActionStop(const tChar* __info) :cAsyncDataTriggeredFilter(__info){

	m_bDebugModeEnabled = tFalse;
	// percentage that should be used for active breaking
	// emerg_output_speed = (-1)*(actualSpeed* PercentageActiveBreaking)
	m_f32PercentageActiveBreaking = 0;

	/* speed in m/s that is seen as 'car stopped'*/
	m_f32SpeedStoppedThreshold = 0.05;
	/* Flag for expressing if actionStop-command was received */
	m_bCommandActivated = tFalse;

	/* Percentage that should be used for active breaking */
	//SetPropertyFloat("Percentage for active breaking", m_f32PercentageActiveBreaking);
	//SetPropertyFloat("Percentage for active breaking" NSSUBPROP_REQUIRED, tTrue);
	//SetPropertyStr("Percentage for active breaking" NSSUBPROP_DESCRIPTION,
		//	"Percentage of actual speed that is applied for a active breaking; '0' means disabled, '1' full means inverse speed is applied; Maximum value is '5'.");

	/* Speed in m/s that is seen as 'car stopped' */
	SetPropertyFloat("Remaining speed of car", m_f32SpeedStoppedThreshold);
	SetPropertyFloat("Remaining speed of car" NSSUBPROP_REQUIRED, tTrue);
	SetPropertyStr("Remaining speed of car" NSSUBPROP_DESCRIPTION,
			"Speed in m/s that is seen as 'car stopped', that means values smaller than this threshold are seen as speed '0'.");

	/* Debug flag activation debug mode and console output */
	SetPropertyBool("Enable Debug", m_bDebugModeEnabled);
	SetPropertyBool("Enable Debug" NSSUBPROP_REQUIRED, tFalse);
	SetPropertyStr("Enable Debug" NSSUBPROP_DESCRIPTION,"Enables printing debug messages to console");
}


ActionStop::~ActionStop() {
}

tResult ActionStop::CreateInputPins(__exception)
{

	RETURN_IF_FAILED(measured_speed_input.Create("Measured_speed", TSignalValue_Input_object.GetMediaType(), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&measured_speed_input));

	RETURN_IF_FAILED(actionStruct_input.Create("ActionStruct", TActionStruct_object.GetMediaType(), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&actionStruct_input));

	RETURN_NOERROR;
}

tResult ActionStop::CreateOutputPins(__exception)
{
	RETURN_IF_FAILED(targetSpeed_output.Create("Target_speed", TSignalValue_Output_object.GetMediaType(), NULL));
	RETURN_IF_FAILED(RegisterPin(&targetSpeed_output));

	RETURN_IF_FAILED(feedbackStruct_output.Create("FeedbackStruct", TFeedbackStruct_object.GetMediaType(), NULL));
	RETURN_IF_FAILED(RegisterPin(&feedbackStruct_output));

	RETURN_NOERROR;
}

tResult ActionStop::Init(tInitStage eStage, __exception)
{
	// never miss calling the parent implementation!!
	RETURN_IF_FAILED(cAsyncDataTriggeredFilter::Init(eStage, __exception_ptr))

	// in StageFirst you can create and register your static pins.
	if (eStage == StageFirst)
	{
		/* StageFirst actions for inputs */
		RETURN_IF_FAILED(TSignalValue_Input_object.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(TActionStruct_object.StageFirst(__exception_ptr));
		/* StageFirst actions for outputs */
		RETURN_IF_FAILED(TSignalValue_Output_object.StageFirst(__exception_ptr));
		RETURN_IF_FAILED(TFeedbackStruct_object.StageFirst(__exception_ptr));

		// create and register the input pin
		RETURN_IF_FAILED(CreateInputPins(__exception_ptr));
		RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));

	}
	else if (eStage == StageNormal)
	{
		/* get debug mode */
		m_bDebugModeEnabled = tBool(GetPropertyBool("Enable Debug"));

		/* get threshold for car speed, seen as zero */
		m_f32SpeedStoppedThreshold = (tFloat32) (GetPropertyFloat("Remaining speed of car"));

		/* maximum breakforce is set to 5time the measured speed*/
		//m_f32PercentageActiveBreaking = tFloat32(GetPropertyFloat("Percentage for active breaking"));
	//	if(m_f32PercentageActiveBreaking < 0){
	//		m_f32PercentageActiveBreaking = 0;
	//	}
	//	else if(m_f32PercentageActiveBreaking > 5){
	//		m_f32PercentageActiveBreaking = 5;
	//	}

	}
	else if (eStage == StageGraphReady)
	{
		/* StageReady actions for inputs */
		RETURN_IF_FAILED(TSignalValue_Input_object.StageGraphReady());
		RETURN_IF_FAILED(TActionStruct_object.StageGraphReady());

		/* StageReady actions for outputs */
		RETURN_IF_FAILED(TSignalValue_Output_object.StageGraphReady());
		RETURN_IF_FAILED(TFeedbackStruct_object.StageGraphReady());
	}

	RETURN_NOERROR;
}

tResult ActionStop::Start(__exception)
{
	return cAsyncDataTriggeredFilter::Start(__exception_ptr);
}

tResult ActionStop::Stop(__exception)
{
	return cAsyncDataTriggeredFilter::Stop(__exception_ptr);
}

tResult ActionStop::Shutdown(tInitStage eStage, __exception)
{


	// call the base class implementation
	return cAsyncDataTriggeredFilter::Shutdown(eStage, __exception_ptr);
}



tResult ActionStop::OnAsyncPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample) {

	// so we received a media sample, so this pointer better be valid.
	RETURN_IF_POINTER_NULL(pMediaSample);
	RETURN_IF_POINTER_NULL(pSource);


	// first check what kind of event it is (and if output exists)
	if (nEventCode == IPinEventSink::PE_MediaSampleReceived) {


		// by comparing it to our member pin variable we can find out which pin received
		if (pSource == &actionStruct_input) {

			TActionStruct::ActionSub  actionStruct_tmp;
			actionStruct_tmp = TActionStruct_object.Read_Action(pMediaSample,F_STOP_ACTION);
			RETURN_IF_FAILED(ProcessActionData(actionStruct_tmp));
		}
		else if (pSource == &measured_speed_input){
			if (getActivatedFlag()){
				if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("ActionStopFilter: ActiveStop Activated, Stopping."));
				TSignalValue::Data measuredSpeed;
				RETURN_IF_FAILED(TSignalValue_Input_object.Read(pMediaSample,&measuredSpeed));
				if(fabsf(measuredSpeed.f32_value) <= m_f32SpeedStoppedThreshold){
					setActivatedFlag(tFalse);
					RETURN_IF_FAILED(ProcessSpeedInput(measuredSpeed));
					RETURN_IF_FAILED(TransmitFeedback());
				}
				else{
					RETURN_IF_FAILED(ProcessSpeedInput(measuredSpeed));
				}
			}

		}
	}

	RETURN_NOERROR;
}

/* Method that is called for processing the input of ActionStruct */
tResult ActionStop::ProcessActionData(TActionStruct::ActionSub actionSub)
{
	/* set flag "enabled", expressing that actionStop input was received, that means breaking has to be activated */
	if(actionSub.command == AC_SA_STOP_CAR){
		/* set flag */
		setActivatedFlag(tTrue);
		TSignalValue::Data targetSpeed;
		targetSpeed.f32_value = 0.0f;
		targetSpeed.ui32_arduinoTimestamp = _clock->GetStreamTime();
		TransmitTargetSpeed(targetSpeed);
		if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("ActionStopFilter: Received actionSTOP. Sent first sample with speed zero. "));
	}
	RETURN_NOERROR;
}

tResult ActionStop::ProcessSpeedInput(TSignalValue::Data measuredSpeed){
	/* calculate break force for active breaking */
	TSignalValue::Data targetSpeed;
	targetSpeed.f32_value = -(measuredSpeed.f32_value * m_f32PercentageActiveBreaking);
	targetSpeed.ui32_arduinoTimestamp = measuredSpeed.ui32_arduinoTimestamp;
	RETURN_IF_FAILED(TransmitTargetSpeed(targetSpeed));
	RETURN_NOERROR;
}


tResult ActionStop::setActivatedFlag(tBool activated){
	__synchronized_obj(m_oCriticalSectionActivationFlag);
	m_bCommandActivated = activated;
	RETURN_NOERROR;
}
tBool ActionStop::getActivatedFlag(){
	__synchronized_obj(m_oCriticalSectionActivationFlag);
	return m_bCommandActivated;
}

tResult ActionStop::TransmitTargetSpeed(TSignalValue::Data targetSpeed){
	__synchronized_obj(m_oCriticalSectionTransmitTargetSpeed);
	RETURN_IF_FAILED(TSignalValue_Output_object.Transmit(&targetSpeed_output,targetSpeed,_clock->GetStreamTime()));
	RETURN_NOERROR;
}

tResult ActionStop::TransmitFeedback(){
	__synchronized_obj(m_oCriticalSectionTransmitFeedbackStruct);
	TFeedbackStruct::Data feedback_tmp;
	feedback_tmp.ui32_filterID = F_STOP_ACTION;
	feedback_tmp.ui32_status = FB_SA_STOPPED;
	tTimeStamp cur_time = _clock->GetStreamTime();
	RETURN_IF_FAILED(TFeedbackStruct_object.Transmit(&feedbackStruct_output,feedback_tmp,cur_time));
	if(m_bDebugModeEnabled) {
		LOG_WARNING(cString::Format("ActionStopFilter: FeedbackStruct with ID %d and status %d successfully transmitted to SCM.",
				feedback_tmp.ui32_filterID,feedback_tmp.ui32_status));
	}
	RETURN_NOERROR;
}




