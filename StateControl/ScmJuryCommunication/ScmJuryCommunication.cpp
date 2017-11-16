/**
 * THIS FILTER IS BASED ON A VERSION FROM:
Copyright (c) 
Audi Autonomous Driving Cup. All rights reserved. Team FAUtonOHM.
 
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: �This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS �AS IS� AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**********************************************************************
* $Author:: spiesra $  $Date:: 2015-05-13 08:29:07#$ $Rev:: 35003   $
**********************************************************************
##########################################################################

New Filter communication between the car of team FAUtonOHM and jury
**********************************************************************
* $Author:: mahill $   $Date:: 2016-03-01 22:34:07#$ $Rev:: 1.0.1   $
**********************************************************************/


#include "ScmJuryCommunication.h"

#define DEBUG_TO_CONSOLE "Debug::Debug output to console"
#define DEBUG_PRINT_STRUCTURE "Debug::Print Structure to Console"
#define DEBUG_SENDSTATE_OUTPUT "Debug::Print SendState Outputs to Console (cyclic)"

ADTF_FILTER_PLUGIN("Scm Jury Communication", OID_ADTF_SCM_JURYCOMMUNICATION, ScmJuryCommunication);

#define SC_PROP_FILENAME "Maneuver File"


ScmJuryCommunication::ScmJuryCommunication(const tChar* __info) : cAsyncDataTriggeredFilter(__info), m_bDebugModeEnabled(tFalse), m_bDebugSendStateModeEnabled(tFalse),m_hTimer(NULL)
{
    SetPropertyBool(DEBUG_TO_CONSOLE,tFalse);
    SetPropertyStr(DEBUG_TO_CONSOLE NSSUBPROP_DESCRIPTION, "If enabled additional debug information is printed to the console (Warning: decreases performance)");

    SetPropertyBool(DEBUG_PRINT_STRUCTURE,tFalse);
    SetPropertyStr(DEBUG_PRINT_STRUCTURE NSSUBPROP_DESCRIPTION, "If enabled the structure defined by xml-file is printed to the console (Warning: decreases performance)");

    SetPropertyBool(DEBUG_SENDSTATE_OUTPUT,tFalse);
    SetPropertyStr(DEBUG_SENDSTATE_OUTPUT NSSUBPROP_DESCRIPTION, "If enabled the current state of the car is printed to the console in a cyclic manner (Warning: decreases performance)");

    /* Initialization of size to allocate for creating mediasample of type DriverStruct */
    nSize_DriverStruct = 0;
}

ScmJuryCommunication::~ScmJuryCommunication()
{
}

tResult ScmJuryCommunication::CreateInputPins(__exception)
{
	//get the media description manager for this filter
	cObjectPtr<IMediaDescriptionManager> pDescManager;
	RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));

	/* Create input pin to receive jury structs from jury module */
	tChar const * strDesctJuryStruct = pDescManager->GetMediaDescription("tJuryStruct");
	RETURN_IF_POINTER_NULL(strDesctJuryStruct);
	cObjectPtr<IMediaType> pTypeJuryStruct = new cMediaType(0, 0, 0, "tJuryStruct", strDesctJuryStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
	RETURN_IF_FAILED(m_JuryStructInputPin.Create("Jury_Struct", pTypeJuryStruct, this));
	RETURN_IF_FAILED(RegisterPin(&m_JuryStructInputPin));
	RETURN_IF_FAILED(pTypeJuryStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionJuryStruct));


	/* Create input pin to receive maneuver list from jury module */
	tChar const * strDesctManeuverList = pDescManager->GetMediaDescription("tManeuverList");
	RETURN_IF_POINTER_NULL(strDesctManeuverList);
	cObjectPtr<IMediaType> pTypeManeuverList = new cMediaType(0, 0, 0, "tManeuverList", strDesctManeuverList, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
	RETURN_IF_FAILED(m_ManeuverListInputPin.Create("Maneuver_List", pTypeManeuverList, this));
	RETURN_IF_FAILED(RegisterPin(&m_ManeuverListInputPin));
	RETURN_IF_FAILED(pTypeManeuverList->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescManeuverList));

	/* Create Input Pin for ActionStruct to receive activation signals from StateControlManagement */
	RETURN_IF_FAILED(m_ActionStructInputPin.Create("Action_Struct", TActionStruct_object.GetMediaType(), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_ActionStructInputPin));

	RETURN_NOERROR;
}

tResult ScmJuryCommunication::CreateOutputPins(__exception)
{
	//get the media description manager for this filter
	cObjectPtr<IMediaDescriptionManager> pDescManager;
	RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));

	/* Create Output Pin providing DriverStruct to JuryModule*/
	tChar const * strDesctDriverStruct = pDescManager->GetMediaDescription("tDriverStruct");
	RETURN_IF_POINTER_NULL(strDesctDriverStruct);
	cObjectPtr<IMediaType> pTypeDriverStruct = new cMediaType(0, 0, 0, "tDriverStruct", strDesctDriverStruct, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
	RETURN_IF_FAILED(m_DriverStructOutputPin.Create("Driver_Struct", pTypeDriverStruct, this));
	RETURN_IF_FAILED(RegisterPin(&m_DriverStructOutputPin));
	RETURN_IF_FAILED(pTypeDriverStruct->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionDriverStruct));

	/* Create Output providing feedback to StateController */
	RETURN_IF_FAILED(m_FeedbackStructOutputPin.Create("Feedback_Struct",TFeedbackStruct_object.GetMediaType(), NULL));
	RETURN_IF_FAILED(RegisterPin(&m_FeedbackStructOutputPin));


	RETURN_NOERROR;
}


tResult ScmJuryCommunication::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cAsyncDataTriggeredFilter::Init(eStage, __exception_ptr));

    // pins need to be created at StageFirst
    if (eStage == StageFirst)    
    {
    	//LOG_WARNING("StateControlManagement: StageFirst;");

    	/* get Information regarding input and output pins for specific types */
    	RETURN_IF_FAILED(TActionStruct_object.StageFirst(__exception_ptr));
        RETURN_IF_FAILED(TFeedbackStruct_object.StageFirst(__exception_ptr));

    	// create and register the input pin
    	RETURN_IF_FAILED(CreateInputPins(__exception_ptr));
    	RETURN_IF_FAILED(CreateOutputPins(__exception_ptr));

        m_bDebugModeEnabled = GetPropertyBool(DEBUG_TO_CONSOLE);
        m_bDebugSendStateModeEnabled= GetPropertyBool(DEBUG_SENDSTATE_OUTPUT);


        /* Jury and Maneuver communication*/
        m_i16CurrentManeuverID = 0;
        m_startSignalreceived = tFalse;
        m_maneuverFirstTransmitrequested = tFalse;
        m_maneuverListLoaded = tFalse;
        m_startSignalrequested = tFalse;
        m_bCompletedJuryList = tFalse;
        m_CarState = stateCar_STARTUP;
        /* using the method provides sending the state to the jury if module is already connected */
        //changeState(stateCar_STARTUP);
        tmpPrevManeuverInt_fb = FB_JURY_PREVIOUS_NO_MAN_EXISTING;

    }
    else if (eStage == StageNormal)
    {
    	//LOG_WARNING("StateControlManagement: StageNormal;");
    	m_bPrintStrucModeEnabled = GetPropertyBool(DEBUG_PRINT_STRUCTURE);


    }
    else if(eStage == StageGraphReady)
    {
    	//LOG_WARNING("StateControlManagement: StageGraphReady;");
    	/* Jury and Maneuver communication */
        m_i16SectionListIndex = -1;
        m_i16ManeuverListIndex =-1;

        // IDs for ActionStruct and FeedbackStruct are set
        RETURN_IF_FAILED(TActionStruct_object.StageGraphReady());
        RETURN_IF_FAILED(TFeedbackStruct_object.StageGraphReady());

        // no ids were set yet
        m_bIDsDriverStructSet = tFalse;
        m_bIDsJuryStructSet = tFalse;

        cObjectPtr<IMediaSerializer> pSerializer;
        m_pDescriptionDriverStruct->GetMediaSampleSerializer(&pSerializer);
        nSize_DriverStruct = pSerializer->GetDeserializedSize();

    }
    RETURN_NOERROR;
}

tResult ScmJuryCommunication::Start(__exception)
{
    return cAsyncDataTriggeredFilter::Start(__exception_ptr);
}

tResult ScmJuryCommunication::Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr/* =NULL */)
{
    if (nActivationCode == IRunnable::RUN_TIMER)
    {
    	stateCar tmp_state = getCarState();
    	tInt16 tmp_ManID = getCurManeuverID();
    	if(m_bDebugSendStateModeEnabled) LOG_WARNING(cString::Format("JCom: TIMER HANDLE CALLED! state is %d, ManID is %d",tmp_state,tmp_ManID));
        SendState(tmp_state,tmp_ManID);
    }   
    
    return cAsyncDataTriggeredFilter::Run(nActivationCode, pvUserData, szUserDataSize, __exception_ptr);
}

tResult ScmJuryCommunication::Stop(__exception)
{
    __synchronized_obj(m_oCriticalSectionTimerSetup);
    
    destroyTimer(__exception_ptr);

    return cAsyncDataTriggeredFilter::Stop(__exception_ptr);
}

tResult ScmJuryCommunication::Shutdown(tInitStage eStage, __exception)
{     
    return cAsyncDataTriggeredFilter::Shutdown(eStage, __exception_ptr);
}

tResult ScmJuryCommunication::OnAsyncPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{

    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);

    if (nEventCode == IPinEventSink::PE_MediaSampleReceived  )
    {
        // process the input from ActionStruct Input, received from SCM
		if (pSource == & m_ActionStructInputPin)
        {
			TActionStruct::ActionSub actionStruct_tmp;
			actionStruct_tmp = TActionStruct_object.Read_Action(pMediaSample,F_JURY_COMMUNICATION);
			ProcessActionData(actionStruct_tmp);

        }

        // process the request to the jury struct input pin
        else if (pSource == &m_JuryStructInputPin) 
        {
            tInt8 i8ActionID = -2;
            tInt16 i16entry = -1;
            
            {   // focus for sample read lock
                __adtf_sample_read_lock_mediadescription(m_pDescriptionJuryStruct,pMediaSample,pCoder);
               
                // get the IDs for the items in the media sample 
                if(!m_bIDsJuryStructSet)
                {
                    pCoder->GetID("i8ActionID", m_szIDJuryStructI8ActionID);
                    pCoder->GetID("i16ManeuverEntry", m_szIDJuryStructI16ManeuverEntry);
                    m_bIDsJuryStructSet = tTrue;
                }      

                pCoder->Get(m_szIDJuryStructI8ActionID, (tVoid*)&i8ActionID);
                pCoder->Get(m_szIDJuryStructI16ManeuverEntry, (tVoid*)&i16entry);              
            }
            //change the state depending on the input
            // action_GETREADY --> stateCar_READY
            // action_START --> stateCar_RUNNING
            // action_STOP --> stateCar_STARTUP
            switch (juryActions(i8ActionID))
            {
                case action_GETREADY: // no influence on system itself, so no communication with SCM necessary
                    if(m_bDebugModeEnabled)  LOG_WARNING(cString::Format("JCom: Received 'Request Ready with maneuver ID %d'",i16entry));
                    if(getCarState()==stateCar_STARTUP){
                    	if(getFlagManeuverListLoaded()){
                    		changeState(stateCar_READY);
                    		setManeuverID(i16entry);
                    	}else{
                    		LOG_ERROR(cString::Format("JCom: Received 'Request Ready', but maneuver list not loaded yet!"));
                    		changeState(stateCar_ERROR);
                    	}
                    }
                    else{
                    	LOG_WARNING(cString::Format("JCom: Received 'Request Ready', but system not in startup state! Thus command will be ignored."));
                    }
                    break;
                case action_START: // communication with SCM necessary, if button is pressed before awaited from SCM -> saved as flag!
                    if(m_bDebugModeEnabled)  LOG_WARNING(cString::Format("JCom: Received: 'Action START with maneuver ID %d'",i16entry));
                    /* (re)set previously executed maneuver to 'no_prev_man_existing' */
                    resetPreviousManeuver();
                    if(getFlagStartSignalReceived() == tFalse){ // false for first time receiving start; set to true afterwards; to false again if STOP is sent
                    	if (i16entry == getCurManeuverID()){
                    		changeState(stateCar_RUNNING);
                    	}
                    	else{
                    		LOG_WARNING(cString::Format("JCom: The id of the action_START corresponds not with the id of the last action_GETREADY! "
                    				"New ID %d will be used.",i16entry));
                    		setManeuverID(i16entry);
                    		changeState(stateCar_RUNNING);
                        }
                    	setFlagStartSignalReceived(tTrue);
                    	if(getFlagStartSignalRequested()){
							 /* Send feedback to SCM if it was requested before */
							TFeedbackStruct::Data feedback_tmp;
							feedback_tmp.ui32_filterID = F_JURY_COMMUNICATION;
							feedback_tmp.ui32_status = FB_JURY_COMMAND_RECEIVED_START;
							TransmitFeedbackStruct(feedback_tmp);
							setFlagStartSignalRequested(tFalse);
							if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("JCom: Sent requested feedback to SCM: ID %d, status %d",F_JURY_COMMUNICATION,FB_JURY_COMMAND_RECEIVED_START));
                    	}
                    }
                    else{
                    	if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("JCom: Action_START was received, but system already running due to previous start-signal!"));
                    }
                    break;
                case action_STOP: // communication with SCM over normal feedback for error-handling via Error-Maneuver in SCM
                    if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("JCom: Received: 'Stop with maneuver ID %d'",i16entry));
                    /* (re)set previously executed maneuver to 'no_prev_man_existing' */
				    resetPreviousManeuver();
                    setFlagStartSignalReceived(tFalse);
                    setFlagCompletedJuryList(tFalse);
                    changeState(stateCar_STARTUP);
                    /* Send feedback to SCM, will have priority-handling; SCM has to stop current maneuver and jump to startup mode, all filters disabled*/
                    TFeedbackStruct::Data feedback_tmp;
					feedback_tmp.ui32_filterID = F_JURY_COMMUNICATION;
					feedback_tmp.ui32_status = FB_JURY_COMMAND_RECEIVED_STOP;
					TransmitFeedbackStruct(feedback_tmp);
                    break;
            }
        }

        else if (pSource == &m_ManeuverListInputPin && m_pDescManeuverList != NULL)
        {

            {   // focus for sample read lock
                __adtf_sample_read_lock_mediadescription(m_pDescManeuverList,pMediaSample,pCoder);


                std::vector<tSize> vecDynamicIDs;

                // retrieve number of elements by providing NULL as first parameter
                tSize szBufferSize = 0;
                if(IS_OK(pCoder->GetDynamicBufferIDs(NULL, szBufferSize)))
                {
                    // create a buffer depending on the size element
                    tChar* pcBuffer = new tChar[szBufferSize];
                    vecDynamicIDs.resize(szBufferSize);
                    // get the dynamic ids (we already got the first "static" size element)
                    if (IS_OK(pCoder->GetDynamicBufferIDs(&(vecDynamicIDs.front()), szBufferSize)))
                    {
                        // iterate over all elements
                        for (tUInt32 nIdx = 0; nIdx < vecDynamicIDs.size(); ++nIdx)
                        {
                            // get the value and put it into the buffer
                            pCoder->Get(vecDynamicIDs[nIdx], (tVoid*)&pcBuffer[nIdx]);
                        }

                        // set the resulting char buffer to the string object
                        m_strManeuverFileString = (const tChar*) pcBuffer;
                    }

                    // cleanup the buffer
                    delete pcBuffer;
                }

            }

            // trigger loading maneuver list and update the ui
            loadManeuverList();

            /* Communicate with jury module about current car-state (supposedly STARTUP) */
            SendState(getCarState(),getCurManeuverID());

            /* List is supposed to be loaded only on startup, before maneuver-execution can be started;
             *  to trigger initialization and startup-mode of SCM, send FB_JURY_ALIVE_AND_READY-status */
            TFeedbackStruct::Data feedback_tmp;
			feedback_tmp.ui32_filterID = F_JURY_COMMUNICATION;
			feedback_tmp.ui32_status = FB_JURY_ALIVE_AND_READY;
			TransmitFeedbackStruct(feedback_tmp);
			if(m_bDebugModeEnabled) (cString::Format("JCom: Maneuverlist loaded, ALIVE_AND_READY-status sent to SCM."));

            // Check if first maneuver has already been requested by SCM
            if (getFlagFirstTransmitRequested()){
            	// get new/current maneuver from maneuverList
            	tUInt32 currMan_tmp = getCurrentManeuverFromJuryList();
				if(currMan_tmp == 0){
					RETURN_AND_LOG_ERROR_STR(ERR_INVALID_STATE, cString::Format("JCom: No valid Maneuver could be read from JuryManeuverList!"));
				}
				TFeedbackStruct::Data feedback_tmp;
				feedback_tmp.ui32_filterID = F_JURY_COMMUNICATION;
				feedback_tmp.ui32_status = currMan_tmp;
				TransmitFeedbackStruct(feedback_tmp);
				setFlagFirstTransmitRequested(tFalse);
			}
        }
    }
    RETURN_NOERROR;

}

/* Method that is called for every feedback that is received by feedback_Struct Pin;
 *   Implements the general structure and calls necessary functions for processing */
tResult ScmJuryCommunication::ProcessActionData(TActionStruct::ActionSub actionSub)
{
	TFeedbackStruct::Data feedback_tmp;
	feedback_tmp.ui32_filterID = F_JURY_COMMUNICATION;

	/* set feedback.status according to input actionSub */
	if(actionSub.command == AC_JURY_ALIVE_AND_READY){
		/* send back status to prove 'up and running' */
		feedback_tmp.ui32_status = FB_JURY_ALIVE_AND_READY;
		TransmitFeedbackStruct(feedback_tmp);
	}
	else if(actionSub.command == AC_JURY_MAN_GET_FIRST){
		/* check if maneuver list was already loaded */
		if(getFlagManeuverListLoaded()){
			/* get new/current maneuver from maneuverList */
			tUInt32 currMan_tmp = getCurrentManeuverFromJuryList();
			if(currMan_tmp == 0){
				RETURN_AND_LOG_ERROR_STR(ERR_INVALID_STATE, cString::Format("JCom: No valid Maneuver could be received from JuryManeuverList!"));
			}
			feedback_tmp.ui32_status = currMan_tmp;
			TransmitFeedbackStruct(feedback_tmp);
		}
		else{
			/* maneuver list not loaded yet, so do nothing until it is loaded;
			 *  after successful loading, feedback will be transmitted to SCM */
			/* set flag for requested transmit of first maneuver to true */
			 setFlagFirstTransmitRequested(tTrue);

			if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("JCom: first maneuver requested, but maneuver list not loaded yet!"));
		}
	}
	else if(actionSub.command == AC_JURY_MAN_GET_PREVIOUS){
		/* send back status to with previously executed maneuver */
		feedback_tmp.ui32_status = getPreviousManeuver();
		TransmitFeedbackStruct(feedback_tmp);
	}
	else if(actionSub.command == AC_JURY_MAN_FINISHED_GET_NEXT){
		/* before incrementing maneuver, save successfully executed maneuver as 'previous maneuver' */
		tUInt32 prevMan_tmp = getCurrentManeuverFromJuryList();
		if(prevMan_tmp == 0){
			RETURN_AND_LOG_ERROR_STR(ERR_INVALID_STATE, cString::Format("JCom: SavePreviousMan - No valid Maneuver could be received from JuryManeuverList!"));
		}
		setPreviousManeuver(prevMan_tmp);

		/* increment maneuver ID; returns retval < 0 in case of error, retval > 0 for completed list */
		tInt32 tmp_res = incrementManeuverID();
		if (tmp_res < 0){ // error occured
			RETURN_AND_LOG_ERROR_STR(ERR_END_OF_FILE,cString::Format("JCom: tried to increment maneuver ID, but failed."));
		}
		else if(tmp_res > 0){ // end of maneuverlist was reached
			if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("JCom: End of maneuverlist was reached! Switch to state COMPLETE."));
			setFlagCompletedJuryList(tTrue);
			changeState(stateCar_COMPLETE);
			feedback_tmp.ui32_status = FB_JURY_NO_MAN_REMAINING;
			TransmitFeedbackStruct(feedback_tmp);
		}
		else{ // ID could be incremented, tmp_res = 0 was returned
			/* get new/current maneuver from maneuverList */
			tUInt32 currMan_tmp = getCurrentManeuverFromJuryList();
			if(currMan_tmp == 0){
				RETURN_AND_LOG_ERROR_STR(ERR_INVALID_STATE, cString::Format("JCom: No valid Maneuver could be received from JuryManeuverList!"));
			}
			feedback_tmp.ui32_status = currMan_tmp;
			TransmitFeedbackStruct(feedback_tmp);
		}
	}
	else if(actionSub.command == AC_JURY_MAN_GET_FINISHING){
		/* get finishing maneuver from maneuverList, that means last one on the list (after state completed reached) */
		tUInt32 finishingMan_tmp = getFinishingManeuverFromJuryList();
		/* check if finishing maneuver (coded as tUInt32) can be received, that means != 0 */
		if(finishingMan_tmp == 0){
			RETURN_AND_LOG_ERROR_STR(ERR_INVALID_STATE, cString::Format("JCom: No valid finishing Maneuver could be received from JuryManeuverList, list might not be finished!"));
		}
		feedback_tmp.ui32_status = finishingMan_tmp;
		TransmitFeedbackStruct(feedback_tmp);

	}
	else if(actionSub.command == AC_JURY_SET_CARSTATE_ERROR_AND_RESET){
		/* call methods to change carState accordingly */
		changeState(stateCar_ERROR); // changes state, transmits to jury
		resetSection(); // after error, section has to be started from beginning
		TFeedbackStruct::Data feedback_tmp;
		feedback_tmp.ui32_filterID = F_JURY_COMMUNICATION;
		feedback_tmp.ui32_status = FB_JURY_CARSTATE_ERROR_SECTION_RESET;
		TransmitFeedbackStruct(feedback_tmp);

	}
	else if(actionSub.command == AC_JURY_WAIT_FOR_COMMAND_START){
		/* wait for the START command from jury, send feedback when received */
		if(getFlagStartSignalReceived()){
			TFeedbackStruct::Data feedback_tmp;
			feedback_tmp.ui32_filterID = F_JURY_COMMUNICATION;
			feedback_tmp.ui32_status = FB_JURY_COMMAND_RECEIVED_START;
			TransmitFeedbackStruct(feedback_tmp);
			/* Reset signal flag for next time */
			setFlagStartSignalReceived(tFalse);
		}
		else{
			/* Do nothing, as Start signal has not been received yet;
			 *  -> waiting for Jury to send action_START */
			if(m_bDebugModeEnabled)  LOG_WARNING(cString::Format("JCom: waiting for jury to send 'action_START'!"));
			setFlagStartSignalRequested(tTrue);
		}

	}

	RETURN_NOERROR;
}


tResult ScmJuryCommunication::TransmitFeedbackStruct(TFeedbackStruct::Data feedbackStruct){
	__synchronized_obj(m_oCriticalSectionTransmitFeedbackStruct);
	tTimeStamp cur_time = _clock->GetStreamTime();
	RETURN_IF_FAILED(TFeedbackStruct_object.Transmit(&m_FeedbackStructOutputPin,feedbackStruct,cur_time));
	if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("JCom: FeedbackStruct with ID %d and status %d successfully transmitted to SCM.",
			feedbackStruct.ui32_filterID,feedbackStruct.ui32_status));
	RETURN_NOERROR;
}



tResult ScmJuryCommunication::loadManeuverList()
{
	__synchronized_obj(m_oCriticalSectionManeuverList);
    m_sectorList.clear();
    cDOM oDOM;
    oDOM.FromString(m_strManeuverFileString);
    cDOMElementRefList oSectorElems;
    cDOMElementRefList oManeuverElems;
    if(m_bPrintStrucModeEnabled) LOG_WARNING(cString::Format("JCom: Received ManeuverList: "));
    //read first Sector Element
    if(IS_OK(oDOM.FindNodes("AADC-Maneuver-List/AADC-Sector", oSectorElems)))
    {                
        //iterate through sectors
        for (cDOMElementRefList::iterator itSectorElem = oSectorElems.begin(); itSectorElem != oSectorElems.end(); ++itSectorElem)
        {
            //if sector found
            tSector sector;
            sector.id = (*itSectorElem)->GetAttributeUInt32("id");
            if(m_bPrintStrucModeEnabled) LOG_WARNING(cString::Format("JCom: --> Sector %d",sector.id));

            if(IS_OK((*itSectorElem)->FindNodes("AADC-Maneuver", oManeuverElems)))
            {
                //iterate through maneuvers
                for(cDOMElementRefList::iterator itManeuverElem = oManeuverElems.begin(); itManeuverElem != oManeuverElems.end(); ++itManeuverElem)
                {
                    tAADC_Maneuver man;
                    man.id = (*itManeuverElem)->GetAttributeUInt32("id");
                    man.action = (*itManeuverElem)->GetAttribute("action");
                    sector.maneuverList.push_back(man);
                    if(m_bPrintStrucModeEnabled) LOG_WARNING(cString::Format("JCom: ----> Maneuver id %d, action: ", man.id) + man.action);
                }
            }
    
            m_sectorList.push_back(sector);
        }
    }
    if (oSectorElems.size() > 0)
    {
        LOG_WARNING(cString::Format("JCom: Transfer and loading procedure of maneuver file successful."));
        m_i16SectionListIndex = 0;
        m_i16ManeuverListIndex = 0;
        setFlagManeuverListLoaded(tTrue);
    }
    else
    {
        LOG_ERROR(cString::Format("JCom: No valid Maneuver Data found!"));
        m_i16SectionListIndex = -1;
        m_i16ManeuverListIndex =-1;
        RETURN_ERROR(ERR_INVALID_FILE);
    }
   
   
    RETURN_NOERROR;
}

/* Function to change or access information regarding the previously executed maneuver */
 tUInt32 ScmJuryCommunication::getPreviousManeuver(){
	 __synchronized_obj(m_oCriticalSectionPreviousManeuverAccess);
	 return tmpPrevManeuverInt_fb;
 }
 tResult ScmJuryCommunication::setPreviousManeuver(tUInt32 curManeuver){
	 __synchronized_obj(m_oCriticalSectionPreviousManeuverAccess);
	 tUInt32 tmp_prevManeuver = 0;
	 switch (curManeuver){
	 	 case FB_JURY_MAN_LEFT:
	 		 tmp_prevManeuver = FB_JURY_PREVIOUS_MAN_LEFT;
	 		 break;
	 	 case FB_JURY_MAN_RIGHT:
	 		 tmp_prevManeuver = FB_JURY_PREVIOUS_MAN_RIGHT;
	 		 break;
	 	 case FB_JURY_MAN_STRAIGHT:
	 		 tmp_prevManeuver = FB_JURY_PREVIOUS_MAN_STRAIGHT;
			 break;
	 	 case FB_JURY_MAN_PARALLEL_PARKING:
	 		 tmp_prevManeuver = FB_JURY_PREVIOUS_MAN_PARALLEL_PARKING;
			 break;
	 	 case FB_JURY_MAN_CROSS_PARKING:
	 		 tmp_prevManeuver = FB_JURY_PREVIOUS_MAN_CROSS_PARKING;
			 break;
	 	 case FB_JURY_MAN_PULL_OUT_LEFT:
	 		 tmp_prevManeuver = FB_JURY_PREVIOUS_MAN_PULL_OUT_LEFT;
	 		 break;
	 	 case FB_JURY_MAN_PULL_OUT_RIGHT:
	 		 tmp_prevManeuver = FB_JURY_PREVIOUS_MAN_PULL_OUT_RIGHT;
			 break;
	 	 default:// FB_JURY_NO_MAN_REMAINING or anything else -> reset
	 		 tmp_prevManeuver = FB_JURY_PREVIOUS_NO_MAN_EXISTING;
			 break;
	 }
	 tmpPrevManeuverInt_fb = tmp_prevManeuver;
	 RETURN_NOERROR;
 }
 tResult ScmJuryCommunication::resetPreviousManeuver(){
	 __synchronized_obj(m_oCriticalSectionPreviousManeuverAccess);
	 tmpPrevManeuverInt_fb = FB_JURY_PREVIOUS_NO_MAN_EXISTING;
	 RETURN_NOERROR;
 }


tBool ScmJuryCommunication::getFlagManeuverListLoaded(){
	__synchronized_obj(m_oCriticalSectionManListLoaded);
	return m_maneuverListLoaded;
}
tResult ScmJuryCommunication::setFlagManeuverListLoaded(tBool value){
	__synchronized_obj(m_oCriticalSectionManListLoaded);
	m_maneuverListLoaded = value;
	RETURN_NOERROR;
}

tBool ScmJuryCommunication::getFlagFirstTransmitRequested(){
	__synchronized_obj(m_oCriticalSectionFirstTransmitReq);
	return m_maneuverFirstTransmitrequested;
}
tResult ScmJuryCommunication::setFlagFirstTransmitRequested(tBool value){
	__synchronized_obj(m_oCriticalSectionFirstTransmitReq);
	m_maneuverFirstTransmitrequested = value;
	RETURN_NOERROR;
}

tBool ScmJuryCommunication::getFlagStartSignalRequested(){
	__synchronized_obj(m_oCriticalSectionStartSignalReq);
	return m_startSignalrequested;
}
tResult ScmJuryCommunication::setFlagStartSignalRequested(tBool value){
	__synchronized_obj(m_oCriticalSectionStartSignalReq);
	m_startSignalrequested = value;
	RETURN_NOERROR;
}
tBool ScmJuryCommunication::getFlagStartSignalReceived(){
	__synchronized_obj(m_oCriticalSectionStartSignalReceived);
	return m_startSignalreceived;
}
tResult ScmJuryCommunication::setFlagStartSignalReceived(tBool value){
	__synchronized_obj(m_oCriticalSectionStartSignalReceived);
	m_startSignalreceived = value;
	RETURN_NOERROR;
}

tBool ScmJuryCommunication::getFlagCompletedJuryList(){
	__synchronized_obj(m_oCriticalSectionCompletedJuryList);
	return m_bCompletedJuryList;
}
tResult ScmJuryCommunication::setFlagCompletedJuryList(tBool value){
	__synchronized_obj(m_oCriticalSectionCompletedJuryList);
	m_bCompletedJuryList = value;
	RETURN_NOERROR;
}

/* Method for decoding given maneuver from Jury list and returning corresponding predefined tUint32 value */
tUInt32 ScmJuryCommunication::decodeManeuverFromJuryList(cString tmp_juryManeuver){
	tUInt32 juryManeuverInt = 0;
	/* tInt CompareNoCase(const cString& strString) const; */
	if(tmp_juryManeuver.CompareNoCase("left") == 0)
	{
		/* action="left" */
		juryManeuverInt = FB_JURY_MAN_LEFT;
		if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("JCom: CurrentJuryManeuver: 'left'"));
	}
	else if(tmp_juryManeuver.CompareNoCase("right") == 0)
	{
		/* action="right" */
		juryManeuverInt = FB_JURY_MAN_RIGHT;
		 if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("JCom: CurrentJuryManeuver: 'right'"));
	}
	else if(tmp_juryManeuver.CompareNoCase("straight") == 0)
	{
		/* action="straight" */
		juryManeuverInt = FB_JURY_MAN_STRAIGHT;
		if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("JCom: CurrentJuryManeuver: 'straight'"));
	}
	else if(tmp_juryManeuver.CompareNoCase("parallel_parking") == 0)
	{
		/* action="parallel_parking" */
		juryManeuverInt = FB_JURY_MAN_PARALLEL_PARKING;
		if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("JCom: CurrentJuryManeuver: 'parallel_parking'"));
	}
//	else if(tmp_juryManeuver.CompareNoCase("cross_parking") == 0)
//	{
//		/* action="cross_parking" */
//		juryManeuverInt = FB_JURY_MAN_CROSS_PARKING;
//		if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("JCom: CurrentJuryManeuver: 'cross_parking'"));
//	}
    // --- handle cross parking with parking spot IDs
    else if(tmp_juryManeuver.StartsWith("cross_parking"))
    {
        // get the ID
        cString parkingSpotIDString = tmp_juryManeuver.Right(2);
        int parkingSpotID = (parkingSpotIDString.AsInt() - 1) % 4;

        switch (parkingSpotID)
        {
            case 0:
                juryManeuverInt = FB_JURY_CROSS_PARKING_SPOT_FOUR;
                break;

            case 1:
                juryManeuverInt = FB_JURY_CROSS_PARKING_SPOT_THREE;
                break;

            case 2:
                juryManeuverInt = FB_JURY_CROSS_PARKING_SPOT_TWO;
                break;

            case 3:
                juryManeuverInt = FB_JURY_CROSS_PARKING_SPOT_ONE;
                break;
        }

        /* action="cross_parking" */
        
        if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("JCom: CurrentJuryManeuver: 'cross_parking' in parking spot ID %d", parkingSpotID));
    }
    // ---
	else if(tmp_juryManeuver.CompareNoCase("pull_out_left") == 0)
	{
		/* action="pull_out_left" */
		juryManeuverInt = FB_JURY_MAN_PULL_OUT_LEFT;
		if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("JCom: CurrentJuryManeuver: 'pull_out_left'"));
	}
	else if(tmp_juryManeuver.CompareNoCase("pull_out_right") == 0)
	{
		/* action="pull_out_right */
		juryManeuverInt = FB_JURY_MAN_PULL_OUT_RIGHT;
		if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("JCom: CurrentJuryManeuver: 'pull_out_right'"));
	}
	else{
		LOG_ERROR(adtf_util::cString::Format("JCom: DecodeJuryManeuver: Maneuver could not be decoded, ERROR occurred!"));
	}

	return juryManeuverInt;
}

/* Method for returning the current maneuver from Jury list */
tUInt32 ScmJuryCommunication::getCurrentManeuverFromJuryList(){
	__synchronized_obj(m_oCriticalSectionManeuverList);
	if (getFlagManeuverListLoaded() == tFalse){
		LOG_ERROR( cString::Format("JCom: getCurrentManeuver -  ManeuverList not loaded, could not get next maneuver!"));
		return 0;
	}
	cString tmp_currJuryManeuver = " ";
	tUInt32 currentJuryManeuverInt = 0;

	tmp_currJuryManeuver = m_sectorList[m_i16SectionListIndex].maneuverList[m_i16ManeuverListIndex].action;
	currentJuryManeuverInt = decodeManeuverFromJuryList(tmp_currJuryManeuver);

	return currentJuryManeuverInt;
}

/* Method for returning the finishing maneuver from Jury list, that means the maneuver that was executed as the last maneuver on the list */
tUInt32 ScmJuryCommunication::getFinishingManeuverFromJuryList(){
	__synchronized_obj(m_oCriticalSectionManeuverList);
	if (getFlagManeuverListLoaded() == tFalse){
		LOG_ERROR( cString::Format("JCom: getFinishingManeuver -  ManeuverList not loaded, could not get finishing maneuver!"));
		return 0;
	}
	cString tmp_finishingJuryManeuver = " ";
	tUInt32 finishingJuryManeuverInt = 0;
	/* check if the maneuver-list was actually completed */
	if (getFlagCompletedJuryList()){
		/* after completing the maneuver list, the id is was NOT incremented anymore (avoiding index out of bounds) */
		tmp_finishingJuryManeuver = m_sectorList[m_i16SectionListIndex].maneuverList[m_i16ManeuverListIndex].action;
		finishingJuryManeuverInt = decodeManeuverFromJuryList(tmp_finishingJuryManeuver);
	}

	return finishingJuryManeuverInt;
}

/* Method for transmitting CarState and ManeuverEntry state to jury module */
tResult ScmJuryCommunication::SendState(stateCar state, tInt16 i16ManeuverEntry)
{
    __synchronized_obj(m_oCriticalSectionTransmitCarState);

    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    tInt8 bValue = tInt8(state);

    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize_DriverStruct));
    {   // focus for sample write lock
        __adtf_sample_write_lock_mediadescription(m_pDescriptionDriverStruct,pMediaSample,pCoder);

        // get the IDs for the items in the media sample
        if(!m_bIDsDriverStructSet)
        {
            pCoder->GetID("i8StateID", m_szIDDriverStructI8StateID);
            pCoder->GetID("i16ManeuverEntry", m_szIDDriverStructI16ManeuverEntry);
            m_bIDsDriverStructSet = tTrue;
        }

        pCoder->Set(m_szIDDriverStructI8StateID, (tVoid*)&bValue);
        pCoder->Set(m_szIDDriverStructI16ManeuverEntry, (tVoid*)&i16ManeuverEntry);
    }

    pMediaSample->SetTime(_clock->GetStreamTime());
    m_DriverStructOutputPin.Transmit(pMediaSample);

    //debug output to console
    if(m_bDebugSendStateModeEnabled)
    {
        switch (state)
            {
            case stateCar_ERROR:
                LOG_WARNING(cString::Format("JCom: Send state: ERROR, Maneuver ID %d",i16ManeuverEntry));
                break;
            case stateCar_READY:
                LOG_WARNING(cString::Format("JCom: Send state: READY, Maneuver ID %d",i16ManeuverEntry));
                break;
            case stateCar_RUNNING:
                 LOG_WARNING(cString::Format("JCom: Send state: RUNNING, Maneuver ID %d",i16ManeuverEntry));
                break;
            case stateCar_COMPLETE:
                LOG_WARNING(cString::Format("JCom: Send state: COMPLETE, Maneuver ID %d",i16ManeuverEntry));
                break;
            case stateCar_STARTUP:
            	LOG_WARNING(cString::Format("JCom: Send state: STARTUP."));
                break;
            }
    }

    RETURN_NOERROR;
}

/* Method for changing the State of the car; After change, new status is immediately sent to jury;
 *  just used for 'startup', 'error' ,'ready' and 'complete' */
tResult ScmJuryCommunication::changeState(stateCar newState)
{
	__synchronized_obj(m_oCriticalSectionCarStatus);
    // if state is the same do nothing
    if (m_CarState == newState) RETURN_NOERROR;
    
    // to secure the state is sent at least one time
    SendState(newState, getCurManeuverID());
   
    // handle the timer depending on the state
    switch (newState)
    {
    case stateCar_ERROR:
        destroyTimer();
        LOG_WARNING(cString::Format("JCom: State ERROR reached."));
        break;
    case stateCar_STARTUP:
        destroyTimer();
        LOG_WARNING(cString::Format("JCom: State STARTUP reached."));
        break;
    case stateCar_READY:
        createTimer();
        LOG_WARNING(cString::Format("JCom: State READY reached (ID %d).",getCurManeuverID()));
        break;
    case stateCar_RUNNING:
        if (m_CarState!=stateCar_READY)
            LOG_WARNING(cString::Format("JCom: Invalid state change to state 'RUNNING'. State 'READY' was not reached before. Nevertheless, change will be executed."));
        LOG_WARNING(cString::Format("JCom: State RUNNING reached."));
        break;
    case stateCar_COMPLETE:
        destroyTimer();
        LOG_WARNING(cString::Format("JCom: State COMPLETE reached."));
        break;
    }
    
    m_CarState = newState;
    RETURN_NOERROR;
}

stateCar ScmJuryCommunication::getCarState(){
	__synchronized_obj(m_oCriticalSectionCarStatus);
	return m_CarState;
}

/* Method returns the current maneuverID, that means the ID of the currently active maneuver that is being processed */
tInt16 ScmJuryCommunication::getCurManeuverID()
{
	__synchronized_obj(m_oCriticalSectionManeuverList);
	return m_i16CurrentManeuverID;
}


/* Methods increments the current maneuver ID after maneuver is successfully finished */
tInt32 ScmJuryCommunication::incrementManeuverID()
{
	__synchronized_obj(m_oCriticalSectionManeuverList);
	tInt32 retval = -1;
    // check if list was successfully loaded during initialization
    if (m_i16ManeuverListIndex!=-1 && m_i16SectionListIndex!=-1)
    {
    	retval = 0;
        // check if end of section is reached
    	// if NOT reached:
    	if (m_sectorList[m_i16SectionListIndex].maneuverList.size()>tUInt(m_i16ManeuverListIndex+1))
        {
            // increment only maneuver index, as there are still maneuvers to process in current section
            m_i16ManeuverListIndex++;
            // increment the current ManeuverID, representing the global position in list (above sector borders)
            m_i16CurrentManeuverID++;
        }
        else //end of section is reached
        {
            // end of section was reached and another section is in list
            if (m_sectorList.size() >tUInt(m_i16SectionListIndex+1))
            {
                //reset maneuver index to zero and increment section list index, since new sector is entered now
                m_i16SectionListIndex++;
                m_i16ManeuverListIndex = 0; // start at beginning of sector, with maneuverIndex zero
                m_i16CurrentManeuverID++; // just incremented, since it represents global position (corresponding to ID)
                if (m_sectorList[m_i16SectionListIndex].maneuverList[m_i16ManeuverListIndex].id !=m_i16CurrentManeuverID)
                {
                    LOG_ERROR("JCom: inconsistency in maneuverfile detected, IDs don't match. Please check the file!");
                }
            }
            else
            {
            	/* End of maneuver list is reached, all sectors completed */
            	if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("JCom: End of maneuverlist reached, cannot increment any more. List completed!"));
                retval = 1;
                /* Change carState to Completed, go to startup or Completed-maneuver in SCM! */
            }
        }
    }
    else
    {
        LOG_ERROR("JCom: could not set new maneuver ID because no maneuver list was loaded!");
    }
    
    if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("JCom: Increment Maneuver ID: Sectionindex is %d, Maneuverindex is %d, ID is %d",m_i16SectionListIndex,m_i16ManeuverListIndex,m_i16CurrentManeuverID));

    return retval;
}

/* Method for reseting section;
 *  After execution, the maneuver that has to be processed next is reset to the first maneuver in the current sector */
tResult ScmJuryCommunication::resetSection()
{
	__synchronized_obj(m_oCriticalSectionManeuverList);
    //maneuver list index to zero, and current maneuver id to first element in list 
    m_i16ManeuverListIndex=0;
    m_i16CurrentManeuverID = m_sectorList[m_i16SectionListIndex].maneuverList[m_i16ManeuverListIndex].id;
   
    if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("JCom: Reset section: Sectionindex is %d, Maneuverindex is %d, ID is %d",m_i16SectionListIndex,m_i16ManeuverListIndex,m_i16CurrentManeuverID));

    RETURN_NOERROR;
}

/* Method for directly setting the maneuver ID;
 *  searches saved maneuverlist for provided 'maneuverId' and sets the sectionListindex and ManeuverListIndex accordingly */
tResult ScmJuryCommunication::setManeuverID(tInt maneuverId)
{   
	__synchronized_obj(m_oCriticalSectionManeuverList);
    //look for the right section id and write it to section combobox
    for(unsigned int i = 0; i < m_sectorList.size(); i++)
    {
        for(unsigned int j = 0; j < m_sectorList[i].maneuverList.size(); j++)
        {
            if(maneuverId == m_sectorList[i].maneuverList[j].id)
            {            
                m_i16SectionListIndex = i;
                m_i16ManeuverListIndex = j;
                m_i16CurrentManeuverID = maneuverId;
                if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("JCom: Sectionindex is %d, Maneuverindex is %d, ID is %d",m_i16SectionListIndex,m_i16ManeuverListIndex,m_i16CurrentManeuverID));
                break;
            }
        }
    }    
    RETURN_NOERROR;
}

/* Method for creating a timer with name m_hTimer with 0.5 seconds */
tResult ScmJuryCommunication::createTimer()
{
     // creates timer with 0.5 sec
     __synchronized_obj(m_oCriticalSectionTimerSetup);
     // additional check necessary because input jury structs can be mixed up because every signal is sent three times
     if (m_hTimer == NULL)
     {
            m_hTimer = _kernel->TimerCreate(tTimeStamp(0.5*1000000), 0, static_cast<IRunnable*>(this),
                                        NULL, NULL, 0, 0, adtf_util::cString::Format("%s.timer", OIGetInstanceName()));
     }
     else
     {
        LOG_ERROR("JCom: Timer is already running. Unable to create a new one.");
     }
     RETURN_NOERROR;
}

/* Method for destroying the timer m_hTimer*/
tResult ScmJuryCommunication::destroyTimer(__exception)
{
    __synchronized_obj(m_oCriticalSectionTimerSetup);
    //destroy timer
    if (m_hTimer != NULL)
    {
        tResult nResult = _kernel->TimerDestroy(m_hTimer);
        if (IS_FAILED(nResult))
        {
            LOG_ERROR("JCom: Unable to destroy the timer.");
            THROW_ERROR(nResult);
        }
        m_hTimer = NULL;
    }
    //check if handle for some unknown reason still exists
    else
    {
        //LOG_WARNING("JCom: Timer handle not set, but I should destroy the timer. Try to find a timer with my name.");
        tHandle hFoundHandle = _kernel->FindHandle(adtf_util::cString::Format("%s.timer", OIGetInstanceName()));
        if (hFoundHandle)
        {
            tResult nResult = _kernel->TimerDestroy(hFoundHandle);
            if (IS_FAILED(nResult))
            {
                LOG_ERROR("JCom: Unable to destroy the specified timer.");
                THROW_ERROR(nResult);
            }
        }
    }

    RETURN_NOERROR;
}
