/**
 *
Copyright (c) 
Audi Autonomous Driving Cup. All rights reserved. Team FAUtonOHM.
 
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: �This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS �AS IS� AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

New Filter managing the car of team FAUtonOHM
**********************************************************************
* $Author:: mahill $   $Date:: 2016-02-09 13:49:07#$ $Rev:: 0.2.1   $
**********************************************************************/

#include "StateControlManagementSlim.h"

#define WRITE_DEBUG_OUTPUT_TO_FILE

ADTF_FILTER_PLUGIN("State Control Management Slim", OID_ADTF_STATE_CONTROL_MGMT_SLM, StateControlManagementSlim);



StateControlManagementSlim::StateControlManagementSlim(const tChar* __info) : cAsyncDataTriggeredFilter(__info), m_bDebugModeEnabled(tFalse)
{
    SetPropertyBool("Debug Output to Console",tFalse);
    SetPropertyStr("Debug Output to Console" NSSUBPROP_DESCRIPTION, "If enabled additional debug information is printed to the console (Warning: decreases performance)"); 

    SetPropertyBool("Print Structure to Console",tFalse);
    SetPropertyStr("Print Structure to Console" NSSUBPROP_DESCRIPTION, "If enabled the structure defined by xml-file is printed to the console (Warning: decreases performance)");


    SetPropertyStr("Configuration File For StateControlManagement","../../../../utilities/SCM_Structures/SCM_Structure_CURRENT.xml");
    SetPropertyBool("Configuration File For StateControlManagement" NSSUBPROP_FILENAME, tTrue);
    SetPropertyStr("Configuration File For StateControlManagement" NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, "XML Files (*.xml)");
    SetPropertyStr("Configuration File For StateControlManagement" NSSUBPROP_DESCRIPTION, "The XML defining the structure that is to be loaded has to be set here");

}

StateControlManagementSlim::~StateControlManagementSlim()
{
	scm_logfile.close();
}

tResult StateControlManagementSlim::CreateInputPins(__exception)
{
	//get the media description manager for this filter
	cObjectPtr<IMediaDescriptionManager> pDescManager;
	RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER, IID_ADTF_MEDIA_DESCRIPTION_MANAGER, (tVoid**)&pDescManager, __exception_ptr));

	/* Create Inputs for Different Filters, providing feedback to StateController */
	RETURN_IF_FAILED(m_FilterFeedbackInputPin.Create("Feedback_Struct",TFeedbackStruct_object.GetMediaType(), static_cast<IPinEventSink*> (this)));
	RETURN_IF_FAILED(RegisterPin(&m_FilterFeedbackInputPin));

	RETURN_NOERROR;
}

tResult StateControlManagementSlim::CreateOutputPins(__exception)
{
	/* Output Pin for ActionStruct to transmit activation signals to filters */
	RETURN_IF_FAILED(m_ActionStructOutputPin.Create("Action_Struct", TActionStruct_object.GetMediaType(), NULL));
	RETURN_IF_FAILED(RegisterPin(&m_ActionStructOutputPin));

	RETURN_NOERROR;
}

tResult StateControlManagementSlim::Init(tInitStage eStage, __exception)
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

        m_bDebugModeEnabled = GetPropertyBool("Debug Output to Console");
		#ifdef WRITE_DEBUG_OUTPUT_TO_FILE
		// creating logfile log file for SCM
        //LOG_WARNING("SCM: debug mode: 'logging' for scm-data enabled, writing to /opt/scm_log-'date-and-time'.log");
        time_t rawtime;
        struct tm * timeinfo;
        time(&rawtime);
        timeinfo = localtime(&rawtime);
//        cString fileDate = cString(asctime(timeinfo));
//        fileDate.Replace(" ", "_");
//        fileDate.Replace(":", "_");
//        cString logFilename = "/opt/scm_log_" + fileDate + ".log";
//        logFilename.Replace("\n", "");
//		  scm_logfile.open(logFilename, ios::out | ios::trunc);
//		  scm_logfile << "# SCM data log, recorded at: " << asctime(timeinfo) << std::endl;
		//scm_logfile.close();
        size_t N = 80;
        char dateAndTime[N];
        strftime(dateAndTime, N, "%Y_%m_%d-%H_%M_%S", timeinfo);

        cString dateAndTime_str = cString(dateAndTime);
        cString logFilename = "/tmp/scm_log_" + cString(dateAndTime) + ".log";
        logFilename.Replace("\n", "");
        LOG_WARNING("SCM: debug mode: 'logging' for scm-data enabled, writing to '" + logFilename + "'.");
		scm_logfile.open(logFilename, ios::out | ios::trunc);
		scm_logfile << "# SCM data log, recorded at: " << asctime(timeinfo) << std::endl;
		#endif

        /* StateController Management*/
        i16CurrentScmManeuverID = -1;
        i16CurrentScmStepID = -1;
        i8CurrentActLvlMode = NOT_INITIALIZED;


    }
    else if (eStage == StageNormal)
    {
    	//LOG_WARNING("StateControlManagement: StageNormal;");
    	m_bPrintStrucModeEnabled = GetPropertyBool("Print Structure to Console");
        // load xml files for defining the structure of the StateController
        tResult scmLoadstate = LoadSCMStructureData();
        if(scmLoadstate < 0) {
        	LOG_ERROR(adtf_util::cString::Format("SCM: Structure could not be loaded!"));
        }
        THROW_IF_FAILED(scmLoadstate);
        if(m_bDebugModeEnabled && scmLoadstate == 0){
        	LOG_WARNING(adtf_util::cString::Format("SCM: Structure loaded successfully!"));
        }

        /* Initialization of SCM after successfully loading structure*/
        i16CurrentScmManeuverID = 0;
        i16CurrentScmStepID = 0;

        /* START StateControl Management in startup-state, wait for first PASSIVE request to start,
         *  e.g. verification of loaded maneuver list from JuryCommunication */
        i8CurrentActLvlMode = PASSIVE;

        if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("SCM: initialized maneuverID, stepID and actLvlmode:Passive"));

    }
    else if(eStage == StageGraphReady)
    {
    	//LOG_WARNING("StateControlManagement: StageGraphReady;");

        // IDs for ActionStruct and FeedbackStruct are set
        RETURN_IF_FAILED(TActionStruct_object.StageGraphReady());
        RETURN_IF_FAILED(TFeedbackStruct_object.StageGraphReady());


        /* START StateControl Management in startup-state (maneuver 0, step 0) in passive mode, wait for response from JCom
         *  that maneuver list was successfully loaded */
        if(m_bDebugModeEnabled) LOG_WARNING(adtf_util::cString::Format("SCM: waiting for feedback from JCom regarding loaded maneuverlist."));
    }
    RETURN_NOERROR;
}

tResult StateControlManagementSlim::Start(__exception)
{
    return cAsyncDataTriggeredFilter::Start(__exception_ptr);
}

tResult StateControlManagementSlim::Stop(__exception)
{

    return cAsyncDataTriggeredFilter::Stop(__exception_ptr);
}

tResult StateControlManagementSlim::Shutdown(tInitStage eStage, __exception)
{     
    return cAsyncDataTriggeredFilter::Shutdown(eStage, __exception_ptr);
}

tResult StateControlManagementSlim::OnAsyncPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{

    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);

    if (nEventCode == IPinEventSink::PE_MediaSampleReceived  )
    {
    	//process the request to set state error pin
		if (pSource == & m_FilterFeedbackInputPin)
		{
			TFeedbackStruct::Data tmp_feedback;
			TFeedbackStruct_object.Read(pMediaSample,&tmp_feedback);
			ProcessData(tmp_feedback);
		}
    }
    RETURN_NOERROR;

}

/* Method is called for every feedback that is received by feedback_Struct Pin;
 *   Implements the general structure and calls necessary functions for processing */
tResult StateControlManagementSlim::ProcessData(TFeedbackStruct::Data feedback)
{
	/* HANDLING OF SPONTANEOUS INTERRUPTIONS BY JURY -> action_STOP signal sent! has to be treated in every mode */
	if(feedback.ui32_status == FB_JURY_COMMAND_RECEIVED_STOP){
		LOG_WARNING(cString::Format("SCM: Interrupted via action_STOP signal from jury! System will be set to startup mode."));
		#ifdef WRITE_DEBUG_OUTPUT_TO_FILE
		scm_logfile << std::endl << "### --- " << std::endl << "SCM: Interrupted via action_STOP signal from jury! System will be set to startup mode." << std::endl;
		#endif
		/* set SCM to startup maneuver, as it can be started again from this position via action_START from jury */
		JumpToManeuver(0);
		ChangeCurrentActivityLvlMode(ACTIVE);
		if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("SCM: Set to current ScmManeuverID %d, step %d, actLvl %d",i16CurrentScmManeuverID,i16CurrentScmStepID,CheckCurrentActivityLvlMode()));
		#ifdef WRITE_DEBUG_OUTPUT_TO_FILE
		scm_logfile << "SCM: Set to current ScmManeuverID: " << i16CurrentScmManeuverID << ", step: "<< i16CurrentScmStepID << ", actLvl: ACTIVE"<< std::endl <<"### --- " << std::endl;
		#endif
		ExecAction();
		/* all filters will be disabled in this way, except of JCom, which sends alive-signal;
		 *  afterwards, system is reset to beginning of current sector, is in CarState startup mode and
		 *  waiting for new Start-Signal from jury */
		RETURN_NOERROR;
	}

	/* regular, normal behaviour without interruptions by action_STOP */

	// check if current activity level is passive, so feedback is requested
	if(CheckCurrentActivityLvlMode() == ACTIVE){
		LOG_WARNING(cString::Format("SCM: System is in ACTIVE mode, but input registered: ID %d, status %d. Input will not be processed.",feedback.ui32_filterID,feedback.ui32_status));
	}else if(CheckCurrentActivityLvlMode() == PASSIVE){
		if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("SCM: In PASSIVE mode, Input registered: ID %d, status %d. Input will be checked for relevance.",feedback.ui32_filterID,feedback.ui32_status));

		// check if input status is relevant to current SCM-State, return command that should be executed
		tUInt32 scmStep_command = CheckRequestRelevance(feedback);
		// check if command was relevant and successfully decoded
		if (scmStep_command == 0){
			if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("SCM: Feedback status was not in list, thus will be ignored!"));
			#ifdef WRITE_DEBUG_OUTPUT_TO_FILE
			scm_logfile << "SCM: Feedback status was not in list, thus will be ignored!" << std::endl;
			#endif
		}else{
			// corresponding command will be executed
			if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("SCM: Feedback relevant, status will be processed."));
			#ifdef WRITE_DEBUG_OUTPUT_TO_FILE
			scm_logfile << "SCM: Feedback relevant, status will be processed." << std::endl << "---" << std::endl;
			#endif
			tResult tmpReturn = ExecRequest(scmStep_command);
			// check if ExecRequest was successful
			if(tmpReturn < 0){
				#ifdef WRITE_DEBUG_OUTPUT_TO_FILE
				scm_logfile << std::endl << "### --- " << std::endl << "SCM ERROR: ERROR occurred in ExecRequest!" << std::endl;
				#endif
				RETURN_AND_LOG_ERROR_STR(ERR_INVALID_ADDRESS,cString::Format("SCM: ERROR occurred in ExecRequest!"));
			}else{
				/* Change current activity level mode to ACTIVE, as passive command has been successfully executed*/
				ChangeCurrentActivityLvlMode(ACTIVE);
				ExecAction();
			}
		}

	}else{ // NOT_INITIALIZED or error occurred

		RETURN_AND_LOG_ERROR_STR(ERR_INVALID_STATE, cString::Format("SCM: ERROR occurred!"));
	}

	RETURN_NOERROR;
}

/* Decode and execute the corresponding command */
tResult StateControlManagementSlim::ExecRequest(tUInt32 scmStep_command){
	// execute the request command by changing the state of the StateController corresponding to scmStep_command
	/* decode the commands and react accordingly -> change tInt16 i16CurrentScmManeuverID and tInt16 i16CurrentScmStepID */
	/* Begin and End of defined intervals for commands specified in 'ScmCommunication.h' */
	tUInt32 stepPlusCommandOffset = C_STEP_PLUS_COMMAND_BEGIN;
	tUInt32 stepJumpCommandOffset = C_STEP_JUMP_COMMAND_BEGIN;
	tUInt32 maneuverCommandOffset = C_MAN_JUMP_COMMAND_BEGIN;
	tUInt32 maneuverCommandEnd = C_MAN_JUMP_COMMAND_END;

	if(scmStep_command >= stepPlusCommandOffset && scmStep_command < stepJumpCommandOffset){
		/* Increase step by certain number , starting from 0 to 99*/
		RETURN_IF_FAILED(ChangeStep(scmStep_command - stepPlusCommandOffset));
	}else if(scmStep_command >= stepJumpCommandOffset && scmStep_command < maneuverCommandOffset){
		/* Jump to certain step , starting from 100 */
		RETURN_IF_FAILED(JumpToStep(scmStep_command- stepJumpCommandOffset));
	}else if(scmStep_command >= maneuverCommandOffset && scmStep_command < maneuverCommandEnd){
		/* Jump to certain maneuver, starting from 200 */
		RETURN_IF_FAILED(JumpToManeuver(scmStep_command - maneuverCommandOffset));
	}else{
		#ifdef WRITE_DEBUG_OUTPUT_TO_FILE
		scm_logfile << std::endl << "### --- " << std::endl << "SCM ERROR: ERROR in ExecRequest(). Undefined Command. Check ENUM-file" << std::endl;
		#endif
		RETURN_AND_LOG_ERROR_STR(ERR_INVALID_STATE, cString::Format("SCM: ERROR in ExecRequest(). Undefined Command. Check ENUM-file"));
	}

	RETURN_NOERROR;
}

tResult StateControlManagementSlim::ChangeStep(tUInt32 numberOfSteps){
	__synchronized_obj(CritSectionCurrentScmState);
	tUInt32 stepCount =  m_SCMstructureList[i16CurrentScmManeuverID].Steps.size();
	if ((i16CurrentScmStepID + numberOfSteps) < stepCount){
		i16CurrentScmStepID = i16CurrentScmStepID + numberOfSteps;
	}else{
		#ifdef WRITE_DEBUG_OUTPUT_TO_FILE
		scm_logfile << std::endl << "### --- " << std::endl << "SCM ERROR: ERROR in changeStep(). Index exceeds." << std::endl;
		#endif
		RETURN_AND_LOG_ERROR_STR(ERR_INVALID_INDEX, cString::Format("SCM: ERROR in changeStep(). Index exceeds."));
	}
	RETURN_NOERROR;
}

tResult StateControlManagementSlim::JumpToStep(tUInt32 scmStepID){
	__synchronized_obj(CritSectionCurrentScmState);
	tUInt32 stepCount =  m_SCMstructureList[i16CurrentScmManeuverID].Steps.size();
	if (scmStepID < stepCount){
		i16CurrentScmStepID = scmStepID;
	}else{
		#ifdef WRITE_DEBUG_OUTPUT_TO_FILE
		scm_logfile << std::endl << "### --- " << std::endl << "SCM ERROR: ERROR in JumpStep(). Index exceeds." << std::endl;
		#endif
		RETURN_AND_LOG_ERROR_STR(ERR_INVALID_INDEX, cString::Format("SCM: ERROR in JumpStep(). Index exceeds."));
	}
	RETURN_NOERROR;
}


tResult StateControlManagementSlim::JumpToManeuver(tUInt32 scmManeuverID){
	__synchronized_obj(CritSectionCurrentScmState);
	tUInt32 maneuverCount =  m_SCMstructureList.size();
	if (scmManeuverID < maneuverCount){
		i16CurrentScmManeuverID = scmManeuverID;
		i16CurrentScmStepID = 0;
	}else{
		#ifdef WRITE_DEBUG_OUTPUT_TO_FILE
		scm_logfile << std::endl << "### --- " << std::endl << "SCM ERROR: ERROR in JumpToManeuver(). Index exceeds." << std::endl;
		#endif
		RETURN_AND_LOG_ERROR_STR(ERR_INVALID_INDEX, cString::Format("SCM: ERROR in JumpToManeuver(). Index exceeds."));
	}
	RETURN_NOERROR;
}

/* Function that is called for processing the data in activityLevel:Active of current SCM state;
 *  calls a transmit function to transmit necessary activation signals & commands to corresponding filters */
tResult StateControlManagementSlim::ExecAction(){
	__synchronized_obj(CritSectionCurrentScmState);
	if (CheckCurrentActivityLvlMode()==ACTIVE){
		if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("SCM: SCM in activityLvlMode ACTIVE, ExecAction() is executed."));
               // LOG_ERROR(cString::Format("SCM: SCM in activityLvlMode ACTIVE, ExecAction() is executed."));
		#ifdef WRITE_DEBUG_OUTPUT_TO_FILE
		scm_logfile << "---" << std::endl << "SCM: SCM in activityLvlMode ACTIVE, ExecAction() is executed." << std::endl;
		#endif
		/* create ActionStruct of type Data for sending purpose */
		TActionStruct::Data ActionStruct;
		std::vector<sc_Filter_active> tmp_filterlist = m_SCMstructureList[i16CurrentScmManeuverID].Steps[i16CurrentScmStepID].activityLvl.active.FilterList;
		/* Maximum number of Actions in ActionStruct is fixed to 5 at the moment */
		if(tmp_filterlist.size() > 5){
			#ifdef WRITE_DEBUG_OUTPUT_TO_FILE
			scm_logfile << std::endl << "### --- " << std::endl << "SCM ERROR: ERROR in ExecAction(). Too many actions in maneuver: " << i16CurrentScmManeuverID << ", step: " << i16CurrentScmStepID << std::endl;
			#endif
			RETURN_AND_LOG_ERROR_STR(ERR_INVALID_INDEX, cString::Format("SCM: ERROR in ExecAction(). Too many actions in maneuver %d, step %d.",
					i16CurrentScmManeuverID,i16CurrentScmStepID));
		}
		/* Maximum number of Actions in ActionStruct is now maximal 5 */
		TActionStruct::Action* tmp_ActionArr = new TActionStruct::Action[5];
		for(tUInt32 i = 0; i < tmp_filterlist.size(); i++){
			tmp_ActionArr[i].filterID = tmp_filterlist[i].filterID;
			tmp_ActionArr[i].subAction.enabled = tmp_filterlist[i].action.enabled;
			tmp_ActionArr[i].subAction.started = tmp_filterlist[i].action.started;
			tmp_ActionArr[i].subAction.command = tmp_filterlist[i].action.command;
			if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("SCM ExecAction: --> ID: %d, subAction.enabled: %d, subAction.started: %d, subAction.command: %d.",
					tmp_ActionArr[i].filterID,tmp_ActionArr[i].subAction.enabled,tmp_ActionArr[i].subAction.started, tmp_ActionArr[i].subAction.command));
			#ifdef WRITE_DEBUG_OUTPUT_TO_FILE
			scm_logfile << "SCM ExecAction: --> ID: " << tmp_ActionArr[i].filterID << ", subAction.enabled: " << tmp_ActionArr[i].subAction.enabled <<
					", subAction.started: " <<tmp_ActionArr[i].subAction.started << ", subAction.command: " << tmp_ActionArr[i].subAction.command << "." << std::endl;
			#endif
		}
		ActionStruct.action_1 = tmp_ActionArr[0];
		ActionStruct.action_2 = tmp_ActionArr[1];
		ActionStruct.action_3 = tmp_ActionArr[2];
		ActionStruct.action_4 = tmp_ActionArr[3];
		ActionStruct.action_5 = tmp_ActionArr[4];

		/* change activity level to passive, has to be done here in order to receive all signals/events! */
		ChangeCurrentActivityLvlMode(PASSIVE);
		/* call transmit function */
		tResult tmp_result = TransmitActionStruct(ActionStruct);
		delete [] tmp_ActionArr;
		if (tmp_result < 0){
			#ifdef WRITE_DEBUG_OUTPUT_TO_FILE
			scm_logfile << std::endl << "### --- " << std::endl << "SCM ERROR: ERROR occurred during transmitting the ActionStruct!" << std::endl;
			#endif
			RETURN_AND_LOG_ERROR_STR(ERR_FAILED,cString::Format("SCM: ERROR occurred during transmitting the ActionStruct!"));
		}

	}else{
		#ifdef WRITE_DEBUG_OUTPUT_TO_FILE
		scm_logfile << std::endl << "### --- " << std::endl << "SCM ERROR: ERROR occurred in ExecAction! ActivityLvlMode is NOT 'ACTIVE'!" << std::endl;
		#endif
		RETURN_AND_LOG_ERROR_STR(ERR_INVALID_STATE,cString::Format("SCM: ERROR occurred in ExecAction! ActivityLvlMode is NOT 'ACTIVE'!"));
	}
	RETURN_NOERROR;
}


/* If filterID is in request-list, and request-status in list is equal to input status,
 *  function returns feedback command corresponding to the filterID and status in FilterList
 *  Method is only called if StateController is in PASSIVE activity mode  */
tUInt32 StateControlManagementSlim::CheckRequestRelevance(TFeedbackStruct::Data feedback)
{
	__synchronized_obj(CritSectionCurrentScmState);
	tUInt32 ret_command = 0;
	// temporary copy of FilterList for current ScmState(Maneuver and step)
	std::vector<sc_Filter_passive> tmp_filterlist = m_SCMstructureList[i16CurrentScmManeuverID].Steps[i16CurrentScmStepID].activityLvl.passive.FilterList;
	for(tUInt32 i = 0; i < tmp_filterlist.size(); i++){
		if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("SCM - CheckRequestRelevance: awaited filterID %d, received filterID %d;",tmp_filterlist[i].filterID, feedback.ui32_filterID));
		#ifdef WRITE_DEBUG_OUTPUT_TO_FILE
		scm_logfile << "SCM - CheckRequestRelevance: awaited filterID "<< tmp_filterlist[i].filterID << ", received filterID "<<  feedback.ui32_filterID << std::endl;
		#endif
		if(tmp_filterlist[i].filterID == feedback.ui32_filterID){
			for(tUInt32 k = 0; k < tmp_filterlist[i].Requests.size(); k++){
				if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("SCM - CheckRequestRelevance: --> awaited status %d, received status %d;",tmp_filterlist[i].Requests[k].request, feedback.ui32_status));
				#ifdef WRITE_DEBUG_OUTPUT_TO_FILE
				scm_logfile << "SCM - CheckRequestRelevance: --> awaited status "<< tmp_filterlist[i].Requests[k].request << ", received status "<< feedback.ui32_status << std::endl;
				#endif
				if(tmp_filterlist[i].Requests[k].request == feedback.ui32_status){
					ret_command = tmp_filterlist[i].Requests[k].command;
					if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("SCM: CheckRequestRelevance successful, command: ",ret_command));
					#ifdef WRITE_DEBUG_OUTPUT_TO_FILE
					scm_logfile << "SCM: CheckRequestRelevance successful, command: "<< ret_command << std::endl;
					#endif
					return ret_command;
				}
			}
		}
	}
	#ifdef WRITE_DEBUG_OUTPUT_TO_FILE
	scm_logfile << "SCM: CheckRequestRelevance failed, status received but not in list: "<< feedback.ui32_status << std::endl;
	#endif
	if(m_bDebugModeEnabled)LOG_ERROR(cString::Format("SCM: CheckRequestRelevance failed, status received but not in list: %d",feedback.ui32_status));
	return ret_command;
}

tInt8 StateControlManagementSlim::CheckCurrentActivityLvlMode(){
	__synchronized_obj(CritSectionCurrentActivityLvlMode);
	return i8CurrentActLvlMode;
}

tResult StateControlManagementSlim::ChangeCurrentActivityLvlMode(tInt8 newMode){
	__synchronized_obj(CritSectionCurrentActivityLvlMode);
	i8CurrentActLvlMode = newMode;
	RETURN_NOERROR;
}

tResult StateControlManagementSlim::TransmitActionStruct(TActionStruct::Data ActionStruct){
	__synchronized_obj(m_oCriticalSectionTransmitActionStruct);
	tTimeStamp cur_time = _clock->GetStreamTime();
	RETURN_IF_FAILED(TActionStruct_object.Transmit(&m_ActionStructOutputPin,ActionStruct,cur_time));

      //  LOG_ERROR("StateMachine-Transmit");
	if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("SCM: ActionStruct successfully transmitted to filters."));
	#ifdef WRITE_DEBUG_OUTPUT_TO_FILE
	scm_logfile << "SCM: ActionStruct successfully transmitted to filters." << std::endl;
	#endif
	RETURN_NOERROR;
}

tResult StateControlManagementSlim::LoadSCMStructureData()
{
	// Get path of structure file
	m_strStructureFileName = GetPropertyStr("Configuration File For StateControlManagement");

    // check if file exits
    if (m_strStructureFileName.IsEmpty())
    {
		#ifdef WRITE_DEBUG_OUTPUT_TO_FILE
    	scm_logfile << std::endl << "### --- " << std::endl << "SCM ERROR: XML-File for structural configuration not found, please check property!" << std::endl;
		#endif
        RETURN_AND_LOG_ERROR_STR(ERR_INVALID_FILE,cString::Format("SCM: XML-File for structural configuration not found, please check property!"));
    }

    // create absolute path
    ADTF_GET_CONFIG_FILENAME(m_strStructureFileName);
    m_strStructureFileName = m_strStructureFileName.CreateAbsolutePath(".");

    //Load file, parse configuration, print the data

    if (cFileSystem::Exists(m_strStructureFileName))
    {
        cDOM oDOM;
        oDOM.Load(m_strStructureFileName);

        m_SCMstructureList.clear();

		cDOMElementRefList oScManeuverElems;
		cDOMElementRefList oScStepElems;
		cDOMElement*  oScLevelActiveElem;
		cDOMElement*  oScLevelPassiveElem;
		cDOMElementRefList oScFilterIDActiveElems;
		cDOMElementRefList oScFilterIDPassiveElems;
		cDOMElement*  oScActionElem;
		cDOMElementRefList  oScRequestElems;
		/* counters for checking if structural configuration file of StateControlManagement was correctly implemented by user, which means
		 * that both maneuvers and steps start at id=0 and are implemented in ascending order in the xml-file, with NO gaps existing between ids! */
		tUInt32 manCounter = 0;
		tUInt32 stepCounter = 0;

		//read first scManeuver Element
		if(IS_OK(oDOM.FindNodes("SCM-Structure-List/SCM-Maneuver", oScManeuverElems)))
		{
			//iterate through scManeuvers
			for (cDOMElementRefList::iterator itScManeuverElem = oScManeuverElems.begin(); itScManeuverElem != oScManeuverElems.end(); ++itScManeuverElem)
			{
				//if scManeuver found
				sc_Maneuver scManeuver; 	// declare object of scManeuver for saving purpose
				scManeuver.id = (*itScManeuverElem)->GetAttributeUInt32("id");
				if(manCounter != scManeuver.id){
					#ifdef WRITE_DEBUG_OUTPUT_TO_FILE
					scm_logfile << std::endl << "### --- " << std::endl << "SCM ERROR: Invalid structure of xml-file at maneuver " << scManeuver.id << ". Maneuvers must have ascending order and start at ID = 0. No gaps allowed. Check file!" << std::endl;
					#endif
					RETURN_AND_LOG_ERROR_STR(ERR_INVALID_FILE,cString::Format("SCM: Invalid structure of xml-file at maneuver %d. Maneuvers must have ascending order and start at ID = 0. No gaps allowed. Check file!",scManeuver.id));
				}

				if(m_bPrintStrucModeEnabled) LOG_WARNING(cString::Format("SCM: Maneuver %d ;",scManeuver.id));
                                //LOG_INFO(cString::Format("LINE 520 SCM: Maneuver %d ;",scManeuver.id));
                                //LOG_ERROR(cString::Format("LINE 521 SCM: Maneuver %d ;",scManeuver.id));

				if(IS_OK((*itScManeuverElem)->FindNodes("SCM-Step", oScStepElems)))
				{
					stepCounter = 0;
					//iterate through scSteps
					for(cDOMElementRefList::iterator itScStepsElem = oScStepElems.begin(); itScStepsElem != oScStepElems.end(); ++itScStepsElem)
					{
						//if scStep found
						sc_Step scStep;
						scStep.id = (*itScStepsElem)->GetAttributeUInt32("id");
						if(stepCounter != scStep.id){
							#ifdef WRITE_DEBUG_OUTPUT_TO_FILE
							scm_logfile << std::endl << "### --- " << std::endl << "SCM ERROR: Invalid structure of xml-file in maneuver " << scManeuver.id << ", step " << scStep.id << ". Maneuvers must have ascending order and start at ID = 0. No gaps allowed. Check file!" << std::endl;

                                                        #endif
                                                       // LOG_ERROR(cString::Format("LINE 538 SCM: Step  %d ;",scStep.id));
							RETURN_AND_LOG_ERROR_STR(ERR_INVALID_FILE,cString::Format("SCM: Invalid structure of xml-file in maneuver %d, step %d. Steps must have ascending order and start at ID = 0. No gaps allowed. Check file!",scManeuver.id,scStep.id));
						}
						if(m_bPrintStrucModeEnabled) LOG_WARNING(cString::Format("SCM: ---> Step %d ;",scStep.id));


						// PATH FOR ACTIVITYLEVEL: ACTIVE
						if(IS_OK((*itScStepsElem)->FindNode("SCM-ActivityLevel/SCM-Active", oScLevelActiveElem)))
						{
							if(m_bPrintStrucModeEnabled) LOG_WARNING(cString::Format("SCM: -----> ACTIVE:"));
                                                        //LOG_ERROR(cString::Format("SCM: -----> ACTIVE:"));
							//if scActivityLevel_ACTIVE found
							if(IS_OK(oScLevelActiveElem->FindNodes("SCM-FilterID", oScFilterIDActiveElems)))
							{
								//iterate through scFilterID_Active
								for(cDOMElementRefList::iterator itScFilterIDActiveElem = oScFilterIDActiveElems.begin(); itScFilterIDActiveElem != oScFilterIDActiveElems.end(); ++itScFilterIDActiveElem)
								{
									//if ScFilterID_Active found, Element of List FilterList active
									sc_Filter_active scFilterActive;
									scFilterActive.filterID= (*itScFilterIDActiveElem)->GetAttributeUInt32("id");
									//if(m_bPrintStrucModeEnabled) LOG_WARNING(cString::Format("SCM: -------> FilterID %d ;",scFilterActive.filterID));
                                                                       // LOG_ERROR(cString::Format("SCM: -------> FilterID %d ;",scFilterActive.filterID));
									// inside ID, there is only SCM-Action, so only search for specific node
									if(IS_OK((*itScFilterIDActiveElem)->FindNode("SCM-Action", oScActionElem)))
										{
												scFilterActive.action.enabled = oScActionElem->GetAttributeBool("enable");
												//if(m_bPrintStrucModeEnabled) LOG_WARNING(cString::Format("SCM: ---------> enable %d ;",scFilterActive.action.enabled));
												scFilterActive.action.started = oScActionElem->GetAttributeBool("start");
												//if(m_bPrintStrucModeEnabled) LOG_WARNING(cString::Format("SCM: ---------> start %d ;",scFilterActive.action.started));
												scFilterActive.action.command = oScActionElem->GetAttributeUInt32("command");
												//if(m_bPrintStrucModeEnabled) LOG_WARNING(cString::Format("SCM: ---------> command %d ;",scFilterActive.action.command));

                                                                                               // LOG_ERROR(cString::Format("LINE 566    SCM: ---------> enable %d ;",scFilterActive.action.enabled));
                                                                                             //   LOG_ERROR(cString::Format("LINE 567    SCM: ---------> start %d ;",scFilterActive.action.started));
                                                                                               // LOG_ERROR(cString::Format("LINE 568    SCM: ---------> command %d ;",scFilterActive.action.command));
										}
									// Push found FilterActive Commands into Filter-Active-list for current step
									scStep.activityLvl.active.FilterList.push_back(scFilterActive);

								}
							}

						}

						// PATH FOR ACTIVITYLEVEL: PASSIVE
						if(IS_OK((*itScStepsElem)->FindNode("SCM-ActivityLevel/SCM-Passive", oScLevelPassiveElem)))
						{
							if(m_bPrintStrucModeEnabled) LOG_WARNING(cString::Format("SCM: -----> PASSIVE:"));
                                                        //LOG_ERROR(cString::Format("SCM: -----> PASSIVE:"));
							//if scActivityLevel_PASSIVE found
							if(IS_OK(oScLevelPassiveElem->FindNodes("SCM-FilterID", oScFilterIDPassiveElems)))
							{
								//iterate through scFilterID_Passive
								for(cDOMElementRefList::iterator itScFilterIDPassiveElem = oScFilterIDPassiveElems.begin(); itScFilterIDPassiveElem != oScFilterIDPassiveElems.end(); ++itScFilterIDPassiveElem)
								{
									//if ScFilterID_Passive found, Element of List FilterList passive
									sc_Filter_passive scFilterPassive;
									scFilterPassive.filterID= (*itScFilterIDPassiveElem)->GetAttributeUInt32("id");
									//if(m_bPrintStrucModeEnabled) LOG_WARNING(cString::Format("SCM: -------> FilterID %d ;",scFilterPassive.filterID));
                                                                       // LOG_ERROR(cString::Format("LINE 592    SCM: -------> FilterID %d ;",scFilterPassive.filterID));

									// inside ID, there can be several SCM-Requests
									if(IS_OK((*itScFilterIDPassiveElem)->FindNodes("SCM-Request", oScRequestElems)))
										{
										//iterate through scFilterID_Passive
											for(cDOMElementRefList::iterator itScReqeustElems = oScRequestElems.begin(); itScReqeustElems != oScRequestElems.end(); ++itScReqeustElems)
											{
												//if sc_Request found, Element of List Request
												sc_Request scRequest;
												scRequest.request = (*itScReqeustElems)->GetAttributeUInt32("status");
												scRequest.command = (*itScReqeustElems)->GetAttributeUInt32("command");

												scFilterPassive.Requests.push_back(scRequest);
                                                                                                //LOG_ERROR(cString::Format("SCM: ---------> status %d, command %d ;",scRequest.request,scRequest.command));
												//if(m_bPrintStrucModeEnabled) LOG_WARNING(cString::Format("SCM: ---------> status %d, command %d ;",scRequest.request,scRequest.command));
                                                                                                //LOG_ERROR(cString::Format("LINE 608    SCM: ---------> status %d, command %d ;",scRequest.request,scRequest.command));
											}

										}
									// Push found FilterPassive Commands into Filter-Passive-list for current step
									scStep.activityLvl.passive.FilterList.push_back(scFilterPassive);

								}
							}

						}
						// Push found steps into step-list
						scManeuver.Steps.push_back(scStep);
						stepCounter++; // increase step counter
					}
				}

				m_SCMstructureList.push_back(scManeuver);
				manCounter++; // increase maneuver counter
			}
		}

		if (oScManeuverElems.size() > 0)
		{
			if(m_bDebugModeEnabled) LOG_WARNING(cString::Format("SCM: Loaded Structural Configuration file successfully."));
			#ifdef WRITE_DEBUG_OUTPUT_TO_FILE
			scm_logfile << "SCM: Loaded Structural Configuration file successfully." << std::endl;
			#endif
		}
		else
		{
			#ifdef WRITE_DEBUG_OUTPUT_TO_FILE
			scm_logfile << std::endl << "### --- " << std::endl << "SCM ERROR: no valid Configuration Data found in file!" << std::endl;
			#endif
			RETURN_AND_LOG_ERROR_STR(ERR_INVALID_FILE, cString::Format("SCM: no valid Configuration Data found in file!"));
		}
    }
    else
	{
		#ifdef WRITE_DEBUG_OUTPUT_TO_FILE
		scm_logfile << std::endl << "### --- " << std::endl << "SCM ERROR: Structural configuration file not found!" << std::endl;
		#endif
		RETURN_AND_LOG_ERROR_STR(ERR_INVALID_FILE,cString::Format("SCM: Structural configuration file not found!"));
	}


    RETURN_NOERROR;
}
