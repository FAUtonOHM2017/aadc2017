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


#ifndef _STATE_CONTROL_MANAGEMENT_SLIM_HEADER
#define _STATE_CONTROL_MANAGEMENT_SLIM_HEADER

#define OID_ADTF_STATE_CONTROL_MGMT_SLM "adtf.user.stateControlManagementSlim"

#include "stdafx.h"


class StateControlManagementSlim: public adtf::cAsyncDataTriggeredFilter
{
    ADTF_DECLARE_FILTER_VERSION(OID_ADTF_STATE_CONTROL_MGMT_SLM, "State Control Management Slim", OBJCAT_DataFilter, "State Control Management Slim", 1, 2, 0, "FAUtonOHM");


public: // construction
    StateControlManagementSlim(const tChar *);
    virtual ~StateControlManagementSlim();

protected:
    /* Object of type TActionStruct */
    TActionStruct TActionStruct_object;

    /* Object of type TFeedbackStruct */
    TFeedbackStruct TFeedbackStruct_object;

    /** INPUTS **/
    /*! input pin for communication with Filters  */
    cInputPin m_FilterFeedbackInputPin;

    /** OUTPUTS **/
    /*! output pin for ActionStruct, used for controlling all relevant filters */
    cOutputPin m_ActionStructOutputPin;

	// memory sizes of media samples to create
	tInt nSize_DriverStruct;			// size of DriverStruct

    /*! overrides cAsyncDataTriggeredFilter */
    virtual tResult Init(tInitStage eStage, __exception = NULL);
    /*! overrides cAsyncDataTriggeredFilter */
    virtual tResult Start(__exception = NULL);
    /*! overrides cAsyncDataTriggeredFilter */
    virtual tResult Stop(__exception = NULL);
    /*! overrides cAsyncDataTriggeredFilter */
    virtual tResult Shutdown(tInitStage eStage, __exception = NULL);

    /*! overrides cFilter, implements IPinEventSink */
    //tResult OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
    /*! overrides cAsyncDataTriggeredFilter, implements IPinEventSink */
    tResult OnAsyncPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample);
 	/*! creates all the input Pins*/
	tResult CreateInputPins(__exception = NULL);
	/*! creates all the output Pins*/
	tResult CreateOutputPins(__exception = NULL);


    /** StateControl Management Methods **/

    /* Function that is called for processing the data in activityLevel:Active of current SCM state;
     *  calls a transmit function to transmit necessary activation signals & commands to corresponding filters */
    tResult ExecAction();

    /* Function that is called for processing the data in activityLevel:Passive of current SCM state;
     *  sets/changes the current SCM state, sets i8CurrentActLvlMode, calls ExecAction afterwards
     *  with corresponding argument of new SCM state
     *   ** actual LOGIC ** : decodes the commands in feedback and executes the corresponding action */
    tResult ExecRequest(tUInt32 command);

    /* Function that is called for processing the data at every OnPinEvent of FeedbackPin;
     *  checks if current state is in passive mode: then feedback-input is requested and accepted;
     *  checks if filterID is in list of feedback that is requested at the moment;
     *  takes corresponding action by calling function*/
    tResult ProcessData(TFeedbackStruct::Data feedback);

    /* Function to check if received feedback data is relevant for current status (data that is waited for);
     *  if filterID is in list: method returns the command read from SCM-structure for (redundant) comparison (fault-detection) */
    tUInt32 CheckRequestRelevance(TFeedbackStruct::Data feedback);

    /* Function to check if current activityLevel is active, passive or not_initialized*/
    tInt8 CheckCurrentActivityLvlMode();
    /* Function to change current activityLevel to: active, passive or not_initialized*/
    tResult ChangeCurrentActivityLvlMode(tInt8 newMode);
    /* Function to transmit the Output Date */
    tResult TransmitActionStruct(TActionStruct::Data ActionStruct);
    /* Function to change current step ID*/
    tResult ChangeStep(tUInt32 numberOfSteps);
    /* Function to jump to step with scmStepID in current maneuver */
    tResult JumpToStep(tUInt32 scmStepID);
    /* Function to jump to maneuver with scmManeuverID */
    tResult JumpToManeuver(tUInt32 scmManeuverID);

    /** STATEMACHINE MANAGEMENT **/
    /* Variables to save current state of stateController*/
    tInt16 i16CurrentScmManeuverID;
    tInt16 i16CurrentScmStepID;
    /* Variable to save current activity level (NOT_INITIALIZED = -1, ACTIVE = 1, PASSIVE = 2)*/
    tInt8 i8CurrentActLvlMode;
    /* Variables necessary for mutex */
    /* see end of file */

    /** LOADING and saving StateControl Structure based on xml-file **/

    /* holds the xml-file of the StateControl structure list*/
    cFilename m_strStructureFileName;

    /* this is the list with all the loaded sections from the maneuver list*/
    std::vector<sc_Maneuver> m_SCMstructureList;

    /* reads the xml file which is set in the filter properties */
    tResult LoadSCMStructureData();
    /* ---------------------------- */


/** Variables, parameters and other necessary declarations **/

    /*! whether output to console is enabled or not*/
    tBool m_bDebugModeEnabled;
    /*! whether output of structure to console is enabled or not*/
    tBool m_bPrintStrucModeEnabled;

    /* file name for logfile, if data-logging is enabled */
    fstream scm_logfile;

    /** Deklaration of Critical Section Parameters **/
    /* Critical Section for Output Pins */
    /*! the critical section of the transmit of ActionStruct */
    cCriticalSection m_oCriticalSectionTransmitActionStruct;

    /* Critical Section for StateControlManagementSlim*/
    /*! the critical section of the CurrentActivityLvlMode */
    cCriticalSection CritSectionCurrentActivityLvlMode;
    /*! the critical section of the CurrentScmState */
    cCriticalSection CritSectionCurrentScmState;

};

#endif // _STATE_CONTROL_MANAGEMENT_SLIM_HEADER
